#include <Arduino.h>
#include <cmath>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <Stepper.h>   // kept for structure compatibility (we don't block with it)

#include "../config/spin.h"
#include "../config/digging.h"

// ------------------------ Helpers ------------------------
#define RCCHECK(fn)           { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); } }
#define RCSOFTCHECK(fn)       { rcl_ret_t temp_rc = fn; (void)temp_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

// ------------------------ ROS entities ------------------------
rcl_publisher_t debug_spin_publisher;
geometry_msgs__msg__Twist debug_spin_msg;

rcl_publisher_t debug_drill_publisher;
geometry_msgs__msg__Twist debug_drill_msg;

rcl_publisher_t debug_gripper_publisher;
geometry_msgs__msg__Twist debug_gripper_msg;

rcl_subscription_t spin_subscriber;     // /quin/cmd_spin
geometry_msgs__msg__Twist spin_msg;

rcl_subscription_t drill_subscriber;    // /quin/cmd_drill
geometry_msgs__msg__Twist drill_msg;

rcl_subscription_t gripper_subscriber;  // /quin/cmd_gripper
geometry_msgs__msg__Twist gripper_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long current_time = 0;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

// Edge-detection memory for spin direction: -1 = last LEFT, 0 = neutral, 1 = RIGHT
int last_dir = 0;

// Keep Stepper objects for compatibility with your configs (we do NOT use blocking .step())
Stepper spinStepper(STEPS_PER_REV, SPIN_STEP_PIN_PLS, SPIN_STEP_PIN_DIR);
Stepper drillStepper(STEPS_PER_CM,  DRILL_STEP_PIN_PLS, DRILL_STEP_PIN_DIR);

Servo myServo;

// ==========================================================
//               Non-blocking Step Pulse Engine
// ----------------------------------------------------------
// This tiny state machine outputs STEP pulses without delay()
// or delayMicroseconds(). Each call to tick() advances the
// HIGH/LOW phases only when the scheduled micros() time arrives.
// That means the ROS executor is never blocked.
// ==========================================================
struct StepPulseEngine {
  uint8_t  pin_step;    // STEP pin
  uint8_t  pin_dir;     // DIR pin
  volatile long steps_left; // remaining steps in current job
  uint8_t  phase;       // 0=idle, 1=HIGH phase, 2=LOW phase
  uint32_t t_next;      // micros() timestamp when to change phase
  uint32_t t_high_us;   // pulse HIGH width
  uint32_t t_low_us;    // pulse LOW width (period - high)

  void begin(uint8_t step_pin, uint8_t dir_pin, uint32_t high_us, uint32_t period_us) {
    pin_step = step_pin; pin_dir = dir_pin;
    pinMode(pin_step, OUTPUT);
    pinMode(pin_dir, OUTPUT);
    digitalWrite(pin_step, LOW);
    steps_left = 0;
    phase = 0;
    t_next = 0;
    t_high_us = high_us;
    t_low_us  = (period_us > high_us) ? (period_us - high_us) : 1; // prevent 0
  }

  // Queue a new motion “segment” with direction and number of steps
  // If the engine is idle, it starts immediately; otherwise the steps add up.
  void enqueue(long n_steps, bool dir_high) {
    if (n_steps <= 0) return;
    digitalWrite(pin_dir, dir_high ? HIGH : LOW);
    steps_left += n_steps;
    if (phase == 0) {                          // start right away if idle
      phase = 1;
      digitalWrite(pin_step, HIGH);
      t_next = micros() + t_high_us;
    }
  }

  // Advance the pulse machine; must be called frequently (e.g., every control tick)
  void tick() {
    if (steps_left <= 0) { phase = 0; return; }
    uint32_t now = micros();

    if (phase == 1) {                           // HIGH phase
      if ((int32_t)(now - t_next) >= 0) {
        digitalWrite(pin_step, LOW);
        phase = 2;
        t_next = now + t_low_us;
      }
    } else if (phase == 2) {                    // LOW phase
      if ((int32_t)(now - t_next) >= 0) {
        steps_left--;                           // one full step completed
        if (steps_left > 0) {                   // start next step
          digitalWrite(pin_step, HIGH);
          phase = 1;
          t_next = now + t_high_us;
        } else {
          phase = 0;                            // job finished
        }
      }
    }
  }

  bool busy() const { return steps_left > 0 || phase != 0; }
};

// One engine per stepper axis
StepPulseEngine spinPE;
StepPulseEngine drillPE;

// ------------------------ Declarations ------------------------
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
struct timespec getTime();
void Spin();
void Drill();
void Gripper();

// ------------------------ setup/loop ------------------------
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // Prepare STEP/DIR pins (LOW default)
  pinMode(SPIN_STEP_PIN_PLS, OUTPUT);
  pinMode(SPIN_STEP_PIN_DIR, OUTPUT);
  pinMode(DRILL_STEP_PIN_PLS, OUTPUT);
  pinMode(DRILL_STEP_PIN_DIR, OUTPUT);
  digitalWrite(SPIN_STEP_PIN_PLS, LOW);
  digitalWrite(DRILL_STEP_PIN_PLS, LOW);

  // Initialize non-blocking engines using your timing constants
  spinPE.begin(SPIN_STEP_PIN_PLS,  SPIN_STEP_PIN_DIR,  STEP_PULSE_HIGH_US, STEP_PERIOD_US);
  drillPE.begin(DRILL_STEP_PIN_PLS, DRILL_STEP_PIN_DIR, STEP_PULSE_HIGH_US, STEP_PERIOD_US);

  // Keep your original Stepper API speed (not used for motion here)
  spinStepper.setSpeed(STEPPER_RPM);
  drillStepper.setSpeed(STEPPER_RPM);

  // IMPORTANT: attach servo pin from your config (ensure SERVO_PIN exists)
  myServo.attach(SERVO_PIN, 500, 2500);
  myServo.write(SERVO_CLOSED);
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      // Periodically ping the agent while not connected
      EXECUTE_EVERY_N_MS(1500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 10)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      // Create all ROS entities; if it fails, go back to waiting
      state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) { destroyEntities(); }
      break;

    case AGENT_CONNECTED:
      // Keep pinging the agent to avoid timeouts
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 10)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED) {
        // Run executor briefly; our motion is non-blocking so this remains timely
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;

    case AGENT_DISCONNECTED:
      // Clean up everything when disconnected
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }
}

// ------------------------ control timer callback ------------------------
// Runs every ~20 ms (see createEntities). We do three things:
//   1) Publish debug quickly (so UI feels instant even while motion continues)
//   2) Process commands (edge detection / enqueue work)
//   3) Advance step engines by time slices (non-blocking)
void controlCallback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (!timer) return;

  publishData();     // (1) quick debug first
  Spin();            // (2) react to spin command (edge-based)
  Drill();           // (2) toggle down/up on press edge
  Gripper();         // (2) open/close with thresholds
  spinPE.tick();     // (3) drive servo loops without blocking
  drillPE.tick();
}

// ------------------------ Subscribers ------------------------
void twistCallback(const void *msgin) {
  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;
  prev_cmd_time = millis();
  spin_msg.angular.z = msg->angular.z; // use angular.z for spin direction
}

void twist2Callback(const void *msgin) {
  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;
  prev_cmd_time = millis();
  drill_msg.linear.z = msg->linear.z;  // use linear.z for drill press
}

void twist3Callback(const void *msgin) {
  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;
  prev_cmd_time = millis();
  gripper_msg.linear.x = msg->linear.x; // use linear.x for gripper state
}

// ------------------------ Entity lifecycle ------------------------
bool createEntities() {
  allocator = rcl_get_default_allocator();
  geometry_msgs__msg__Twist__init(&debug_spin_msg);
  geometry_msgs__msg__Twist__init(&debug_drill_msg);
  geometry_msgs__msg__Twist__init(&debug_gripper_msg);

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 77);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  RCCHECK(rclc_node_init_default(&node, "quin_robot_node", "", &support));

  // Debug publishers (Best Effort is fine for telemetry)
  RCCHECK(rclc_publisher_init_best_effort(
    &debug_spin_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/quin/debug/spin"));

  RCCHECK(rclc_publisher_init_best_effort(
    &debug_drill_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/quin/debug/drill"));

  RCCHECK(rclc_publisher_init_best_effort(
    &debug_gripper_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/quin/debug/gripper"));

  // Command subscribers
  RCCHECK(rclc_subscription_init_best_effort(
    &spin_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/quin/cmd_spin"));

  RCCHECK(rclc_subscription_init_best_effort(
    &drill_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/quin/cmd_drill"));

  RCCHECK(rclc_subscription_init_best_effort(
    &gripper_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/quin/cmd_gripper"));

  // 20 ms control loop (non-blocking inside)
  const unsigned int control_timeout = 20;
  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(control_timeout), controlCallback));

  // Executor holds 3 subs + 1 timer (we allocate 5 for headroom)
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &spin_subscriber,    &spin_msg,    &twistCallback,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &drill_subscriber,   &drill_msg,   &twist2Callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &gripper_subscriber, &gripper_msg, &twist3Callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  syncTime();
  return true;
}

bool destroyEntities() {
  rcl_publisher_fini(&debug_spin_publisher, &node);
  rcl_subscription_fini(&spin_subscriber, &node);
  rcl_publisher_fini(&debug_drill_publisher, &node);
  rcl_subscription_fini(&drill_subscriber, &node);
  rcl_publisher_fini(&debug_gripper_publisher, &node);
  rcl_subscription_fini(&gripper_subscriber, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
  return true;
}

// ------------------------ Command layers ------------------------
// SPIN: rotate tray by exactly 36° per *edge* in the direction of input.
// Uses edge detection so holding the stick won’t re-trigger until you return to deadzone.
void Spin() {
  const float z = spin_msg.angular.z;

  if (z > TWIST_THRESH && last_dir != 1 && !spinPE.busy()) {
    // clockwise: DIR=HIGH
    spinPE.enqueue(STEPS_PER_36, true);
    last_dir = 1;
  } else if (z < -TWIST_THRESH && last_dir != -1 && !spinPE.busy()) {
    // counter-clockwise: DIR=LOW
    spinPE.enqueue(STEPS_PER_36, false);
    last_dir = -1;
  } else if (fabs(z) < TWIST_DEADZONE) {
    last_dir = 0; // reset edge state when neutral
  }

  debug_spin_msg.angular.z = spin_msg.angular.z;
}

// DRILL: press -> go down a fixed travel; next press -> go up the same travel.
// Non-blocking: we enqueue the total steps once, then the engine clocks them out.
void Drill() {
  static int  move_state = 0;      // 0=idle, 1=down next, -1=up next
  static bool prev_pressed = false;

  const float z = drill_msg.linear.z;
  bool pressed_now = (fabs(z) > JOY_PRESS_THRESH);
  bool rising_edge = pressed_now && !prev_pressed;

  if (rising_edge && !drillPE.busy()) {
    long total_steps = (long)(TRAVEL_CM * STEPS_PER_CM);
    if (move_state != 1) {
      // Next move = DOWN
      move_state = 1;
      digitalWrite(DRILL_STEP_PIN_DIR, HIGH);
      drillPE.enqueue(total_steps, true);
    } else {
      // Next move = UP
      move_state = -1;
      digitalWrite(DRILL_STEP_PIN_DIR, LOW);
      drillPE.enqueue(total_steps, false);
    }
  }

  prev_pressed = pressed_now;
  debug_drill_msg.linear.z = drill_msg.linear.z;
}

// GRIPPER: thresholds instead of exact float equality (1.0, 2.0).
void Gripper() {
  if (gripper_msg.linear.x > 1.5f) {
    myServo.write(SERVO_OPENED);
  } else if (gripper_msg.linear.x > 0.5f) {
    myServo.write(SERVO_CLOSED);
  }
  debug_gripper_msg.linear.x = gripper_msg.linear.x;
}

// ------------------------ Debug publish ------------------------
void publishData() {
  // Publishing first in the control cycle makes UI responsive even while motion continues
  rcl_publish(&debug_spin_publisher,    &debug_spin_msg,    NULL);
  rcl_publish(&debug_drill_publisher,   &debug_drill_msg,   NULL);
  rcl_publish(&debug_gripper_publisher, &debug_gripper_msg, NULL);
}

// ------------------------ Time sync ------------------------
void syncTime() {
  struct timespec ts; ts.tv_sec = 0; ts.tv_nsec = 0;
  // Wait until agent gives us a non-zero time
  do { ts = getTime(); delay(10); } while (ts.tv_sec == 0 && ts.tv_nsec == 0);

  unsigned long long now_millis =
    (unsigned long long)ts.tv_sec * 1000ULL + (unsigned long long)(ts.tv_nsec / 1000000ULL);
  time_offset = now_millis - millis();

  Serial.print("Synchronized time: ");
  Serial.print(ts.tv_sec);  Serial.print(" sec, ");
  Serial.print(ts.tv_nsec); Serial.println(" nsec");
}

struct timespec getTime() {
  struct timespec tp = {0};
  unsigned long long now = millis() + time_offset;
  tp.tv_sec  = now / 1000ULL;
  tp.tv_nsec = (now % 1000ULL) * 1000000ULL;
  return tp;
}

void rclErrorLoop() {
  // Minimal recovery: reboot the MCU so the agent can reconnect cleanly
  ESP.restart();
}
