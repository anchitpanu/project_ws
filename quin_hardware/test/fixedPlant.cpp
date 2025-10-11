// ============================================================================
// ESP32 micro-ROS: Spin Tray + Drill (Rack&Pinion) + Gripper Servo  (ISR-based)
// - Spin & Drill use HW Timer ISRs to generate step pulses (no blocking).
// - Edge-triggered spin: ±36° per press (STEPS_PER_36).
// - Drill toggle: first press DOWN (TRAVEL_CM*STEPS_PER_CM), next press UP.
// - Gripper: thresholded open/close (no float equality).
// - Debug topics are rate-limited to avoid serial congestion.
// - All pins/speeds/thresholds come from ../config/spin.h and ../config/digging.h
// ============================================================================

#include <Arduino.h>
#include <cmath>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <ESP32Servo.h>

#include "../config/spin.h"     // expects: SPIN_STEP_PIN_PLS, SPIN_STEP_PIN_DIR, STEPS_PER_36, STEP_PULSE_HIGH_US, STEP_PERIOD_US, TWIST_THRESH, TWIST_DEADZONE
#include "../config/digging.h"  // expects: DRILL_STEP_PIN_PLS, DRILL_STEP_PIN_DIR, TRAVEL_CM, STEPS_PER_CM, JOY_PRESS_THRESH, SERVO_OPENED, SERVO_CLOSED, SERVO_PIN (optional)

// Fast GPIO
#include "driver/gpio.h"
// HW timers
#include "esp32-hal-timer.h"

// ----------------- Safety fallbacks (if not defined in your configs) -----------------
#ifndef SERVO_PIN
#define SERVO_PIN 26
#endif
#ifndef STEP_PULSE_HIGH_US
#define STEP_PULSE_HIGH_US 8
#endif
#ifndef STEP_PERIOD_US
#define STEP_PERIOD_US 800
#endif

// ----------------- ROS entities -----------------
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     control_timer;

rcl_subscription_t spin_sub;      // /quin/cmd_spin      (Twist.angular.z)
rcl_subscription_t drill_sub;     // /quin/cmd_drill     (Twist.linear.z)
rcl_subscription_t grip_sub;      // /quin/cmd_gripper   (Twist.linear.x)
geometry_msgs__msg__Twist spin_cmd, drill_cmd, grip_cmd;

rcl_publisher_t dbg_spin_pub;     // /quin/debug/spin
rcl_publisher_t dbg_drill_pub;    // /quin/debug/drill
rcl_publisher_t dbg_grip_pub;     // /quin/debug/gripper
geometry_msgs__msg__Twist dbg_spin_msg, dbg_drill_msg, dbg_grip_msg;

// ----------------- Agent state -----------------
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

// ----------------- Servo -----------------
Servo gripper;

// ----------------- Spin (tray) ISR state -----------------
volatile long     spin_pos_steps = 0;      // accumulated position
volatile long     spin_pending   = 0;      // queue (signed)
volatile bool     spin_dir_high  = true;   // current DIR level
volatile bool     spin_high_phase= false;  // pulse phase
volatile uint32_t spin_interval_us = STEP_PERIOD_US;  // fixed period (simple; you can add ramp if needed)
hw_timer_t*       spinTimer = nullptr;

portMUX_TYPE spin_mux = portMUX_INITIALIZER_UNLOCKED;     // protect host-visible counters
portMUX_TYPE spin_isr_mux = portMUX_INITIALIZER_UNLOCKED; // protect ISR shared vars

// ----------------- Drill ISR state -----------------
volatile long     drill_pos_steps = 0;
volatile long     drill_pending   = 0;
volatile bool     drill_dir_high  = true;  // HIGH = DOWN, LOW = UP (convention)
volatile bool     drill_high_phase= false;
volatile uint32_t drill_interval_us = STEP_PERIOD_US;
hw_timer_t*       drillTimer = nullptr;

portMUX_TYPE drill_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE drill_isr_mux = portMUX_INITIALIZER_UNLOCKED;

// ----------------- Edge/Toggle helpers -----------------
int  spin_last_dir = 0;           // -1/0/1 for edge detection on angular.z
bool drill_prev_pressed = false;  // toggle on rising edge
int  drill_toggle_state = 0;      // 0 = next DOWN, 1 = next UP

// ----------------- Debug rate limit -----------------
static inline bool allow_debug_publish(uint32_t interval_ms = 100) {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= interval_ms) { last = now; return true; }
  return false;
}

// ----------------- helpers -----------------

#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static int64_t _t = -1; \
    if (_t == -1) _t = uxr_millis(); \
    if ((int32_t)(uxr_millis() - _t) > (MS)) { \
      X; \
      _t = uxr_millis(); \
    } \
  } while (0)

// Robust version: safe in if/else without braces
#define RCCHECK(fn) do {                              \
  rcl_ret_t _rc = (fn);                               \
  if (_rc != RCL_RET_OK) {                            \
    rclErrorLoop();                                   \
  }                                                   \
} while (0)

#define RCSOFTCHECK(fn) do {                          \
  rcl_ret_t _rc = (fn);                               \
  (void)_rc;                                          \
} while (0)


// ----------------- Prototypes -----------------
void rclErrorLoop();
bool createEntities();
bool destroyEntities();

void spinCallback(const void* msgin);
void drillCallback(const void* msgin);
void gripCallback(const void* msgin);

void controlCallback(rcl_timer_t *timer, int64_t last_call_time);

void queueSpinSteps(long steps, bool dirHigh);
void queueDrillSteps(long steps, bool dirHigh);

// ----------------- ISRs -----------------
void IRAM_ATTR onSpinTimer() {
  if (!spin_high_phase) {
    // Idle fast path
    if (spin_pending == 0) {
      gpio_set_level((gpio_num_t)SPIN_STEP_PIN_PLS, 0);
      timerAlarmWrite(spinTimer, 1000, true); // 1 ms when idle
      return;
    }
    // Set DIR while STEP is LOW
    gpio_set_level((gpio_num_t)SPIN_STEP_PIN_DIR, spin_dir_high ? 1 : 0);
    // Rising edge
    gpio_set_level((gpio_num_t)SPIN_STEP_PIN_PLS, 1);
    spin_high_phase = true;
    timerAlarmWrite(spinTimer, STEP_PULSE_HIGH_US, true);
  } else {
    // Falling edge: count one step
    gpio_set_level((gpio_num_t)SPIN_STEP_PIN_PLS, 0);
    spin_high_phase = false;

    portENTER_CRITICAL_ISR(&spin_mux);
    spin_pos_steps += spin_dir_high ? 1 : -1;
    spin_pending   += spin_dir_high ? -1 : +1;
    portEXIT_CRITICAL_ISR(&spin_mux);

    // LOW phase
    timerAlarmWrite(spinTimer, (spin_interval_us > STEP_PULSE_HIGH_US) ? (spin_interval_us - STEP_PULSE_HIGH_US) : 1, true);
  }
}

void IRAM_ATTR onDrillTimer() {
  if (!drill_high_phase) {
    if (drill_pending == 0) {
      gpio_set_level((gpio_num_t)DRILL_STEP_PIN_PLS, 0);
      timerAlarmWrite(drillTimer, 1000, true);
      return;
    }
    gpio_set_level((gpio_num_t)DRILL_STEP_PIN_DIR, drill_dir_high ? 1 : 0);
    gpio_set_level((gpio_num_t)DRILL_STEP_PIN_PLS, 1);
    drill_high_phase = true;
    timerAlarmWrite(drillTimer, STEP_PULSE_HIGH_US, true);
  } else {
    gpio_set_level((gpio_num_t)DRILL_STEP_PIN_PLS, 0);
    drill_high_phase = false;

    portENTER_CRITICAL_ISR(&drill_mux);
    drill_pos_steps += drill_dir_high ? 1 : -1;
    drill_pending   += drill_dir_high ? -1 : +1;
    portEXIT_CRITICAL_ISR(&drill_mux);

    timerAlarmWrite(drillTimer, (drill_interval_us > STEP_PULSE_HIGH_US) ? (drill_interval_us - STEP_PULSE_HIGH_US) : 1, true);
  }
}

// ----------------- Enqueue helpers -----------------
void queueSpinSteps(long steps, bool dirHigh) {
  if (steps <= 0) return;
  portENTER_CRITICAL(&spin_isr_mux);
  spin_dir_high = dirHigh;
  spin_pending += dirHigh ? steps : -steps; // signed queue: +CW, -CCW
  portEXIT_CRITICAL(&spin_isr_mux);
}

void queueDrillSteps(long steps, bool dirHigh) {
  if (steps <= 0) return;
  portENTER_CRITICAL(&drill_isr_mux);
  drill_dir_high = dirHigh;                 // HIGH = DOWN
  drill_pending += dirHigh ? steps : -steps;
  portEXIT_CRITICAL(&drill_isr_mux);
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // IO
  pinMode(SPIN_STEP_PIN_PLS, OUTPUT);
  pinMode(SPIN_STEP_PIN_DIR, OUTPUT);
  pinMode(DRILL_STEP_PIN_PLS, OUTPUT);
  pinMode(DRILL_STEP_PIN_DIR, OUTPUT);
  gpio_set_level((gpio_num_t)SPIN_STEP_PIN_PLS, 0);
  gpio_set_level((gpio_num_t)SPIN_STEP_PIN_DIR, 0);
  gpio_set_level((gpio_num_t)DRILL_STEP_PIN_PLS, 0);
  gpio_set_level((gpio_num_t)DRILL_STEP_PIN_DIR, 0);

  // Servo
  gripper.attach(SERVO_PIN, 500, 2500);
  gripper.write(SERVO_CLOSED);
  dbg_grip_msg.linear.x = 0.0;  // initial: closed

  // HW Timers @ 1 MHz (1us ticks)
  spinTimer  = timerBegin(0, 80, true);
  drillTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(spinTimer,  &onSpinTimer,  true);
  timerAttachInterrupt(drillTimer, &onDrillTimer, true);
  timerAlarmWrite(spinTimer,  spin_interval_us,  true);
  timerAlarmWrite(drillTimer, drill_interval_us, true);
  timerAlarmEnable(spinTimer);
  timerAlarmEnable(drillTimer);
}


// ----------------- Loop (agent state) -----------------
void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 2)) ? AGENT_AVAILABLE : WAITING_AGENT
      );
      break;
    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroyEntities();
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1000,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 2)) ? AGENT_CONNECTED : AGENT_DISCONNECTED
      );
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)); // short, frequent
      }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }
}

// ----------------- ROS wiring -----------------
void spinCallback(const void* msgin) {
  const auto* m = (const geometry_msgs__msg__Twist*) msgin;
  float z = m->angular.z;

  if (z > TWIST_THRESH && spin_last_dir != 1 && spin_pending == 0) {
    queueSpinSteps((long)STEPS_PER_36, /*CW*/true);
    spin_last_dir = 1;
  } else if (z < -TWIST_THRESH && spin_last_dir != -1 && spin_pending == 0) {
    queueSpinSteps((long)STEPS_PER_36, /*CCW*/false);
    spin_last_dir = -1;
  } else if (fabsf(z) < TWIST_DEADZONE) {
    spin_last_dir = 0;
  }

  dbg_spin_msg.angular.z = z;
}

void drillCallback(const void* msgin) {
  const auto* m = (const geometry_msgs__msg__Twist*) msgin;
  float z = m->linear.z;
  bool pressed = (fabsf(z) > JOY_PRESS_THRESH);
  bool rising  = pressed && !drill_prev_pressed;

  if (rising && drill_pending == 0) {
    long total = (long) llround(TRAVEL_CM * (double)STEPS_PER_CM);
    if (drill_toggle_state == 0) {
      // DOWN next
      queueDrillSteps(total, /*DOWN*/true);
      drill_toggle_state = 1;
    } else {
      // UP next
      queueDrillSteps(total, /*UP*/false);
      drill_toggle_state = 0;
    }
  }
  drill_prev_pressed = pressed;
  dbg_drill_msg.linear.z = z;
}

void gripCallback(const void* msgin) {
  const auto* m = (const geometry_msgs__msg__Twist*) msgin;

  if (m->linear.x == 0.0) {
    gripper.write(SERVO_CLOSED);   // closed
    dbg_grip_msg.linear.x = 0.0;   // echo state as 0
  } 
  else if (m->linear.x == 1.0) {
    gripper.write(SERVO_OPENED);   // opened
    dbg_grip_msg.linear.x = 1.0;   // echo state as 1
  }
  // else: ignore (keeps last state)
}

void controlCallback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
  // publish debug at 10 Hz
  if (allow_debug_publish(100)) {
    // Spin status
    long s_pos, s_pend;
    portENTER_CRITICAL(&spin_mux);
    s_pos  = spin_pos_steps;
    s_pend = spin_pending;
    portEXIT_CRITICAL(&spin_mux);
    dbg_spin_msg.linear.x  = (double)s_pos;
    dbg_spin_msg.linear.y  = (double)s_pend;
    dbg_spin_msg.angular.x = (double)spin_interval_us;
    rcl_publish(&dbg_spin_pub, &dbg_spin_msg, NULL);

    // Drill status
    long d_pos, d_pend;
    portENTER_CRITICAL(&drill_mux);
    d_pos  = drill_pos_steps;
    d_pend = drill_pending;
    portEXIT_CRITICAL(&drill_mux);
    dbg_drill_msg.linear.x  = (double)d_pos;
    dbg_drill_msg.linear.y  = (double)d_pend;
    dbg_drill_msg.angular.x = (double)drill_interval_us;
    rcl_publish(&dbg_drill_pub, &dbg_drill_msg, NULL);

    // Gripper echo
    rcl_publish(&dbg_grip_pub, &dbg_grip_msg, NULL);
  }
}

bool createEntities() {
  allocator = rcl_get_default_allocator();

  geometry_msgs__msg__Twist__init(&spin_cmd);
  geometry_msgs__msg__Twist__init(&drill_cmd);
  geometry_msgs__msg__Twist__init(&grip_cmd);
  geometry_msgs__msg__Twist__init(&dbg_spin_msg);
  geometry_msgs__msg__Twist__init(&dbg_drill_msg);
  geometry_msgs__msg__Twist__init(&dbg_grip_msg);

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rcl_init_options_set_domain_id(&init_options, 77);

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "quin_robot_isr_node", "", &support));

  // Subs
  RCCHECK(rclc_subscription_init_best_effort(
    &spin_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/quin/cmd_spin"));
  RCCHECK(rclc_subscription_init_best_effort(
    &drill_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/quin/cmd_drill"));
  RCCHECK(rclc_subscription_init_best_effort(
    &grip_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/quin/cmd_gripper"));

  // Pubs
  RCCHECK(rclc_publisher_init_best_effort(
    &dbg_spin_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/quin/debug/spin"));
  RCCHECK(rclc_publisher_init_best_effort(
    &dbg_drill_pub,&node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/quin/debug/drill"));
  RCCHECK(rclc_publisher_init_best_effort(
    &dbg_grip_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/quin/debug/gripper"));

  // Control timer (lightweight)
  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(20), controlCallback));

  // Executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &spin_sub,  &spin_cmd,  &spinCallback,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &drill_sub, &drill_cmd, &drillCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &grip_sub,  &grip_cmd,  &gripCallback,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  // Reset counters
  portENTER_CRITICAL(&spin_mux);  spin_pos_steps = 0; spin_pending = 0; portEXIT_CRITICAL(&spin_mux);
  portENTER_CRITICAL(&drill_mux); drill_pos_steps= 0; drill_pending= 0; portEXIT_CRITICAL(&drill_mux);

  return true;
}

bool destroyEntities() {
  rcl_timer_fini(&control_timer);
  rcl_subscription_fini(&spin_sub,  &node);
  rcl_subscription_fini(&drill_sub, &node);
  rcl_subscription_fini(&grip_sub,  &node);
  rcl_publisher_fini(&dbg_spin_pub, &node);
  rcl_publisher_fini(&dbg_drill_pub,&node);
  rcl_publisher_fini(&dbg_grip_pub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  return true;
}

void rclErrorLoop() {
  // Blink fast to indicate fatal ROS error
  pinMode(2, OUTPUT);
  while (1) { digitalWrite(2, HIGH); delay(100); digitalWrite(2, LOW); delay(100); }
}
