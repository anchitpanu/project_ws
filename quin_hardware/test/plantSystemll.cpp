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
#include <Stepper.h> 

#include "../config/spin.h" 
#include "../config/digging.h" 

// -------- Helpers --------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)

// -------- ROS entities --------
rcl_publisher_t debug_spin_publisher;
geometry_msgs__msg__Twist debug_spin_msg;

rcl_publisher_t debug_drill_publisher;
geometry_msgs__msg__Twist debug_drill_msg;

rcl_publisher_t debug_gripper_publisher;
geometry_msgs__msg__Twist debug_gripper_msg;

rcl_subscription_t spin_subscriber;        // subscribe /cmd_spin
geometry_msgs__msg__Twist spin_msg;

rcl_subscription_t drill_subscriber;       // subscribe /cmd_drill
geometry_msgs__msg__Twist drill_msg;

rcl_subscription_t gripper_subscriber;     // subscribe /cmd_gripper
geometry_msgs__msg__Twist gripper_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long current_time = 0;

enum states
{
    WAITING_AGENT,       // 0
    AGENT_AVAILABLE,     // 1
    AGENT_CONNECTED,     // 2
    AGENT_DISCONNECTED   // 3
} state;

// Movement queue: remaining steps to execute (positive or negative)
volatile long remaining_steps = 0;

// Edge-detection state for joystick direction
// -1 = last was LEFT, 0 = neutral, 1 = RIGHT
int last_dir = 0;

// Keep original Stepper objects (not used for pulse timing now, but preserved to avoid structural changes)
Stepper spinStepper(STEPS_PER_REV, SPIN_STEP_PIN_PLS, SPIN_STEP_PIN_DIR);
Stepper drillStepper(STEPS_PER_CM, DRILL_STEP_PIN_PLS, DRILL_STEP_PIN_DIR);

Servo myServo;

// ------ Non-blocking stepper helpers (ADDED) ------
// These helpers implement a non-blocking STEP/DIR pulse generator.
// They are serviced at high frequency from control timer without blocking ROS executor.
struct StepperCtl {
  uint8_t pin_step;
  uint8_t pin_dir;
  volatile long steps_remaining;   // remaining steps to emit (counts down to 0)
  unsigned long next_us;           // timestamp for the next edge (micros)
  bool pulse_high;                 // current edge state (HIGH interval in progress)
};

StepperCtl spinCtl  = {SPIN_STEP_PIN_PLS,  SPIN_STEP_PIN_DIR, 0, 0, false};
StepperCtl drillCtl = {DRILL_STEP_PIN_PLS, DRILL_STEP_PIN_DIR, 0, 0, false};

// Emit one edge if it's time; call this from control timer frequently.
inline void service_stepper(StepperCtl &ctl) {
  if (ctl.steps_remaining == 0) return;
  unsigned long now = micros();
  if (now < ctl.next_us) return;

  if (!ctl.pulse_high) {
    // Start HIGH interval of the step pulse
    digitalWrite(ctl.pin_step, HIGH);
    ctl.pulse_high = true;
    ctl.next_us = now + STEP_PULSE_HIGH_US;                 // HIGH width
  } else {
    // End HIGH interval -> go LOW and count one step
    digitalWrite(ctl.pin_step, LOW);
    ctl.pulse_high = false;
    ctl.next_us = now + (STEP_PERIOD_US - STEP_PULSE_HIGH_US);  // remaining LOW time
    ctl.steps_remaining--;                                     // one full step completed
  }
}

// Queue steps (non-blocking). Direction is set immediately, pulse starts ASAP.
inline void queue_steps(StepperCtl &ctl, bool dir_high, long steps) {
  digitalWrite(ctl.pin_dir, dir_high ? HIGH : LOW);
  ctl.pulse_high = false;           // ensure we begin from a LOW edge
  ctl.steps_remaining += steps;     // allow stacking multiple commands
  ctl.next_us = micros();           // ready to emit immediately
}

// ------ function list ------
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
struct timespec getTime();
void Spin();
void Drill();
void Gripper();

// ------ main ------
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(SPIN_STEP_PIN_PLS, OUTPUT);
  pinMode(SPIN_STEP_PIN_DIR, OUTPUT);
  pinMode(DRILL_STEP_PIN_PLS, OUTPUT);
  pinMode(DRILL_STEP_PIN_DIR, OUTPUT);

  // Keep original API calls to avoid structural changes
  spinStepper.setSpeed(STEPPER_RPM);
  drillStepper.setSpeed(STEPPER_RPM);

  myServo.attach(19);
  myServo.write(SERVO_CLOSED);
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 10)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) { destroyEntities(); }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 10)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}

// ------ functions ------
void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        // Non-blocking pulse service for both steppers (IMPORTANT: this must be called very frequently)
        service_stepper(spinCtl);
        service_stepper(drillCtl);

        // High-level logic (enqueues steps only, no busy-wait loops)
        Spin();
        Drill();
        Gripper();

        publishData();
    }
}

void twistCallback(const void *msgin)
{   
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    prev_cmd_time = millis();
    spin_msg.angular.z = msg->angular.z; // rotate
}

void twist2Callback(const void *msgin)
{   
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    prev_cmd_time = millis();
    drill_msg.linear.z = msg->linear.z; // up - down
}

void twist3Callback(const void *msgin)
{   
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    prev_cmd_time = millis();
    gripper_msg.linear.x = msg->linear.x; // rotate
}

bool createEntities()       // create ROS entities 
{
    allocator = rcl_get_default_allocator();    // manage memory of micro-ROS
    geometry_msgs__msg__Twist__init(&debug_spin_msg);
    geometry_msgs__msg__Twist__init(&debug_drill_msg);
    geometry_msgs__msg__Twist__init(&debug_gripper_msg);

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 77);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    RCCHECK(rclc_node_init_default(&node, "quin_robot_node", "", &support));

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

    // Shorter control period so the pulse service runs at ~1kHz (non-blocking stepping)
    const unsigned int control_timeout = 1;   // ms
    RCCHECK(rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(control_timeout), controlCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));    // max handles = 5

    RCCHECK(rclc_executor_add_subscription(
        &executor, &spin_subscriber, &spin_msg, &twistCallback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor, &drill_subscriber, &drill_msg, &twist2Callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor, &gripper_subscriber, &gripper_msg, &twist3Callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    syncTime();

    return true;
}

bool destroyEntities()      // destroy ROS entities
{
    // Keep original destruction order; ensures clean teardown on disconnect
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

void Spin()
{
    // Edge-based triggering: enqueue exactly 36-degree worth of steps per direction change press
    const float z = spin_msg.angular.z;

    if (z > TWIST_THRESH && last_dir != 1) {
      // Clockwise (adjust DIR polarity to match your driver if needed)
      queue_steps(spinCtl, /*dir_high=*/true, STEPS_PER_36);
      last_dir = 1;  // block re-trigger until direction changes or neutral
    } 
    else if (z < -TWIST_THRESH && last_dir != -1) {
      // Counter-clockwise
      queue_steps(spinCtl, /*dir_high=*/false, STEPS_PER_36);
      last_dir = -1;
    } 
    else if (fabs(z) < TWIST_DEADZONE) {
      // Neutral: allow next edge
      last_dir = 0;
    }

    debug_spin_msg.angular.z = spin_msg.angular.z;
}

void Drill()
{
    // Toggle down/up per press; movement distance is TRAVEL_CM each time.
    static int  move_state   = 0;    // 0 = idle/ready, 1 = next is down, -1 = next is up
    static bool prev_pressed = false;

    const float z = drill_msg.linear.z;
    bool pressed_now  = (fabs(z) > JOY_PRESS_THRESH);
    bool rising_edge  = pressed_now && !prev_pressed;

    if (rising_edge) {
        long total_steps = (long)(TRAVEL_CM * STEPS_PER_CM);

        if (move_state != 1) {
            // Queue "down" travel
            queue_steps(drillCtl, /*dir_high=*/true, total_steps);
            move_state = 1;
        } else {
            // Queue "up" travel
            queue_steps(drillCtl, /*dir_high=*/false, total_steps);
            move_state = -1;
        }
    }

    prev_pressed = pressed_now;
    debug_drill_msg.linear.z = drill_msg.linear.z;
}

void Gripper()
{
    // Simple immediate servo actuation (already non-blocking)
    if (gripper_msg.linear.x == 1) {
        myServo.write(SERVO_OPENED);
        gripper_msg.linear.x = 1.0;  // indicate opened
    } else if (gripper_msg.linear.x == 0) {
        myServo.write(SERVO_CLOSED);
        gripper_msg.linear.x = 0.0;  // indicate closed
    }

    debug_gripper_msg.linear.x = gripper_msg.linear.x;
}

void publishData()
{
    // Best-effort debug publishers; keep as-is
    debug_spin_msg.angular.z = spin_msg.angular.z;
    rcl_publish(&debug_spin_publisher, &debug_spin_msg, NULL);

    debug_drill_msg.linear.z = drill_msg.linear.z;
    rcl_publish(&debug_drill_publisher, &debug_drill_msg, NULL);

    debug_gripper_msg.linear.x = gripper_msg.linear.x;
    rcl_publish(&debug_gripper_publisher, &debug_gripper_msg, NULL);
}

void syncTime()
{
    // Wait until time is non-zero, then compute offset to Arduino millis()
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 0;
    do
    {
        ts = getTime();
        delay(10);
    } while (ts.tv_sec == 0 && ts.tv_nsec == 0);

    unsigned long long now_millis = (unsigned long long)ts.tv_sec * 1000ULL + (unsigned long long)(ts.tv_nsec / 1000000ULL);
    time_offset = now_millis - millis();

    Serial.print("Synchronized time: ");
    Serial.print(ts.tv_sec);
    Serial.print(" sec, ");
    Serial.print(ts.tv_nsec);
    Serial.println(" nsec");
}

struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000ULL;
    tp.tv_nsec = (now % 1000ULL) * 1000000UL;
    return tp;
}

void rclErrorLoop() {
    // Keep the original behavior on fatal RCL errors
    ESP.restart();
}
