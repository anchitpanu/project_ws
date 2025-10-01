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
#include <Stepper.h>

#include "../config/digging.h" 


// -------- Helpers --------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)


// -------- ROS entities --------

rcl_publisher_t debug_drill_publisher;
geometry_msgs__msg__Twist debug_drill_msg;

rcl_subscription_t drill_subscriber;        // subscribe /cmd_drill
geometry_msgs__msg__Twist drill_msg;

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

Stepper myStepper(STEPS_PER_MM, DRILL_STEP_PIN_PLS, DRILL_STEP_PIN_DIR);
volatile long remaining_steps = 0;
int last_dir = 0;

// ---------------- Function prototypes ----------------
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
struct timespec getTime();
void Drill();


void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    set_microros_serial_transports(Serial);     // connect between esp32 and micro-ros agent

    myStepper.setSpeed(STEPPER_RPM); 
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

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        Drill();
        publishData();
    }
}

void twistCallback(const void *msgin)
{   
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    prev_cmd_time = millis();
    drill_msg.linear.z = msg->linear.z;
}

bool createEntities()       // create ROS entities 
{
    allocator = rcl_get_default_allocator();    // manage memory of micro-Ros
    geometry_msgs__msg__Twist__init(&debug_drill_msg);

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 77);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    RCCHECK(rclc_node_init_default(&node, "quin_robot_node", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_drill_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/debug/drill"));

    RCCHECK(rclc_subscription_init_best_effort(
        &drill_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/cmd_drill"));

    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(control_timeout), controlCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));    // max handles = 3

    RCCHECK(rclc_executor_add_subscription(
        &executor, &drill_subscriber, &drill_msg, &twistCallback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    syncTime();

    return true;
}

bool destroyEntities()      // destroy ROS entities
{
    // rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    // (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_drill_publisher, &node);
    rcl_subscription_fini(&drill_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void Drill()
{
  // keep state inside the function to avoid editing globals
  static bool prev_pressed   = false;
  static bool move_down_next = true;   // toggle each press
  static long current_steps  = 0;      // 0 = top; + = down

  const float z = drill_msg.linear.z;
  const bool pressed_now = (fabsf(z) >= PRESS_THRESH);
  const bool rising_edge = pressed_now && !prev_pressed;

  if (rising_edge)
  {
    const long steps_per_press = lroundf(PRESS_TRAVEL_MM * STEPS_PER_MM);
    const long max_steps       = (MAX_TRAVEL_MM > 0) ? lroundf(MAX_TRAVEL_MM * STEPS_PER_MM) : LONG_MAX;

    long cur = current_steps;

    if (move_down_next) {
      // DOWN movement (+ direction)
      long down_room    = max_steps - cur;                          // how many steps left to bottom
      long steps_to_do  = (down_room < steps_per_press) ? down_room : steps_per_press;

      if (steps_to_do > 0) {
        // Stepper.step() blocks until done; sign = direction
        myStepper.step( steps_to_do );                               // positive = one direction
        current_steps += steps_to_do;
      }
      move_down_next = false;
    } else {
      // UP movement (− direction)
      long up_room     = cur;                                        // distance to top (0)
      long steps_to_do = (up_room < steps_per_press) ? up_room : steps_per_press;

      if (steps_to_do > 0) {
        myStepper.step( -steps_to_do );                              // negative = opposite direction
        current_steps -= steps_to_do;
      }
      move_down_next = true;
    }
  }

  prev_pressed = pressed_now;

  // debug out
  debug_drill_msg.linear.z = drill_msg.linear.z;
}




void publishData()
{
    debug_drill_msg.linear.z = drill_msg.linear.z;

    rcl_publish(&debug_drill_publisher, &debug_drill_msg, NULL);
}

void syncTime()
{
    // wait for time to be non-zero
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 0;
    do
    {
        ts = getTime();
        delay(10);
    } while (ts.tv_sec == 0 && ts.tv_nsec == 0);

    unsigned long long now_millis = (unsigned long long)ts.tv_sec * 1000 + (unsigned long long)(ts.tv_nsec / 1000000);
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
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}
void rclErrorLoop() {

    ESP.restart();
}