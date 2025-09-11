#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>   // ping/sync agent

#include "../config/config.h"            // ใช้พิน/พารามิเตอร์เดิมของคุณ

// ====================== micro-ROS objects ======================
rcl_subscription_t sub_twist;
geometry_msgs__msg__Twist msg_twist;

rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;

// ====================== Agent state machine ====================
enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
static AgentState state = WAITING_AGENT;

// helper macro เหมือนโค้ดที่รันได้
#define EXECUTE_EVERY_N_MS(MS, X) do {                        \
  static volatile int64_t _t = -1;                            \
  if (_t == -1) { _t = (int64_t)millis(); }                   \
  if (((int64_t)millis()) - _t > (MS)) { {X;} _t = millis(); }\
} while (0)

// ====================== Utilities ==============================
static void stop_all_motors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

static void die_here(const char* what, rcl_ret_t rc) {
  Serial.print("[micro-ROS] "); Serial.print(what);
  Serial.print(" failed rc="); Serial.println((int)rc);
  // ปลอดภัยไว้ก่อน: หยุดมอเตอร์และค้าง
  stop_all_motors();
  while (1) { delay(200); }
}

// ====================== Motor Control ==========================
// โหมดนี้: IN_A = PWM (ledcWrite), IN_B = ทิศทาง (digital)
static void setMotor(int pwm, int IN_A, int IN_B, bool inv, int channel) {
  if (inv) pwm = -pwm;
  int duty = constrain(abs(pwm), 0, PWM_Max);
  bool forward = (pwm >= 0);

  // ตั้งทิศทางบน IN_B (digital) เท่านั้น
  digitalWrite(IN_B, forward ? HIGH : LOW);
  // เขียน PWM เฉพาะ IN_A
  ledcWrite(channel, duty);
}

static void driveVW(float v, float w) {
  // คำนวณความเร็วล้อซ้าย/ขวา (m/s)
  float vL = v - (w * LR_WHEELS_DISTANCE / 2.0f);
  float vR = v + (w * LR_WHEELS_DISTANCE / 2.0f);

  // m/s → RPM
  float rpmL = (vL / (PI * WHEEL_DIAMETER)) * 60.0f;
  float rpmR = (vR / (PI * WHEEL_DIAMETER)) * 60.0f;

  float max_rpm = MOTOR_MAX_RPM * MAX_RPM_RATIO;
  rpmL = constrain(rpmL, -max_rpm, max_rpm);
  rpmR = constrain(rpmR, -max_rpm, max_rpm);

  int pwmL = (int)(rpmL / max_rpm * PWM_Max);
  int pwmR = (int)(rpmR / max_rpm * PWM_Max);

  // ส่งสัญญาณไป 4 ล้อ (2 ล้อซ้ายใช้ค่าเดียวกัน, 2 ล้อขวาใช้ค่าเดียวกัน)
  setMotor(pwmL, MOTOR1_IN_A, MOTOR1_IN_B, MOTOR1_INV, 0);
  setMotor(pwmL, MOTOR2_IN_A, MOTOR2_IN_B, MOTOR2_INV, 1);
  setMotor(pwmR, MOTOR3_IN_A, MOTOR3_IN_B, MOTOR3_INV, 2);
  setMotor(pwmR, MOTOR4_IN_A, MOTOR4_IN_B, MOTOR4_INV, 3);
}

// ====================== ROS Callbacks ==========================
static void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;
  driveVW(twist->linear.x, twist->angular.z);
}

// ====================== Entities lifecycle =====================
static bool createEntities() {
  allocator = rcl_get_default_allocator();
  geometry_msgs__msg__Twist__init(&msg_twist);

  // init options + (ถ้าต้อง) Domain ID ให้ตรงฝั่ง PC
  init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t rc = rcl_init_options_init(&init_options, allocator);
  if (rc != RCL_RET_OK) { die_here("rcl_init_options_init", rc); return false; }

  // ปลดคอมเมนต์ถ้าฝั่ง PC ใช้ Domain อื่น (เช่น 96)
  // rc = rcl_init_options_set_domain_id(&init_options, 96);
  // if (rc != RCL_RET_OK) { die_here("rcl_init_options_set_domain_id", rc); return false; }

  rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  if (rc != RCL_RET_OK) { die_here("rclc_support_init_with_options", rc); return false; }

  rc = rclc_node_init_default(&node, "diff_drive_4wd", "", &support);
  if (rc != RCL_RET_OK) { die_here("rclc_node_init_default", rc); return false; }

  rc = rclc_subscription_init_default(
        &sub_twist, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
  if (rc != RCL_RET_OK) { die_here("rclc_subscription_init_default(cmd_vel)", rc); return false; }

  executor = rclc_executor_get_zero_initialized_executor();
  rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (rc != RCL_RET_OK) { die_here("rclc_executor_init", rc); return false; }

  rc = rclc_executor_add_subscription(&executor, &sub_twist, &msg_twist, &twist_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK) { die_here("rclc_executor_add_subscription", rc); return false; }

  return true;
}

static bool destroyEntities() {
  // หยุดมอเตอร์ก่อน
  stop_all_motors();

  rcl_ret_t rc;
  rc = rclc_executor_fini(&executor);
  (void)rc;
  rc = rcl_subscription_fini(&sub_twist, &node);
  (void)rc;
  rc = rcl_node_fini(&node);
  (void)rc;
  rc = rclc_support_fini(&support);
  (void)rc;
  return true;
}

// ====================== Setup / Loop ===========================
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // PWM channels (0..3) สำหรับ 4 มอเตอร์ ตามโค้ดแรก
  ledcSetup(0, PWM_FREQUENCY, PWM_BITS);
  ledcSetup(1, PWM_FREQUENCY, PWM_BITS);
  ledcSetup(2, PWM_FREQUENCY, PWM_BITS);
  ledcSetup(3, PWM_FREQUENCY, PWM_BITS);

  // แนบ PWM เฉพาะขา IN_A ของแต่ละมอเตอร์
  ledcAttachPin(MOTOR1_IN_A, 0);
  ledcAttachPin(MOTOR2_IN_A, 1);
  ledcAttachPin(MOTOR3_IN_A, 2);
  ledcAttachPin(MOTOR4_IN_A, 3);

  // ขา IN_B เป็นทิศทาง (digital)
  pinMode(MOTOR1_IN_B, OUTPUT);
  pinMode(MOTOR2_IN_B, OUTPUT);
  pinMode(MOTOR3_IN_B, OUTPUT);
  pinMode(MOTOR4_IN_B, OUTPUT);

  // ตั้งทิศเริ่มต้นเป็นหยุด (ลดกระชาก)
  digitalWrite(MOTOR1_IN_B, LOW);
  digitalWrite(MOTOR2_IN_B, LOW);
  digitalWrite(MOTOR3_IN_B, LOW);
  digitalWrite(MOTOR4_IN_B, LOW);
  stop_all_motors();

  // เริ่มที่ WAITING_AGENT
  state = WAITING_AGENT;
  Serial.println("[micro-ROS] booted. Waiting agent...");
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      // ping ทุก ๆ 1s จนกว่าจะมี agent
      EXECUTE_EVERY_N_MS(1000, {
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
          state = AGENT_AVAILABLE;
          Serial.println("[micro-ROS] agent available.");
        } else {
          Serial.println("[micro-ROS] agent not available, retry...");
        }
      });
      break;

    case AGENT_AVAILABLE: {
      // สร้าง entity ถ้าสำเร็จ → CONNECTED ไม่งั้นวนรอใหม่
      bool ok = createEntities();
      if (ok) {
        state = AGENT_CONNECTED;
        Serial.println("[micro-ROS] entities created. Connected.");
      } else {
        destroyEntities();
        state = WAITING_AGENT;
      }
      break;
    }

    case AGENT_CONNECTED:
      // คอยเช็ค agent หลุดไหม ทุก ๆ 200ms
      EXECUTE_EVERY_N_MS(200, {
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
          state = AGENT_DISCONNECTED;
          Serial.println("[micro-ROS] agent disconnected.");
        }
      });
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
      }
      break;

    case AGENT_DISCONNECTED:
      // หยุดมอเตอร์, เก็บของ แล้ววนไป WAITING
      stop_all_motors();
      destroyEntities();
      state = WAITING_AGENT;
      Serial.println("[micro-ROS] cleaned up. Back to WAITING_AGENT.");
      break;

    default:
      state = WAITING_AGENT;
      break;
  }
}
