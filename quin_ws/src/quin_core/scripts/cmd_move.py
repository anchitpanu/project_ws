#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32MultiArray, Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy import qos
from quin_core.utilize import *
from quin_core.controller import *
import time 
import math

# CHANGED: add Float32 for debug encoder topic (plant count removed)
from std_msgs.msg import Float32   # for /quin/debug/encoder

def mm_to_cm(x_mm: float) -> float:
    return x_mm / 10.0

class Cmd_vel_to_motor_speed(Node):

    def __init__(self):
        super().__init__("Cmd_move")
        
        self.moveSpeed: float = 0.0
        self.turnSpeed: float = 0.0

        self.maxSpeed : float = 1023.0  # pwm
        self.maxRPM : int = 6000
        self.max_linear_speed = 3.0  # m/s max

        self.wheel_base = 0.375             # Distance between left-right wheels (m)
        self.wheel_radius = 0.085/2         # Wheel radius (m)

        self.motor1Speed : float = 0
        self.motor2Speed : float = 0
        self.motor3Speed : float = 0
        self.motor4Speed : float = 0

        self.yaw : float = 0
        self.yaw_setpoint = self.yaw

        self.motorspin1Speed: float = 0.0
        self.motordrillSpeed: float = 0.0

        # ------- encoder state (for local baseline/reset) -------
        self.encoder_mode = 1.0          # 1=Bit, 2=RPM (default)
        self.rack_distance = 0.0         # accumulated distance from local zero (cm)
        self._tick_zero = 0.0            # baseline ticks
        self._have_baseline = False      # first-sample guard

        self.servo_angle : float = 0.0

        self.BTN_CIRCLE = 1
        self.BTN_SQUARE = 3
        self.BTN_L1 = 4
        self.BTN_R1 = 5

        # ---------- Publishers (unchanged, except plant count removed) ----------
        self.send_robot_speed = self.create_publisher(
            Twist, "/quin/cmd_move/rpm", qos_profile=qos.qos_profile_system_default
        )
        self.send_spin_speed = self.create_publisher(
            Twist, "/quin/cmd_spin/rpm", qos_profile=qos.qos_profile_system_default
        )
        self.send_drill_speed = self.create_publisher(
            Twist, "/quin/cmd_drill/rpm", qos_profile=qos.qos_profile_system_default
        )
        self.send_gripper_angle = self.create_publisher(
            Twist, "/quin/cmd_gripper/angle", qos_profile=qos.qos_profile_system_default
        )

        # ADDED: single “check” topic for distance after reset
        self.pub_debug_encoder = self.create_publisher(
            Float32, "/quin/debug/encoder", qos_profile=qos.qos_profile_system_default
        )

        # ---------- Subscriptions ----------
        self.create_subscription(
            Twist, '/quin/cmd_move', self.cmd_move, qos_profile=qos.qos_profile_system_default
        )
        self.create_subscription(
            Joy, '/quin/joy/spin', self.cmd_spin, qos_profile=qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Joy, '/quin/joy/drill', self.cmd_drill, qos_profile=qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Twist, "/quin/drill/feedback", self.drill_feedback, qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Twist, '/quin/cmd_gripper', self.cmd_gripper, qos_profile=qos.qos_profile_system_default
        )
        # CHANGED: route encoder data to cmd_encoder (handles in-band reset + distance)
        self.create_subscription(
            Twist, "/quin/cmd_encoder", self.cmd_encoder, qos_profile=qos.qos_profile_sensor_data
        )

        self.sent_data_timer = self.create_timer(0.01, self.sendData)
        
        # keep drill press edge detector for pulse
        self._prev_drill_pressed = False

    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 

    # (Kept for compatibility; not used by subscription anymore)
    def encoder(self,msg):
        self.encoder_mode = msg.linear.x    # 1 Bit, 2 RPM

    def cmd_move(self, msg):
        self.moveSpeed = msg.linear.y               # Forward/Backward
        self.turnSpeed = msg.angular.z * (-5.0)     # scale + invert if needed

        # Compute left and right wheel speeds (in m/s)
        left_speed = self.moveSpeed - (self.turnSpeed * self.wheel_base / 2.0)
        right_speed = self.moveSpeed + (self.turnSpeed * self.wheel_base / 2.0)

        # Map to motor speeds in RPM (example; tune to your system)
        rpm_left = float(left_speed * self.maxRPM)
        rpm_right = float(right_speed * self.maxRPM)

        self.motor1Speed = rpm_left
        self.motor2Speed = rpm_right

    # CHANGED: unified encoder input + in-band reset
    def cmd_encoder(self, msg: Twist):
        """
        Use /quin/cmd_encoder (Twist) to carry both:
          - raw ticks in msg.linear.z (keeps running from motor node)
          - mode in msg.linear.x (1=Bit, 2=RPM) [optional]
          - one-shot reset flag in msg.angular.x (>=0.5 triggers reset)

        We keep a local baseline (self._tick_zero) and publish
        user-facing distance (cm) on /quin/debug/encoder.
        """
        # optional mode passthrough
        self.encoder_mode = msg.linear.x

        # one-shot reset flag (sent by joystick)
        reset_flag = float(msg.angular.x)

        # geometry for conversion
        tick_per_revolution = 60.0
        diameter_mm = 85.0
        circumference_mm = math.pi * diameter_mm

        # raw ticks from motor node (monotonic)
        current_ticks = float(msg.linear.z)

        # handle reset immediately: rebase baseline and publish 0.0
        if reset_flag >= 0.5:
            self._tick_zero = current_ticks
            self._have_baseline = True
            self.rack_distance = 0.0
            self.pub_debug_encoder.publish(Float32(data=0.0))
            return

        # establish baseline on first sample
        if not self._have_baseline:
            self._tick_zero = current_ticks
            self._have_baseline = True
            self.rack_distance = 0.0
            self.pub_debug_encoder.publish(Float32(data=0.0))
            return

        # compute distance from local zero
        delta_ticks = current_ticks - self._tick_zero
        mm = (delta_ticks / tick_per_revolution) * circumference_mm
        self.rack_distance = mm_to_cm(mm)

        # publish “check” value
        self.pub_debug_encoder.publish(Float32(data=float(self.rack_distance)))

    def cmd_encoder_reset(self, msg):
        # NOTE: no longer used (we removed the separate reset topic)
        self.rack_distance = 0.0
        self._have_baseline = False
        self.get_logger().info("Encoder/local odometry reset")

    def cmd_spin(self, msg: Joy):
        circle = (len(msg.buttons) > self.BTN_CIRCLE and msg.buttons[self.BTN_CIRCLE] == 1)
        square = (len(msg.buttons) > self.BTN_SQUARE and msg.buttons[self.BTN_SQUARE] == 1)

        if circle and not square :       # Circle
            self.motorspin1Speed = 1.0          # CW       
        elif square and not circle :     # Square
            self.motorspin1Speed = -1.0         # CCW
        else:
            self.motorspin1Speed = 0.0          # Stop

    def cmd_drill(self, msg: Joy):
        # Keep behavior: short pulse on L1 rising edge (~50 ms)
        l1 = (len(msg.buttons) > self.BTN_L1 and msg.buttons[self.BTN_L1] == 1)
        if l1 and not self._prev_drill_pressed:
            self.motordrillSpeed = 1.0
            threading.Timer(0.05, lambda: setattr(self, "motordrillSpeed", 0.0)).start()
        self._prev_drill_pressed = l1

    def cmd_gripper(self, msg):
        if msg.linear.x == 0:               # Closed Servo
            self.servo_angle = float(0.0)
        if msg.linear.x == 1:               # Opened Servo
            self.servo_angle = float(50.0)

    def drill_feedback(self, msg: Twist):
        # unchanged placeholder
        pass

    def sendData(self):
        motorspeed_msg = Twist()
        motorspeed_msg.linear.x = float(self.motor1Speed)
        motorspeed_msg.linear.y = float(self.motor2Speed)

        motorspin_msg = Twist()
        motorspin_msg.angular.z = float(self.motorspin1Speed)

        motordrill_msg = Twist()
        motordrill_msg.linear.z = float(self.motordrillSpeed)

        servo_msg = Twist()
        servo_msg.linear.x = float(self.servo_angle)
        
        self.send_robot_speed.publish(motorspeed_msg)
        self.send_spin_speed.publish(motorspin_msg)
        self.send_drill_speed.publish(motordrill_msg)
        self.send_gripper_angle.publish(servo_msg)


def main():
    rclpy.init()

    sub = Cmd_vel_to_motor_speed()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
