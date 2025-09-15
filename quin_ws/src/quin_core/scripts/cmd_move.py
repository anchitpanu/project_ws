#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy import qos
from quin_core.utilize import *
from quin_core.controller import *
import time 

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

        # self.previous_manual_turn = time.time()

        # self.controller = Controller(kp = 1.0, ki = 0.05, kd = 0.001, baseSpeed = 0.3  ,errorTolerance = To_Radians(0.5), i_min= -1, i_max= 1)
            
        # self.macro_active = False

        self.send_robot_speed = self.create_publisher(
            Twist, "/quin/cmd_move/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.send_spin_speed = self.create_publisher(
            Twist, "/quin/cmd_spin/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/quin/cmd_move', self.cmd_move, qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/quin/cmd_spin', self.cmd_spin, qos_profile=qos.qos_profile_system_default
        )
        
        self.create_subscription(
            Twist, '/quin/cmd_macro', self.cmd_macro, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.create_subscription(
            Twist, "/quin/cmd_encoder", self.encoder, qos_profile=qos.qos_profile_sensor_data
        )


        self.sent_data_timer = self.create_timer(0.01, self.sendData)
        

    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 

    def encoder(self,msg):
        self.encoder_mode = msg.linear.x    # 1 Bit, 2 RPM

    def cmd_move(self, msg):

        self.moveSpeed = msg.linear.y           # Forward/Backward
        self.turnSpeed = msg.angular.z          # Rotation
        self.turnSpeed = self.turnSpeed * 5     # Scale factor for angular velocity

        # Compute left and right wheel speeds (in m/s)
        left_speed = self.moveSpeed - (self.turnSpeed * self.wheel_base / 2.0)
        right_speed = self.moveSpeed + (self.turnSpeed * self.wheel_base / 2.0)

        # Convert to motor speeds in RPM (optional)
        rpm_left = float(left_speed * self.maxRPM)
        rpm_right = float(right_speed * self.maxRPM)

        # Store or send these speeds to motor controller
        self.motor1Speed = rpm_left
        self.motor2Speed = rpm_right

        print(f"Left Motor: {self.motor1Speed:.2f} RPM, Right Motor: {self.motor2Speed:.2f} RPM")


    def cmd_spin(self, msg):

        if msg.angular.z == 1:
            self.remaining_steps += 205         # 1 rotation = 205 steps

        if msg.angular.z == 2:
            self.remaining_steps -= 205         # 1 rotation = 205 steps


    def sendData(self):
        motorspeed_msg = Twist()
       
        motorspeed_msg.linear.x = float(self.motor1Speed)
        motorspeed_msg.linear.y = float(self.motor2Speed)

        motorspin_msg = Twist()
        motorspin_msg.angular.z = float(self.motorspin1Speed)
        
        self.send_robot_speed.publish(motorspeed_msg)
        self.send_spin_speed.publish(motorspin_msg)


def main():
    rclpy.init()

    sub = Cmd_vel_to_motor_speed()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
