#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy import qos
from kobe_core.utilize import *
from kobe_core.controller import *
import time 

class Cmd_vel_to_motor_speed(Node):

    def __init__(self):
        super().__init__("Cmd_vel_to_motor_speed")
        
        self.moveSpeed: float = 0.0
        self.turnSpeed: float = 0.0

        self.maxSpeed : float = 1023.0
        self.maxRPM : int = 6000
        self.max_linear_speed = 3.0  # m/s max

        self.wheel_base = 0.4         # Distance between left-right wheels (m)
        self.wheel_radius = 0.05      # Wheel radius (m)

        self.motor1Speed : float = 0
        self.motor2Speed : float = 0
        self.motor3Speed : float = 0
        self.motor4Speed : float = 0

        # self.yaw : float = 0
        # self.yaw_setpoint = self.yaw

        self.previous_manual_turn = time.time()

        # self.controller = Controller(kp = 1.0, ki = 0.05, kd = 0.001, baseSpeed = 0.3  ,errorTolerance = To_Radians(0.5), i_min= -1, i_max= 1)
            
        self.macro_active = False

        self.send_robot_speed = self.create_publisher(
            Twist, "/quin/cmd_move/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/quin/cmd_move', self.cmd_move, qos_profile=qos.qos_profile_system_default
        )
        
        self.create_subscription(
            Twist, '/quin/cmd_macro', self.cmd_macro, qos_profile=qos.qos_profile_sensor_data # 10
        )


        self.sent_data_timer = self.create_timer(0.01, self.sendData)
        

    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 

    def cmd_move(self, msg):

        CurrentTime = time.time()
        self.moveSpeed = msg.linear.y       # Forward/Backward
        self.turnSpeed = msg.angular.z      # Rotation
        
        rotation = self.controller.Calculate(WrapRads(self.yaw_setpoint - self.yaw))
        rotation = clip(rotation, -1.0, 1.0)
        if self.moveSpeed  == 0 and self.turnSpeed == 0 and abs(rotation) < 0.2:
            rotation = 0


        self.previous_manual_turn = CurrentTime if self.turnSpeed != 0 else self.previous_manual_turn


        """
        
        (Motor2)//-------------\\(Motor1)
                |               |
                |               |
                |               |
                |               |               
        (Motor3)\\-------------//(Motor4)


        """

        # Differential drive calculation
        left_speed  = (self.moveSpeed - rotation) * self.maxSpeed
        right_speed = (self.moveSpeed + rotation) * self.maxSpeed

        # Assign to motors
        self.motor1Speed = float("{:.1f}".format(left_speed))   # Front Left
        self.motor3Speed = float("{:.1f}".format(left_speed))   # Back Left
        self.motor2Speed = float("{:.1f}".format(right_speed))  # Front Right
        self.motor4Speed = float("{:.1f}".format(right_speed))  # Back Right

            
    def sendData(self):
        motorspeed_msg = Twist()


        """
        
        (Motor2)//-------------\\(Motor1)
                |               |
                |               |
                |               |
                |               |               
        (Motor3)\\-------------//(Motor4)
  
        
        """

       
        motorspeed_msg.linear.x = float(self.motor1Speed) 
        motorspeed_msg.linear.y = float(self.motor2Speed)
        motorspeed_msg.linear.z = float(self.motor3Speed) 
        motorspeed_msg.angular.x = float(self.motor4Speed) 

        self.send_robot_speed.publish(motorspeed_msg)


def main():
    rclpy.init()

    sub = Cmd_vel_to_motor_speed()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
