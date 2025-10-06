#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy import qos
from quin_core.utilize import *
from quin_core.controller import *
import time 

from std_msgs.msg import Int32   # plant counter publisher

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

        # encoder/range state you may want to zero on reset
        self.encoder_mode = 1.0          # 1=Bit, 2=RPM (default)
        self.rack_distance = 0.0         # example accumulated distance (cm)
        self.left_ticks_offset = 0       # if you track offsets, zero them here
        self.right_ticks_offset = 0

        self.servo_angle : float = 0.0

        self.BTN_CIRCLE = 1
        self.BTN_SQUARE = 3
        self.BTN_L1 = 4
        self.BTN_R1 = 5


        # -------- plant counting via press-toggle --------
        self.move_down_next = True       # first press is DOWN, second (UP) completes a plant
        # self._prev_drill_pressed = False
        # self._last_press_ts = 0.0
        # self._press_debounce_s = 0.25     # ignore repeat within 250 ms
        self.plant_count = 0


        # self.previous_manual_turn = time.time()

        # self.controller = Controller(kp = 1.0, ki = 0.05, kd = 0.001, baseSpeed = 0.3  ,errorTolerance = To_Radians(0.5), i_min= -1, i_max= 1)
            
        # self.macro_active = False

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

        self.send_plant_count = self.create_publisher(
            Twist, "/quin/plant_count", qos_profile=qos.qos_profile_system_default
        )



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

        self.create_subscription(
            Twist, "/quin/cmd_encoder", self.encoder, qos_profile=qos.qos_profile_sensor_data
        )

        self.create_subscription(
            Twist, "/quin/cmd_encoder_reset", self.encoder_reset, qos_profile=qos.qos_profile_system_default
        )


        self.sent_data_timer = self.create_timer(0.01, self.sendData)
        

    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 

    def encoder(self,msg):
        self.encoder_mode = msg.linear.x    # 1 Bit, 2 RPM


    def cmd_move(self, msg):

        self.moveSpeed = msg.linear.y               # Forward/Backward
        self.turnSpeed = msg.angular.z              # Rotation
        self.turnSpeed = self.turnSpeed * 5 * (-1.0)   # Scale factor for angular velocity

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


    def cmd_encoder(self, msg):

        tick_per_revolution = 60
        diameter = 85  # mm
        circumference = math.pi * diameter  # mm

        self.rack_distance += mm_to_cm((msg.linear.z / tick_per_revolution) * circumference)


    def cmd_encoder_reset(self, msg):

        # Zero any locally accumulated values/offsets
        self.rack_distance = 0.0
        self.left_ticks_offset = 0
        self.right_ticks_offset = 0

        # If you keep any PID integrators or filters, clear them here too
        # e.g., self.controller.reset()  (if you use one)

        self.get_logger().info("Encoder/local odometry reset")
    

    def cmd_spin(self, msg):
        
        circle = (len(msg.buttons) > self.BTN_CIRCLE and msg.buttons[self.BTN_CIRCLE] == 1)
        square = (len(msg.buttons) > self.BTN_SQUARE and msg.buttons[self.BTN_SQUARE] == 1)

        if circle and not square :       # Circle
            self.motorspin1Speed = 1.0          # Clockwise       
        
        elif square and not circle :     # Square
            self.motorspin1Speed = -1.0         # Counter-Clockwise

        else:
            self.motorspin1Speed = 0.0          # Stop


    def cmd_drill(self, msg):

        l1 = (len(msg.buttons) > self.BTN_L1 and msg.buttons[self.BTN_L1] == 1)

        # Edge-detect without editing __init__
        if not hasattr(self, "_prev_drill_pressed"):
            self._prev_drill_pressed = False

        if l1 and not self._prev_drill_pressed:
            # Emit a short pulse: 1.0 then auto-reset to 0.0
            self.motordrillSpeed = 1.0
            threading.Timer(0.05, lambda: setattr(self, "motordrillSpeed", 0.0)).start()

            # --- ADDED: count 1 plant on the 'UP' press (every second press) ---
            # move_down_next True  => this press commands DOWN (start of cycle)
            # move_down_next False => this press commands UP   (end of cycle) -> count +1
            if not self.move_down_next:
                self.plant_count += 1
                self.send_plant_count.publish(Int32(data=self.plant_count))

            # flip expected direction for next press
            self.move_down_next = not self.move_down_next

        self._prev_drill_pressed = l1
        

    def cmd_gripper(self, msg):
        if msg.linear.x == 1:               # Closed Servo
            self.servo_angle = float(0.0)

        if msg.linear.x == 2:               # Opened Servo
            self.servo_angle = float(60.0)


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
