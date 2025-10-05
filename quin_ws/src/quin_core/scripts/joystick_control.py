#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray , Float32MultiArray
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy
from rclpy import qos


class Gamepad:
    def __init__(self):
        #Axes:--------------------------------------------------------
        
        self.lx : float = 0.0                   # 0: Left X-Axis
        self.ly : float = 0.0                   # 1: Left Y-Axis
        self.l2 : float = 0.0                   # 2: L2
        self.rx : float = 0.0                   # 3: Right X-Axis
        self.ry : float = 0.0                   # 4: Right Y-Axis
        self.r2 : float = 0.0                   # 5: R2
        self.dpadLeftRight : float = 0.0        # 6: Dpad Left and Right
        self.dpadUpDown : float = 0.0           # 7: Dpad Up and Down
        
        #Buttons:-------------------------------------------------------
        
        self.button_cross : float = 0.0         # 0: 
        self.button_circle : float = 0.0        # 1:
        self.button_triangle : float = 0.0      # 2:
        self.button_square : float = 0.0        # 3:
        self.l1 : float = 0.0                   # 4:
        self.r1 : float = 0.0                   # 5:
        #self.l2_logic : float = 0.0                   # 6:
        #self.r2_logic : float = 0.0                   # 7:
        self.button_share : float = 0.0         # 8:
        self.button_option : float = 0.0        # 9:
        self.button_logo : float = 0.0          # 10:
        self.PressedLeftAnalog : float = 0.0    # 11:
        self.PressedRightAnalog : float = 0.0   # 12:

        #----------------------------------------------------------------
        
        self.drill: bool = False
        self.spin: bool = False
        self.spinback: bool = False
        self.gripper: bool = False
        self.encoder_reset: bool = False
        self.toggle_encoder_bool : bool = False
    

        self.previous_triangle_state = False
        self.previous_circle_state = False
        self.previous_square_state = False
        self.previous_l1_state = False
        self.previous_r1_state = False
        self.previous_PressedRightAnalog_state = False

        self.last_macro_button = None  # Stores 'drill' 'spin'


    def update_drill(self):
        if self.l1 and not self.previous_l1_state:
            self.drill = not self.drill
            if self.drill:
                self.spin = False
                self.spinback = False
                self.gripper = False
                self.last_macro_button = 'drill'
            else:
                self.last_macro_button = None
        self.previous_l1_state = self.l1

    def update_spin(self):
        if self.button_circle and not self.previous_circle_state:
            self.spin = not self.spin
            if self.spin:
                self.drill = False
                self.spinback = False
                self.gripper = False
                self.last_macro_button = 'spin'
            else:
                self.last_macro_button = None
        self.previous_circle_state = self.button_circle

    def update_spinback(self):
        if self.button_square and not self.previous_square_state:
            self.spinback = not self.spinback
            if self.spinback:
                self.drill = False
                self.spin = False
                self.gripper = False
                self.last_macro_button = 'spinback'
            else:
                self.last_macro_button = None
        self.previous_square_state = self.button_square

    def update_gripper(self):
        if self.r1 and not self.previous_r1_state:
            self.gripper = not self.gripper
            if self.gripper:
                self.drill = False
                self.spin = False
                self.spinback = False
                self.last_macro_button = 'gripper'
            else:
                self.last_macro_button = None
        self.previous_r1_state = self.r1

    def update_encoder_reset(self):
        if self.button_triangle and not self.previous_triangle_state:
            self.encoderReset = True
        else:
            self.encoderReset = False
        self.previous_triangle_state = self.button_triangle

    def update_toggle_encoder(self):
        if self.PressedRightAnalog and not self.previous_PressedRightAnalog_state:
            self.toggle_encoder_bool = not self.toggle_encoder_bool

        self.previous_PressedRightAnalog_state = self.PressedRightAnalog

    def reset_toggles(self):
        self.drill = False
        self.spin = False 
        self.spinback = False
        self.gripper = False
        self.last_macro_button = None
        


class Joystick(Node):
    def __init__(self):
        super().__init__("joystick")

        self.pub_move = self.create_publisher(
            Twist, "/quin/cmd_move", qos_profile=qos.qos_profile_system_default
        )

        self.pub_spin = self.create_publisher(
            Twist, "/quin/cmd_spin", qos_profile=qos.qos_profile_system_default
        )

        self.pub_drill = self.create_publisher(
            Twist, "/quin/cmd_drill", qos_profile=qos.qos_profile_system_default
        )

        self.pub_gripper = self.create_publisher(
            Twist, "/quin/cmd_gripper", qos_profile=qos.qos_profile_system_default
        )

        self.pub_encoder = self.create_publisher(
            Twist, "/quin/cmd_encoder", qos_profile=qos.qos_profile_system_default
        )

        self.pub_encoder_reset = self.create_publisher(
            Twist, "/quin/cmd_encode/reset", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Joy, '/quin/joy', self.joy, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.gamepad = Gamepad()
        self.maxspeed : float = 1.0
        


        self.sent_data_timer = self.create_timer(0.01, self.sendData)

    def joy(self, msg):
        
        #Axes:--------------------------------------------------------
        
        self.gamepad.lx = float(msg.axes[0] * -1)                   # 0: Left X-Axis
        self.gamepad.ly = float(msg.axes[1])                        # 1: Left Y-Axis
        self.gamepad.l2 = float((msg.axes[2] + 1)/ 2)               # 2: L2
        self.gamepad.rx = float(msg.axes[3] * -1)                   # 3: Right X-Axis
        self.gamepad.ry = float(msg.axes[4])                        # 4: Right Y-Axis
        self.gamepad.r2 = float((msg.axes[5] + 1)/ 2)               # 5: R2
        self.gamepad.dpadLeftRight  = float(msg.axes[6])            # 6: Dpad Left and Right
        self.gamepad.dpadUpDown     = float(msg.axes[7])            # 7: Dpad Up and Down
        
        #Buttons:-------------------------------------------------------

        self.gamepad.button_cross    = float(msg.buttons[0])        # 0: 
        self.gamepad.button_circle   = float(msg.buttons[1])        # 1:
        self.gamepad.button_triangle = float(msg.buttons[2])        # 2:
        self.gamepad.button_square   = float(msg.buttons[3])        # 3:
        self.gamepad.l1              = float(msg.buttons[4])        # 4:
        self.gamepad.r1              = float(msg.buttons[5])        # 5:
        #self.gamepad.l2              = float(msg.buttons[6])        # 6:
        #self.gamepad.r2              = float(msg.buttons[7])        # 7:
        self.gamepad.button_share    = float(msg.buttons[8])        # 8:
        self.gamepad.button_option   = float(msg.buttons[9])        # 9:
        self.gamepad.button_logo     = float(msg.buttons[10])       # 10:
        self.gamepad.PressedLeftAnalog  = float(msg.buttons[11])    # 11:
        self.gamepad.PressedRightAnalog = float(msg.buttons[12])    # 12:
        
        
        #Macro-----------------------------------------------------------
        
        # self.gamepad.update_drill()
        # self.gamepad.update_gripper()
        self.gamepad.update_toggle_encoder()

        # press=1 release=0
        self.gamepad.spin     = bool(self.gamepad.button_circle) 
        self.gamepad.spinback = bool(self.gamepad.button_square)   
        self.gamepad.drill    = bool(self.gamepad.l1)
        self.gamepad.gripper  = bool(self.gamepad.r1)
        self.gamepad.encoderReset = bool(self.gamepad.button_triangle) 
    

        if self.gamepad.button_logo:
            self.gamepad.reset_toggles()



    def sendData(self):
        
        cmd_vel_move = Twist()
        cmd_encoder = Twist()

        cmd_vel_move.linear.x = float(self.gamepad.ly * self.maxspeed)
        cmd_vel_move.angular.z = float(self.gamepad.rx * self.maxspeed)


        # spin command: -1/0/+1 on angular.z
        cmd_spin = Twist()

        if self.gamepad.spin and not self.gamepad.spinback:
            cmd_spin.angular.z = +1.0         # Circle: clockwise

        elif self.gamepad.spinback and not self.gamepad.spin:
            cmd_spin.angular.z = -1.0         # Square: counter-clockwise

        else:
            cmd_spin.angular.z = 0.0          # Stop
        self.pub_spin.publish(cmd_spin)


        # drill command: -1/0/+1 on linear.z
        cmd_drill = Twist()

        if self.gamepad.drill:
            cmd_drill.linear.z = +1.0         # l1: drill down
        else:
            cmd_drill.linear.z = 0.0          # Stop
        self.pub_drill.publish(cmd_drill)
        

        if self.gamepad.encoder_reset:
+            self.pub_encoder_reset.publish(Empty())
+            self.gamepad.encoder_reset = False


        if self.gamepad.toggle_encoder_bool:
            cmd_encoder.linear.x = 2.0 #RPM
        else:
            cmd_encoder.linear.x = 1.0 #Bit
        

        self.pub_encoder.publish(cmd_encoder)
        self.pub_move.publish(cmd_vel_move)
        self.pub_gripper.publish(cmd_gripper)


def main():
    rclpy.init()

    sub = Joystick()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
