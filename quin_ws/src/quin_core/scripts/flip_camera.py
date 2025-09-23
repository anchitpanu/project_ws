#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Flip180(Node):
    def __init__(self):
        super().__init__('flip180')
        self.in_topic  = self.declare_parameter('in_topic',  '/quin/image_raw').get_parameter_value().string_value
        self.out_topic = self.declare_parameter('out_topic', '/quin/image_raw/upright').get_parameter_value().string_value
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.in_topic, self.cb, 10)
        self.pub = self.create_publisher(Image, self.out_topic, 10)
        self.get_logger().info(f'Rotate 180°: {self.in_topic} -> {self.out_topic}')

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        rot = cv2.rotate(img, cv2.ROTATE_180)
        out = self.bridge.cv2_to_imgmsg(rot, encoding='rgb8')
        out.header = msg.header
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(Flip180())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
