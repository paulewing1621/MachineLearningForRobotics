import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np


class FloorFollower(Node):
    def __init__(self):
        super().__init__("floor_follower")

        self.bridge = CvBridge()

        self.declare_parameter("forward_speed", 0.3)
        self.declare_parameter("turn_speed", 0.1)

        self.forward_speed = self.get_parameter("forward_speed").value
        self.turn_speed = self.get_parameter("turn_speed").value

        self.create_subscription(Image, "image", self.image_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, "twist", 10)

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = img.shape

        roi = img[int(0.7 * h):h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_grey = np.array([0, 0, 50])
        upper_grey = np.array([180, 40, 200])
        grey_mask = cv2.inRange(hsv, lower_grey, upper_grey)

        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        grey_pixels = np.sum(grey_mask > 0)
        green_pixels = np.sum(green_mask > 0)

        twist = Twist()
        twist.linear.x = self.forward_speed

        if grey_pixels > green_pixels:
            twist.angular.z = -self.turn_speed
        else:
            twist.angular.z = self.turn_speed

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FloorFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()