#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import sin, pi
from geometry_msgs.msg import Twist
import random


class DiversifiedDriver(Node):
    def __init__(self):
        super().__init__("random_driver")
        self.pub = self.create_publisher(Twist, "/vrep/twistCommand", 1)
        # Timer at 1 Hz -> one publish (and one random choice) per second
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        twist = Twist()
        twist.linear.x = random.choice([1.0, 0.5, -0.5, -1.0])
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    driver = DiversifiedDriver()

    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()