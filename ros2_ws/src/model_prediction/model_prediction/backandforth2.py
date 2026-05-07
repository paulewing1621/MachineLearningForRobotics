#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import sin, pi
from geometry_msgs.msg import Twist


class DiversifiedDriver(Node):
    def __init__(self):
        super().__init__("diversified_driver")
        self.pub = self.create_publisher(Twist, "/vrep/twistCommand", 1)

        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(0.1, self.timer_cb)  # 10 Hz update

    def timer_cb(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self.t0
        twist = Twist()

        # Periodic phases every 30 seconds
        phase = int(t // 10) % 6  # Each phase lasts 20 seconds

        if phase == 5:
            # Phase 5: Constant velocity (slow forward)
            twist.linear.x = 0.5

        elif phase == 4:
            # Phase 4: Constant velocity (slow backward)
            twist.linear.x = -0.7

        elif phase == 3:
            # Phase 3: Short bursts
            if int(t * 2) % 2 == 0:  # toggle every 0.5s
                twist.linear.x = 0.8
            else:
                twist.linear.x = 0.0

        elif phase == 2:
            # Phase 2: Sine wave (smooth oscillation)
            twist.linear.x = sin(2 * pi * 0.4 * t)  # frequency 0.2 Hz

        elif phase == 1:
            # Phase 1: Slow ramp transition between -1 and 1
            twist.linear.x = ((t % 5) / 5.0) * 2.0 - 1.0  # ramp -1 → +1

        elif phase == 5:
            # Phase 5: Stop (all zeros)
            twist.linear.x = 0.0

        # Publish
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    driver = DiversifiedDriver()

    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()