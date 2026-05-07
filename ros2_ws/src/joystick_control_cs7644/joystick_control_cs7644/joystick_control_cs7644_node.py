import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control_cs7644_node')

        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 0)
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)
        self.declare_parameter('precise_scale', 10.0)
        self.declare_parameter('fast_scale', 10.0)
        self.declare_parameter('precise', 0) # A button
        self.declare_parameter('fast', 1) # B button

        self.publisher_ = self.create_publisher(Twist, '/vrep/twistCommand', 10)

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info("Vroom")

    def joy_callback(self, msg: Joy):
        twist = Twist()

        axis_linear = self.get_parameter('axis_linear').get_parameter_value().integer_value
        axis_angular = self.get_parameter('axis_angular').get_parameter_value().integer_value
        scale_linear = self.get_parameter('scale_linear').get_parameter_value().double_value
        scale_angular = self.get_parameter('scale_angular').get_parameter_value().double_value
        precise_scale = self.get_parameter('precise_scale').get_parameter_value().double_value
        fast_scale = self.get_parameter('fast_scale').get_parameter_value().double_value
        precise = self.get_parameter('precise').get_parameter_value().integer_value
        fast = self.get_parameter('fast').get_parameter_value().integer_value

        if msg.buttons[precise]:
            linear_scale = scale_linear / precise_scale
            angular_scale = scale_angular / precise_scale
        elif msg.buttons[fast]:
            linear_scale = scale_linear * fast_scale
            angular_scale = scale_angular * fast_scale
        else:
            linear_scale = scale_linear
            angular_scale = scale_angular

        twist.linear.x = linear_scale * msg.axes[axis_linear]
        twist.angular.z = angular_scale * msg.axes[axis_angular]

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
