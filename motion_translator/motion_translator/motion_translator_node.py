# motion_translator_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MotionTranslator(Node):
    def __init__(self):
        super().__init__('motion_translator')

        # Parameters
        self.declare_parameter("max_linear_speed", 0.3)
        self.declare_parameter("linear_increment", 0.05)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("angular_increment", 0.1)
        self.declare_parameter("update_rate", 10.0)

        # Load parameters
        self.max_linear = self.get_parameter("max_linear_speed").value
        self.linear_step = self.get_parameter("linear_increment").value
        self.max_angular = self.get_parameter("max_angular_speed").value
        self.angular_step = self.get_parameter("angular_increment").value
        self.rate = self.get_parameter("update_rate").value

        # Current motion state
        self.target_gesture = "stop"
        self.current_linear = 0.0
        self.current_angular = 0.0

        # ROS interfaces
        self.gesture_sub = self.create_subscription(String, "/gesture_cmds", self.gesture_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Timer to update motion
        self.timer = self.create_timer(1.0 / self.rate, self.update_motion)

    def gesture_callback(self, msg):
        self.target_gesture = msg.data.lower()

    def update_motion(self):
        # Reset speeds
        target_linear = 0.0
        target_angular = 0.0

        if self.target_gesture == "forward":
            target_linear = self.max_linear
        elif self.target_gesture == "left":
            target_angular = self.max_angular
        elif self.target_gesture == "right":
            target_angular = -self.max_angular
        elif self.target_gesture == "stop":
            pass  # stay at zero

        # Smoothly adjust speeds
        self.current_linear = self._approach(self.current_linear, target_linear, self.linear_step)
        self.current_angular = self._approach(self.current_angular, target_angular, self.angular_step)

        # Publish command
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_pub.publish(twist)

    def _approach(self, current, target, step):
        if abs(target - current) < step:
            return target
        return current + step if target > current else current - step

def main(args=None):
    rclpy.init(args=args)
    node = MotionTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

