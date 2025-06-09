# motion_translator_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

DEBUG = False # Set to false in production

class MotionTranslator(Node):
    def __init__(self):
        super().__init__('motion_translator')

        # Parameters
        self.declare_parameter("max_linear_speed", 0.3)
        self.declare_parameter("linear_increment", 0.03)
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
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_valid_command = "Stop"

        # Missed gesture handling
        self.missed_frames = 0
        self.n1 = 3
        self.n2 = 10
        self.n3 = 15

        # ROS interfaces
        self.gesture_sub = self.create_subscription(String, "/gesture_cmds", self.gesture_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Timer to update motion
        self.timer = self.create_timer(1.0 / self.rate, self.update_motion)

    def gesture_callback(self, msg):
        gesture = msg.data
        if gesture == "Unknown gesture":
            self.missed_frames += 1
        else:
            self.missed_frames = 0
            self.last_valid_command = gesture

    def update_motion(self):
        if self.missed_frames > self.n3:
            # Emergency stop
            target_linear = 0.0
            target_angular = 0.0
            if DEBUG:
                self.get_logger().info(f"[DEBUG] Missed frames > n3 ({self.n3}) → Emergency Stop")
        elif self.missed_frames > self.n2:
            # Safe reduction in speed
            target_linear = max(self.current_linear - self.linear_step, 0)
            target_angular = 0.0
            if DEBUG:
                self.get_logger().info(f"[DEBUG] Missed frames > n2 ({self.n2}) → Safe speed reduction")
        elif self.missed_frames > self.n1:
            # Normal operation or n1 < missed <= n2 fallback
            target_linear, target_angular = self._target_for_command(self.last_valid_command)
            if DEBUG:
                self.get_logger().info(f"[DEBUG] Missed frames > n1 ({self.n1}) → Fallback to last known command: '{self.last_valid_command}'")
        else:
            # Normal operation with valid recent gesture
            target_linear, target_angular = self._target_for_command(self.last_valid_command)
            if DEBUG:
                self.get_logger().info(f"[DEBUG] Missed frames <= n1 ({self.n1}) → Using current gesture: '{self.last_valid_command}'")


        # Smooth transition
        self.current_linear = self._approach(self.current_linear, target_linear, self.linear_step)
        self.current_angular = self._approach(self.current_angular, target_angular, self.angular_step)

        # Publish Twist
        twist = Twist()
        twist.linear.x = float(self.current_linear)
        twist.angular.z = float(self.current_angular)
        self.cmd_pub.publish(twist)

    def _target_for_command(self, cmd):
        if cmd == "Stop":
            return 0.0, 0.0
        elif cmd == "Wait":
            return self.current_linear, self.current_angular
        elif cmd == "Continue moving forward":
            return self.current_linear, 0.0
        elif cmd == "Turn left":
            return 0.0, self.max_angular
        elif cmd == "Turn right":
            return 0.0, -self.max_angular
        elif cmd == "Increase speed":
            return min(self.current_linear + self.linear_step, self.max_linear), 0.0
        else:
            return 0.0, 0.0  # Unknown

    def _approach(self, current, target, step):
        if abs(current - target) < step:
            return target
        return current + step if target > current else current - step

def main(args=None):
    rclpy.init(args=args)
    node = MotionTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
