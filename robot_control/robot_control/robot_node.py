import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Subscribes to cmd_vel topic for movement commands
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publishes marker to RViz for visualization
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)

        # Timer for updating robot simulation every 0.1s
        self.timer = self.create_timer(0.1, self.update_robot)

        # Last received velocity
        self.last_cmd = Twist()

        # Simulated robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Heading angle in radians

        self.get_logger().info("Robot Control Node has started.")

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg

    def update_robot(self):
        dt = 0.1
        v = self.last_cmd.linear.x
        w = self.last_cmd.angular.z

        # Update robot's position
        self.theta += w * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        # Create and publish marker
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.2
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=1)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
