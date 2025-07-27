import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from std_msgs.msg import ColorRGBA

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Subscribes to cmd_vel topic for movement commands
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publishes marker to RViz for visualization
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)

        # Timer for updating robot simulation every 0.1s
        self.timer = self.create_timer(0.1, self.update_robot)

        # Add this line to create a publisher for path visualization
        self.path_pub = self.create_publisher(Marker, '/robot_path', 10)

        # Last received velocity
        self.last_cmd = Twist()

        # Simulated robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Heading angle in radians

        self.path_points = []

        self.get_logger().info("Robot Control Node with heading and path visualization started.")

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

        # Save current position to path points
        self.path_points.append((self.x, self.y))

        # Limit path points length (optional)
        max_path_length = 200
        if len(self.path_points) > max_path_length:
            self.path_points.pop(0)

        ## Publish robot base marker (cube)
        base_marker = Marker()
        base_marker.header.frame_id = 'map'
        base_marker.header.stamp = self.get_clock().now().to_msg()
        base_marker.ns = 'robot'
        base_marker.id = 0
        base_marker.type = Marker.CUBE
        base_marker.action = Marker.ADD
        base_marker.pose.position.x = self.x
        base_marker.pose.position.y = self.y
        base_marker.pose.position.z = 0.1
        base_marker.pose.orientation.w = 1.0
        base_marker.scale.x = 0.3
        base_marker.scale.y = 0.2
        base_marker.scale.z = 0.1
        base_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=1.0)
        base_marker.lifetime = Duration(sec=1)
        self.marker_pub.publish(base_marker)

        # Publish heading arrow marker
        arrow_marker = Marker()
        arrow_marker.header.frame_id = 'map'
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = 'robot'
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        arrow_marker.scale.x = 0.03  # shaft length
        arrow_marker.scale.y = 0.06  # shaft diameter
        arrow_marker.scale.z = 0.12  # head diameter

        arrow_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        arrow_marker.points = []

        from geometry_msgs.msg import Point

        start_point = Point()
        start_point.x = self.x
        start_point.y = self.y
        start_point.z = 0.15

        end_point = Point()
        # Arrow points in heading direction
        arrow_length = 0.5
        end_point.x = self.x + arrow_length * math.cos(self.theta)
        end_point.y = self.y + arrow_length * math.sin(self.theta)
        end_point.z = 0.15

        arrow_marker.points.append(start_point)
        arrow_marker.points.append(end_point)

        arrow_marker.lifetime = Duration(sec=1)
        self.marker_pub.publish(arrow_marker)

        # Publish path marker (line strip)
        path_marker = Marker()
        path_marker.header.frame_id = 'map'
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = 'robot_path'
        path_marker.id = 2
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05  # line width
        path_marker.color = ColorRGBA(r=0.0, g=0.7, b=1.0, a=1.0)
        path_marker.points = []

        for px, py in self.path_points:
            pt = Point()
            pt.x = px
            pt.y = py
            pt.z = 0.1
            path_marker.points.append(pt)

        path_marker.lifetime = Duration(sec=1)
        self.path_pub.publish(path_marker)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
