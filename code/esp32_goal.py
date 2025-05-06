#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from rclpy.action import ActionClient

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('esp32_goal_listener')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, '/rpi_07/navigate_to_pose')

        # Subscriptions
        self.create_subscription(Odometry, '/rpi_07/odom', self.odom_callback, 10)
        self.create_subscription(Point, '/esp32_07/goal', self.esp32_goal_callback, 10)

        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)

        self.origin_pose = None
        self.goal_sent = False

    def odom_callback(self, msg):
        if self.origin_pose is None:
            self.origin_pose = msg.pose.pose
            self.get_logger().info(
                f"✅ Origin pose recorded: x={self.origin_pose.position.x:.2f}, y={self.origin_pose.position.y:.2f}"
            )

    def esp32_goal_callback(self, point_msg):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("⏳ Waiting for Nav2 server...")
            return

        if self.goal_sent:
            return

        goal_x = point_msg.x
        goal_y = point_msg.y

        self.get_logger().info(f"📡 Received goal from ESP32: x={goal_x:.2f}, y={goal_y:.2f}")
        self.send_goal(goal_x, goal_y, self.done_callback)
        self.publish_marker(goal_x, goal_y, marker_id=1, label="ESP32_Goal", color=(1.0, 0.0, 0.0))
        self.goal_sent = True

    def send_goal(self, x, y, done_cb, orientation_w=1.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = orientation_w

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, done_cb))

    def goal_response_callback(self, future, done_cb):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal was rejected.")
            return

        self.get_logger().info("✅ Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(done_cb)

    def done_callback(self, _):
        self.get_logger().info("🏁 Reached the ESP32-sent goal.")
        if self.origin_pose:
            x = self.origin_pose.position.x
            y = self.origin_pose.position.y
            print(f"[📍 Final Robot Position] x: {x:.4f}, y: {y:.4f}")

    def publish_marker(self, x, y, marker_id=0, label="Goal", color=(0.0, 1.0, 0.0), shape=Marker.SPHERE):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = label
        marker.id = marker_id
        marker.type = shape
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

