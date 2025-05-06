#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time

class LidarAvoid(Node):
    def __init__(self):
        super().__init__('lidar_avoidance_node')
        self.cmd_pub = self.create_publisher(Twist, '/rpi_07/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/rpi_07/scan', self.scan_callback, 10)

        self.linear_speed = 0.15
        self.angular_speed = 1.5
        self.response_dist = 0.4
        self.sector_angle = 60
        self.turning = False
        self.turn_end_time = 0.0
        self.start_time = time.time()  # ✅ startup delay

        self.get_logger().info("✅ LiDAR obstacle avoidance node ready.")

    def scan_callback(self, msg: LaserScan):
        if time.time() - self.start_time < 5.0:
            self.get_logger().info("⏳ Waiting for RViz to start... (Startup delay)")
            return

        self.get_logger().info("📡 LiDAR scan callback triggered.")

        # Clean up ranges
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        ranges = np.where(np.isnan(ranges), msg.range_max, ranges)

        total_angles = len(ranges)
        mid = total_angles // 2
        delta = int((self.sector_angle / 360.0) * total_angles)

        front_ranges = ranges[mid - delta : mid + delta]
        min_dist = np.min(front_ranges)

        self.get_logger().info(f"📏 Closest obstacle: {min_dist:.2f} m")

        twist = Twist()
        current_time = time.time()

        # Phase 1: Still turning
        if self.turning and current_time < self.turn_end_time:
            twist.angular.z = self.angular_speed * 0.5  # Reduced turn rate
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().warn("🔁 Turning to avoid obstacle...")
            return

        # Phase 2: Pause after turn
        if self.turning and current_time < self.turn_end_time + 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("⏸️ Pausing after turn...")
            return
        else:
            self.turning = False

        # Phase 3: Move or start new turn
        if min_dist < self.response_dist:
            self.turning = True
            self.turn_end_time = current_time + 1.0  # 1s turn
            twist.angular.z = self.angular_speed * 0.5
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().warn("⚠️ Obstacle detected! Initiating turn.")
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("✅ Path clear, moving forward.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

