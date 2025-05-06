#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import re
import time

class ESP32GoalPublisher(Node):
    def __init__(self):
        super().__init__('esp32_goal_publisher')
        self.publisher_ = self.create_publisher(Point, '/esp32_07/goal', 10)

        # 🛠️ Update to match your actual port (e.g., COM3 or /dev/ttyUSB0)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        time.sleep(2)  # Allow serial to initialize
        self.get_logger().info("📡 ESP32 Serial opened. Waiting for data...")

        self.timer = self.create_timer(0.5, self.publish_goal_once)
        self.goal_sent = False

    def publish_goal_once(self):
        if self.goal_sent:
            return  # Don't resend goal

        try:
            line = self.ser.readline().decode('utf-8').strip()

            # Match format like: "X: 3.4375  Y: -1.8750  Z: 2.5625"
            match = re.search(r'X:\s*(-?\d+\.?\d*)\s*Y:\s*(-?\d+\.?\d*)\s*Z:\s*(-?\d+\.?\d*)', line)
            if not match:
                self.get_logger().warn("⚠️ Skipping unrecognized format.")
                return

            # Parse and (optionally) apply offsets if needed
            x, y, z = map(float, match.groups())

            # Optional static offset (use if ESP32 is placed in known relative frame)
            OFFSET_X = 0.0
            OFFSET_Y = 0.0

            x_map = x + OFFSET_X
            y_map = y + OFFSET_Y

            point = Point()
            point.x = x_map
            point.y = y_map
            point.z = 0.0  # We ignore Z for 2D Nav

            self.publisher_.publish(point)
            self.get_logger().info(f"✅ Published goal: x={x_map:.2f}, y={y_map:.2f}")
            self.goal_sent = True

        except Exception as e:
            self.get_logger().error(f"❌ Serial error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
