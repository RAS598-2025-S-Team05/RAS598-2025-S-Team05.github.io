#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import sys
import termios
import tty
import os

class MapSaverTrigger(Node):
    def __init__(self):
        super().__init__('map_saver_trigger')
        self.save_path = self.declare_parameter('save_path', '~/ros2_ws/src/turtlebot4_mapping/maps/my_map').get_parameter_value().string_value
        self.get_logger().info("🕹️ Press 'q' or 's' to save the map")

        self.timer = self.create_timer(1.0, self.check_for_keypress)

    def check_for_keypress(self):
        char = self.get_key()
        if char in ['q', 's']:
            self.get_logger().info(f"💾 Saving map to: {self.save_path}")
            try:
                subprocess.run([
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '--ros-args', '--remap', 'map:=/rpi_07/map',
                    '--', '-f', self.save_path
                ], check=True)
                self.get_logger().info("✅ Map saved successfully.")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"❌ Failed to save map: {e}")
            rclpy.shutdown()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def main(args=None):
    rclpy.init(args=args)
    node = MapSaverTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

