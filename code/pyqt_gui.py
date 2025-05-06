import os
import sys
import threading
import subprocess
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, CompressedImage, LaserScan
from cv_bridge import CvBridge
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, QSlider
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
import cv2

os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/arm-linux-gnueabihf/qt5/plugins/platforms"

def apply_low_pass_filter(new_val, prev_val, alpha=0.3):
    return alpha * new_val + (1 - alpha) * prev_val

class IMUPlot(FigureCanvas):
    def __init__(self, max_points=100):
        self.fig = Figure(figsize=(5, 3))
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("IMU - Angular Velocity (rad/s)", fontsize=12)
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Velocity")
        self.ax.grid(True)
        self.ax.set_facecolor("#f9f9f9")
        self.max_points = max_points
        self.counter = 0
        self.x_vals = deque(maxlen=max_points)
        self.y_vals = deque(maxlen=max_points)
        self.z_vals = deque(maxlen=max_points)
        self.ticks = deque(maxlen=max_points)
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_z = 0.0
        self.line_x, = self.ax.plot([], [], label="X-axis", color='r')
        self.line_y, = self.ax.plot([], [], label="Y-axis", color='g')
        self.line_z, = self.ax.plot([], [], label="Z-axis", color='b')
        self.ax.legend(loc="upper right")

    def update_plot(self, x, y, z):
        self.last_x = apply_low_pass_filter(x, self.last_x)
        self.last_y = apply_low_pass_filter(y, self.last_y)
        self.last_z = apply_low_pass_filter(z, self.last_z)
        self.counter += 1
        self.ticks.append(self.counter)
        self.x_vals.append(self.last_x)
        self.y_vals.append(self.last_y)
        self.z_vals.append(self.last_z)
        self.line_x.set_data(self.ticks, self.x_vals)
        self.line_y.set_data(self.ticks, self.y_vals)
        self.line_z.set_data(self.ticks, self.z_vals)
        self.ax.relim()
        self.ax.autoscale_view()
        self.draw()

class LIDARPlot(FigureCanvas):
    def __init__(self, max_points=360):
        self.max_points = max_points
        self.fig = Figure(figsize=(5, 5))
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.zoom_factor = 1.0
        self.current_ranges = []
        self.colorbar = None
        self._draw_static()

    def _draw_static(self):
        self.ax.set_title("LIDAR PointCloud Visualization", fontsize=12)
        self.ax.set_facecolor("#f9f9f9")
        limit = 4 / self.zoom_factor
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(0, 0.1)

    def set_zoom(self, factor):
        self.zoom_factor = factor
        self.refresh_plot()

    def update_data(self, ranges):
        self.current_ranges = ranges

    def refresh_plot(self):
        self.ax.clear()
        self._draw_static()
        ranges = np.array(self.current_ranges[:self.max_points])
        angles = np.linspace(0, 2 * np.pi, len(ranges))
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)
        valid = ranges > 0.05
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        z = np.zeros_like(x)
        sc = self.ax.scatter(x, y, z, c=ranges[valid], cmap='viridis', s=10, label='LIDAR Points')
        if self.colorbar:
            self.colorbar.remove()
        self.colorbar = self.fig.colorbar(sc, ax=self.ax, shrink=0.5, label='Distance (m)')
        self.ax.legend(loc='upper right')
        self.draw()

class SensorGUI(Node):
    def __init__(self):
        super().__init__('sensor_gui_node')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.imu_data = None
        self.image = None
        self.lidar_ranges = None
        self.imu_plot_enabled = True
        self.cam_enabled = True
        self.lidar_enabled = True
        self.bridge = CvBridge()
        self.bag_process = None
        self.create_subscription(Imu, '/rpi_07/imu', self.imu_callback, qos)
        self.create_subscription(CompressedImage, '/rpi_07/oakd_camera/rgb/image_raw/compressed', self.image_callback, qos)
        self.create_subscription(LaserScan, '/rpi_07/scan', self.lidar_callback, qos)
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Sensor Data Viewer")
        self.imu_plot = IMUPlot(max_points=100)
        self.lidar_plot = LIDARPlot(max_points=360)
        self.image_label = QLabel("Waiting for camera image...")
        self.image_label.setFixedHeight(480)

        camera_heading = QLabel("Camera Preview")
        camera_heading.setStyleSheet("font-weight: bold; font-size: 16px")
        lidar_heading = QLabel("LIDAR Visualization")
        lidar_heading.setStyleSheet("font-weight: bold; font-size: 16px")
        imu_heading = QLabel("IMU Angular Velocity Plot")
        imu_heading.setStyleSheet("font-weight: bold; font-size: 16px")

        main_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        sensor_box = QVBoxLayout()
        cam_layout = QVBoxLayout()
        cam_layout.addWidget(camera_heading)
        cam_layout.addWidget(self.image_label)
        lidar_layout = QVBoxLayout()
        lidar_layout.addWidget(lidar_heading)
        lidar_layout.addWidget(self.lidar_plot)
        zoom_slider = QSlider(Qt.Horizontal)
        zoom_slider.setMinimum(1)
        zoom_slider.setMaximum(10)
        zoom_slider.setValue(1)
        zoom_slider.valueChanged.connect(lambda val: self.lidar_plot.set_zoom(val))
        lidar_layout.addWidget(QLabel("Zoom"))
        lidar_layout.addWidget(zoom_slider)
        top_layout.addLayout(cam_layout)
        top_layout.addLayout(lidar_layout)

        sensor_box.addWidget(imu_heading)
        sensor_box.addWidget(self.imu_plot)
        control_layout = QHBoxLayout()
        self._add_toggle_button(control_layout, "Start IMU", lambda: self.set_imu_plot(True))
        self._add_toggle_button(control_layout, "Stop IMU", lambda: self.set_imu_plot(False))
        self._add_toggle_button(control_layout, "Start Camera", lambda: self.set_cam(True))
        self._add_toggle_button(control_layout, "Stop Camera", lambda: self.set_cam(False))
        self._add_toggle_button(control_layout, "Start LIDAR", lambda: self.set_lidar(True))
        self._add_toggle_button(control_layout, "Stop LIDAR", lambda: self.set_lidar(False))
        self._add_toggle_button(control_layout, "Start ROSBag", self.start_rosbag)
        self._add_toggle_button(control_layout, "Stop ROSBag", self.stop_rosbag)
        sensor_box.addLayout(control_layout)

        main_layout.addLayout(top_layout)
        main_layout.addLayout(sensor_box)
        self.window.setLayout(main_layout)
        self.window.resize(1200, 1000)
        self.window.show()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)

    def _add_toggle_button(self, layout, label, callback):
        btn = QPushButton(label)
        btn.clicked.connect(callback)
        layout.addWidget(btn)

    def imu_callback(self, msg):
        self.imu_data = msg

    def image_callback(self, msg):
        if not self.cam_enabled:
            return
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            rgb_bytes = rgb_image.tobytes()
            qt_img = QImage(rgb_bytes, w, h, ch * w, QImage.Format_RGB888)
            self.image = QPixmap.fromImage(qt_img).scaled(640, 480)
        except Exception as e:
            self.get_logger().error(f"Compressed image conversion failed: {e}")

    def lidar_callback(self, msg):
        if not self.lidar_enabled:
            return
        self.lidar_ranges = msg.ranges
        self.lidar_plot.update_data(self.lidar_ranges)

    def update_gui(self):
        if self.imu_data and self.imu_plot_enabled:
            imu = self.imu_data
            self.imu_plot.update_plot(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z)
        if self.cam_enabled and self.image:
            self.image_label.setPixmap(self.image)
        if self.lidar_enabled and self.lidar_ranges:
            self.lidar_plot.refresh_plot()

    def set_imu_plot(self, enabled: bool):
        self.imu_plot_enabled = enabled

    def set_cam(self, enabled: bool):
        self.cam_enabled = enabled

    def set_lidar(self, enabled: bool):
        self.lidar_enabled = enabled

    def start_rosbag(self):
        if self.bag_process is None:
            bag_path = os.path.expanduser("~/Documents/sensor_data")
            self.bag_process = subprocess.Popen(["ros2", "bag", "record", "-o", bag_path,
                                                 "/rpi_07/imu",
                                                 "/rpi_07/oakd_camera/rgb/image_raw/compressed",
                                                 "/rpi_07/scan"])
            self.get_logger().info("Started recording rosbag")

    def stop_rosbag(self):
        if self.bag_process:
            self.bag_process.terminate()
            self.bag_process = None
            self.get_logger().info("Stopped recording rosbag")

    def run(self):
        sys.exit(self.app.exec_())

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = SensorGUI()
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# ... [Rest of the code remains unchanged] ...

