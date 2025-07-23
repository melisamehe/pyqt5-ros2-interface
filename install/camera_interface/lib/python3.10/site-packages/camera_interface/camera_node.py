import sys
import cv2
import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QGroupBox
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QThread

class ROS2Thread(QThread):
    # Sinyaller aynı kalıyor
    new_frame_signal_main = pyqtSignal(np.ndarray)
    new_frame_signal_rear = pyqtSignal(np.ndarray)
    new_frame_signal_aiming = pyqtSignal(np.ndarray)
    lidar_signal = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.node = None

    def run(self):
        # ROS2 abonelikleri de aynı kalıyor
        rclpy.init()
        self.node = Node('interface_subscriber_node')
        self.bridge = CvBridge()
        self.main_sub = self.node.create_subscription(
            Image, '/camera/color/image_raw', self.main_callback, 10)
        self.rear_sub = self.node.create_subscription(
            Image, '/camera/rear/image_raw', self.rear_callback, 10)
        self.aiming_sub = self.node.create_subscription(
            Image, '/camera/aiming/image_raw', self.aiming_callback, 10)
        self.lidar_sub = self.node.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.node.get_logger().info('Tüm sensör dinleyicileri başlatıldı.')
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

    # Callback fonksiyonları da aynı
    def main_callback(self, msg):
        self.new_frame_signal_main.emit(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
    def rear_callback(self, msg):
        self.new_frame_signal_rear.emit(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
    def aiming_callback(self, msg):
        self.new_frame_signal_aiming.emit(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
    def lidar_callback(self, msg):
        self.lidar_signal.emit(msg)

class MergedInterface(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IKA - Birleşik Arayüz (LIDAR Odaklı)")
        self.setGeometry(100, 100, 1250, 850) # Pencere boyutunu ayarladık
        self.initUI()

    def initUI(self):
        main_layout = QHBoxLayout()

        # --- DEĞİŞİKLİK: SOL TARAF ARTIK LIDAR ---
        self.lidar_label = QLabel("LIDAR - Awaiting data...")
        self.lidar_label.setFixedSize(800, 800) # LIDAR için büyük kare alan
        self.lidar_label.setStyleSheet("background-color: black; border: 2px solid green; color: white;")
        self.lidar_label.setAlignment(Qt.AlignCenter)

        # Sağ taraf: Tüm kameralar için dikey bir layout
        right_panel_layout = QVBoxLayout()

        # --- DEĞİŞİKLİK: REALSENSE KAMERA SAĞ ÜSTTE ---
        self.main_camera_label = QLabel("Forward View (RealSense) - Awaiting data...")
        self.main_camera_label.setFixedSize(400, 300)
        self.main_camera_label.setStyleSheet("background-color: #2C3E50; border: 1px solid #3498DB; color: white;")
        self.main_camera_label.setAlignment(Qt.AlignCenter)

        self.rear_camera_label = QLabel("Rear View - Awaiting data...")
        self.rear_camera_label.setFixedSize(400, 300)
        self.rear_camera_label.setStyleSheet("background-color: #17202A; border: 1px solid gray; color: white;")
        self.rear_camera_label.setAlignment(Qt.AlignCenter)

        self.aim_camera_label = QLabel("Aiming View - Awaiting data...")
        self.aim_camera_label.setFixedSize(400, 300)
        self.aim_camera_label.setStyleSheet("background-color: #17202A; border: 1px solid gray; color: white;")
        self.aim_camera_label.setAlignment(Qt.AlignCenter)

        # Grupları oluşturup sağ panele ekliyoruz
        main_cam_group = QGroupBox("Forward Camera (RealSense)")
        main_cam_group.setLayout(QVBoxLayout())
        main_cam_group.layout().addWidget(self.main_camera_label)

        rear_group = QGroupBox("Rear Camera")
        rear_group.setLayout(QVBoxLayout())
        rear_group.layout().addWidget(self.rear_camera_label)

        aim_group = QGroupBox("Aiming Camera")
        aim_group.setLayout(QVBoxLayout())
        aim_group.layout().addWidget(self.aim_camera_label)

        right_panel_layout.addWidget(main_cam_group)
        right_panel_layout.addWidget(rear_group)
        right_panel_layout.addWidget(aim_group)

        # Ana layout'u birleştiriyoruz
        main_layout.addWidget(self.lidar_label) # Sol tarafa LIDAR
        main_layout.addLayout(right_panel_layout) # Sağ tarafa kameralar
        self.setLayout(main_layout)

    # Slot fonksiyonları aynı, sadece boyutları değişti
    @pyqtSlot(np.ndarray)
    def update_main_slot(self, cv_img):
        self.main_camera_label.setPixmap(self.frame_to_pixmap(cv_img, 400, 300))
    @pyqtSlot(np.ndarray)
    def update_rear_slot(self, cv_img):
        self.rear_camera_label.setPixmap(self.frame_to_pixmap(cv_img, 400, 300))
    @pyqtSlot(np.ndarray)
    def update_aiming_slot(self, cv_img):
        self.aim_camera_label.setPixmap(self.frame_to_pixmap(cv_img, 400, 300))

    @pyqtSlot(object)
    def update_lidar_slot(self, scan_msg):
        size = self.lidar_label.width()
        img = np.zeros((size, size, 3), dtype=np.uint8)
        center = size // 2
        max_range = 5.0

        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < max_range:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                px = int(center + x * (size / (max_range * 2)))
                py = int(center - y * (size / (max_range * 2)))
                if 0 <= px < size and 0 <= py < size:
                    img[py, px] = (0, 255, 0)
            angle += scan_msg.angle_increment

        cv2.circle(img, (center, center), 5, (255, 0, 0), -1)
        self.lidar_label.setPixmap(self.frame_to_pixmap(img, size, size))

    def frame_to_pixmap(self, frame, width, height):
        # Bu fonksiyonu tekilleştirdik
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            qt_image = QImage(rgb_image.data, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        else: # Bu kısım LIDAR'ın siyah-beyaz görüntüsü için
            qt_image = QImage(frame.data, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        return QPixmap.fromImage(qt_image).scaled(width, height, Qt.KeepAspectRatio)

def main():
    app = QApplication(sys.argv)
    arayuz = MergedInterface()
    ros_thread = ROS2Thread()
    # Bağlantılar aynı
    ros_thread.new_frame_signal_main.connect(arayuz.update_main_slot)
    ros_thread.new_frame_signal_rear.connect(arayuz.update_rear_slot)
    ros_thread.new_frame_signal_aiming.connect(arayuz.update_aiming_slot)
    ros_thread.lidar_signal.connect(arayuz.update_lidar_slot)
    ros_thread.start()
    arayuz.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()