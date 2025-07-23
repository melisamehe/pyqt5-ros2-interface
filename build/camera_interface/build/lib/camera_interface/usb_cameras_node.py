import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class UsbCamerasNode(Node):
    def __init__(self):
        super().__init__('usb_cameras_node')
        self.bridge = CvBridge()

        # --- Geri Sürüş Kamerası (Laptop Kamerası) ---
        self.rear_cam_capture = cv2.VideoCapture(8) 

        # --- Nişan Kamerası (Logitech Brio 100) ---
        self.aim_cam_capture = cv2.VideoCapture(6)

        # Kameraların açılıp açılmadığını kontrol edelim
        if not self.rear_cam_capture.isOpened():
            self.get_logger().error('HATA: Laptop kamerası (index 8) açılamadı.')
        if not self.aim_cam_capture.isOpened():
            self.get_logger().error('HATA: Logitech kamera (index 6) açılamadı.')

        self.rear_cam_publisher = self.create_publisher(Image, '/camera/rear/image_raw', 10)
        self.aim_cam_publisher = self.create_publisher(Image, '/camera/aiming/image_raw', 10)
        self.timer = self.create_timer(1/30.0, self.publish_frames)
        self.get_logger().info('USB Kameralar (Laptop ve Logitech) yayına başladı.')

    def publish_frames(self):
        if self.rear_cam_capture.isOpened():
            ret_rear, frame_rear = self.rear_cam_capture.read()
            if ret_rear:
                self.rear_cam_publisher.publish(self.bridge.cv2_to_imgmsg(frame_rear, "bgr8"))

        if self.aim_cam_capture.isOpened():
            ret_aim, frame_aim = self.aim_cam_capture.read()
            if ret_aim:
                self.aim_cam_publisher.publish(self.bridge.cv2_to_imgmsg(frame_aim, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = UsbCamerasNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()