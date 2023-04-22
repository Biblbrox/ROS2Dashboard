import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.timer_ = self.create_timer(0.1, self.publish_image)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
