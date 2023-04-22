import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_topic',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Convert OpenCV image to QImage
        height, width, channels = cv_image.shape
        q_image = QImage(cv_image.data, width, height, channels * width, QImage.Format_BGR888)
        
        # Do something with the QImage here...

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
