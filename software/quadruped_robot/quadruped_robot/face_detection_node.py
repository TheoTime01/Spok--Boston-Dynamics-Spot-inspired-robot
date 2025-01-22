import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class HeadDetectionNode(Node):
    def __init__(self):
        super().__init__('head_detection_node')

        # Create a subscriber for the input video
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Create a publisher for the output video
        self.publisher_ = self.create_publisher(Image, 'output_video', 10)

        # Initialize the OpenCV bridge
        self.bridge = CvBridge()

        self.counter = 0

        # Load the Haar cascade for head detection
        self.head_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.get_logger().info("Face detection node started")

    def image_callback(self, msg):

        self.counter += 1

        if self.counter%10 == 0:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                heads = self.head_cascade.detectMultiScale(
                    gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
                )

                for (x, y, w, h) in heads:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

                output_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

                self.publisher_.publish(output_msg)
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HeadDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down head detection node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
