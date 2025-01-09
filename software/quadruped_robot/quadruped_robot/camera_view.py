import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscriber to /camera/image_raw
        self.sub_camera = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )

        # Initialize CvBridge for converting ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        self.get_logger().info("Sensor Fusion Node has been started.")

    def camera_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the image using OpenCV
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
