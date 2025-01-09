import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image
from cv_bridge import CvBridge
import cv2

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscribers to /sonar_1 and /sonar_2
        self.sub_sonar_1 = self.create_subscription(
            Range, '/sonar_1', self.sonar_1_callback, 10
        )
        self.sub_sonar_2 = self.create_subscription(
            Range, '/sonar_2', self.sonar_2_callback, 10
        )

        # Subscriber to /camera/image_raw
        self.sub_camera = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )
        
        # Initialize variables to store the range data
        self.sonar_1_range = None
        self.sonar_2_range = None

        # Initialize CvBridge for converting ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        self.get_logger().info("Sensor Fusion Node has been started.")

    def sonar_1_callback(self, msg):
        self.sonar_1_range = msg.range
        self.compute_average()

    def sonar_2_callback(self, msg):
        self.sonar_2_range = msg.range
        self.compute_average()

    def compute_average(self):
        if self.sonar_1_range is not None and self.sonar_2_range is not None:
            self.avg_distance = (self.sonar_1_range + self.sonar_2_range) / 2.0
        else:
            self.avg_distance = None

    def camera_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Draw the average distance on the image if available
            if hasattr(self, 'avg_distance') and self.avg_distance is not None:
                text = f"Distance: {self.avg_distance:.2f} m"
                cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Display the image using OpenCV
            cv2.imshow("Camera View with Distance", cv_image)
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