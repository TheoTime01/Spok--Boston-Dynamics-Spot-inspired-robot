import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class SonarAverageNode(Node):
    def __init__(self):
        super().__init__('sonar_average_node')

        # Subscribers to /sonar_1 and /sonar_2
        self.sub_sonar_1 = self.create_subscription(
            Range, '/sonar_1', self.sonar_1_callback, 10
        )
        self.sub_sonar_2 = self.create_subscription(
            Range, '/sonar_2', self.sonar_2_callback, 10
        )

        # Initialize variables to store the range data
        self.sonar_1_range = None
        self.sonar_2_range = None

        self.get_logger().info("Sonar Average Node has been started.")

    def sonar_1_callback(self, msg):
        self.sonar_1_range = msg.range
        self.compute_average()

    def sonar_2_callback(self, msg):
        self.sonar_2_range = msg.range
        self.compute_average()

    def compute_average(self):
        if self.sonar_1_range is not None and self.sonar_2_range is not None:
            avg_distance = (self.sonar_1_range + self.sonar_2_range) / 2.0
            self.get_logger().info(f"Distance : {avg_distance:.2f} meters")


def main(args=None):
    rclpy.init(args=args)
    node = SonarAverageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
