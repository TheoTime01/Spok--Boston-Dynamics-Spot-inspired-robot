import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarRepublisher(Node):
    def __init__(self):
        super().__init__('lidar_republisher')
        # Create a subscriber to '/lidar_link/scan'
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar_link/scan',
            self.lidar_callback,
            10
        )
        # Create a publisher to '/scan'
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info('Lidar Republisher Node has been started.')

    def lidar_callback(self, msg):
        # Log the reception of a message (optional)
        self.get_logger().info('Received message on /lidar_link/scan')
        # Publish the same message on '/scan'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node terminated.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
