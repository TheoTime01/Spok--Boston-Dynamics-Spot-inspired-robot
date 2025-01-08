import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointPositionPrinter(Node):
    def __init__(self):
        super().__init__('joint_position_printer')
        # Subscribe to the /joint_states topic to get joint positions
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # Topic where joint angles are published
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Extract joint names and positions
        joint_names = msg.name
        positions = msg.position  # Joint angles in radians

        # Convert positions to degrees and log the results
        for joint, position in zip(joint_names, positions):
            position_degrees = position * (180.0 / 3.14159)  # Convert radians to degrees
            self.get_logger().info(f"Joint: {joint}, Position (degrees): {position_degrees:.2f}")

def main(args=None):
    rclpy.init(args=args)
    joint_position_converter = JointPositionPrinter()

    # Spin the node to keep it running and listening for messages
    rclpy.spin(joint_position_converter)

    # Shutdown the node after spinning
    joint_position_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
