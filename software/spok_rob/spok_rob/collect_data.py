import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import csv

class JointTrajectorySubscriber(Node):
    def __init__(self):
        super().__init__('joint_trajectory_subscriber')
        
        # Create a subscription to the /joint_group_effort_controller/joint_trajectory topic
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_group_effort_controller/joint_trajectory',
            self.listener_callback,
            10
        )
        
        # Initialize CSV file
        self.csv_file = '/home/tototime/ros2_spok_ws/src/S7_G7_Perrichet_Sibenaler/software/spok_rob/spok_rob/joint_trajectory_sit_down_data.csv'
        self.counter = 0  # Message sequence counter

        # Write the header to the CSV file
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['i', 'joint_names', 'positions'])

        self.get_logger().info(f'Subscriber node initialized. Writing data to {self.csv_file}')

    def listener_callback(self, msg):
        # Increment message counter
        self.counter += 1

        # Extract data from the message
        joint_names = msg.joint_names
        if msg.points:
            positions = msg.points[0].positions  # Assuming only one point
        else:
            positions = []

        # Write data to the CSV file
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([self.counter, joint_names, positions])
        
        self.get_logger().info(f'Logged message {self.counter}')

        # Stop the node if the counter reaches 100
        if self.counter >= 2000:
            self.get_logger().info(f'Maximum message count reached ({self.counter}). Shutting down node.')
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectorySubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        # Shutdown the node and cleanup
        rclpy.shutdown()

if __name__ == '__main__':
    main()
