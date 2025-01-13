import json
import os
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class ManualMotion(Node):
    def __init__(self):
        super().__init__('manual_motion')
        
        # Declare and get the parameter for the JSON file path
        package_share_directory = get_package_share_directory('quadruped_robot')
        self.json_file_path = os.path.join(package_share_directory, 'json', 'motion_sequence.json')


        # Create a publisher
        self.publisher_ = self.create_publisher(Int32MultiArray, '/control_servo_node', 10)

        # Load and publish the motion sequence
        self.publish_motion_sequence()

    def read_json_file(self, file_path):
        """Read the JSON file containing the motion sequence."""
        try:
            with open(file_path, 'r') as file:
                data = json.load(file)
            return data
        except Exception as e:
            self.get_logger().error(f"Failed to read JSON file: {e}")
            return None

    def publish_motion_sequence(self):
        """Publish each step of the motion sequence to the /control_servo_node."""
        json_data = self.read_json_file(self.json_file_path)

        if not json_data or 'sequence' not in json_data:
            self.get_logger().error("Invalid JSON data: 'sequence' key not found.")
            return

        sequence = json_data['sequence']
        self.get_logger().info("Publishing motion sequence...")

        for step in sequence:
            try:
                time_step = step.get('time', 0.0)
                legs = step.get('legs', {})

                # Flatten the joint values into a list of integers
                joint_values = []
                for leg in ['frontLeft', 'frontRight', 'backLeft', 'backRight']:
                    if leg in legs:
                        joint_values.extend([legs[leg].get('hip', 0), 
                                             legs[leg].get('knee', 0), 
                                             legs[leg].get('ankle', 0)])

                # Create and publish the Int32MultiArray message
                msg = Int32MultiArray()
                msg.data = joint_values
                self.publisher_.publish(msg)

                self.get_logger().info(f"Published step at time {time_step}: {joint_values}")

                # Simulate time delay between steps
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=time_step))
            except Exception as e:
                self.get_logger().error(f"Error publishing step: {e}")


def main(args=None):
    rclpy.init(args=args)
    manual_motion = ManualMotion()

    try:
        rclpy.spin(manual_motion)
    except KeyboardInterrupt:
        manual_motion.get_logger().info("Manual motion node terminated.")
    finally:
        manual_motion.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
