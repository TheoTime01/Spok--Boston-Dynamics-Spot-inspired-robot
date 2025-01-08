import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from rclpy.clock import ROSClock


class MedorTFBroadcaster(Node):
    def __init__(self):
        super().__init__('medor_tf_broadcaster')

        # Initialize the TransformBroadcaster
        self.br = TransformBroadcaster(self)

        # Create a timer to periodically send the transforms
        self.timer = self.create_timer(0.1, self.broadcast_transform)  # 10 Hz

    def broadcast_transform(self):
        # Get the current time
        now = ROSClock().now()

        # Create a TransformStamped message for the arm transformation
        t = TransformStamped()

        # Fill the message with data
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'second_arm'
        t.child_frame_id = 'third_arm'  # Example child frame

        # Set the translation (x, y, z)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0  # Example height from base_link to arm_base_link

        # Set the rotation (using quaternions)
        # Assuming no rotation, just identity quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send the transform
        self.br.sendTransform(t)

        # You can add additional transforms for other joints if needed.
        # Example of broadcasting another joint transform:
        self.broadcast_joint_transform(now)

    def broadcast_joint_transform(self, now):
        # Create a TransformStamped message for a joint transformation (e.g., arm_joint1)
        joint_transform = TransformStamped()

        joint_transform.header.stamp = now.to_msg()
        joint_transform.header.frame_id = 'first_rot_link_joint'
        joint_transform.child_frame_id = 'second_rot_link_joint'

        # Set translation and rotation (you can dynamically compute these based on joint state)
        joint_transform.transform.translation.x = 0.0
        joint_transform.transform.translation.y = 1.0
        joint_transform.transform.translation.z = 0.0  # Example offset

        # Rotation around the z-axis
        angle = math.sin(ROSClock().now().seconds_nanoseconds()[0])  # Simple sinusoidal motion
        joint_transform.transform.rotation.x = 0.0
        joint_transform.transform.rotation.y = 0.0
        joint_transform.transform.rotation.z = math.sin(angle / 2.0)
        joint_transform.transform.rotation.w = math.cos(angle / 2.0)

        # Broadcast the joint transform
        self.br.sendTransform(joint_transform)


def main(args=None):
    rclpy.init(args=args)
    node = MedorTFBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
