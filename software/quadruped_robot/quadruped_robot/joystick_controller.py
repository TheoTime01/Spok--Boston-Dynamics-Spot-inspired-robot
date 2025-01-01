#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from tf_transformations import quaternion_from_euler

# Custom message type for PoseLite (assuming this is a custom message)
from champ_msgs.msg import Pose as PoseLite

class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__('joy_teleop_node')
        
        # Publishers
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pose_lite_publisher = self.create_publisher(PoseLite, 'body_pose/raw', 1)
        self.pose_publisher = self.create_publisher(Pose, 'body_pose', 1)
        
        # Subscriber
        self.joy_subscriber = self.create_subscription(
            Joy, 
            'joy', 
            self.joy_callback, 
            1
        )
        
        # Declare and get parameters with default values
        self.declare_parameter("gait/swing_height", 0.0)
        self.declare_parameter("gait/nominal_height", 0.0)
        self.declare_parameter("speed", 0.5)
        self.declare_parameter("turn", 1.0)
        
        self.swing_height = self.get_parameter("gait/swing_height").value
        self.nominal_height = self.get_parameter("gait/nominal_height").value
        self.speed = self.get_parameter("speed").value
        self.turn = self.get_parameter("turn").value
        
        # Optional: Log parameter values
        self.get_logger().info(f"Initialized with parameters: "
                                f"swing_height={self.swing_height}, "
                                f"nominal_height={self.nominal_height}, "
                                f"speed={self.speed}, "
                                f"turn={self.turn}")
    
    def joy_callback(self, data):
        """
        Callback function to process joystick input and publish messages
        
        Joystick Mapping:
        - Axes[1]: Left stick vertical (forward/backward)
        - Axes[0]: Left stick horizontal (strafe)
        - Axes[3]: Right stick horizontal (turn/yaw)
        - Axes[4]: Right stick vertical (pitch)
        - Buttons[4]: Left shoulder button (modify movement)
        - Buttons[5]: Right shoulder button (modify rotation)
        - Axes[5]: Trigger for vertical movement
        """
        # Create Twist message for velocity
        twist = Twist()
        twist.linear.x = data.axes[1] * self.speed
        twist.linear.y = data.buttons[4] * data.axes[0] * self.speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = (not data.buttons[4]) * data.axes[0] * self.turn
        self.velocity_publisher.publish(twist)
        
        # Create PoseLite message
        body_pose_lite = PoseLite()
        body_pose_lite.x = 0.0
        body_pose_lite.y = 0.0
        body_pose_lite.roll = (not data.buttons[5]) * -data.axes[3] * 0.349066  # ~20 degrees max
        body_pose_lite.pitch = data.axes[4] * 0.174533  # ~10 degrees max
        body_pose_lite.yaw = data.buttons[5] * data.axes[3] * 0.436332  # ~25 degrees max
        
        # Vertical movement from trigger
        if data.axes[5] < 0:
            body_pose_lite.z = data.axes[5] * 0.5
        
        self.pose_lite_publisher.publish(body_pose_lite)
        
        # Create full Pose message
        body_pose = Pose()
        body_pose.position.z = body_pose_lite.z
        
        # Convert Euler angles to quaternion
        quaternion = quaternion_from_euler(
            body_pose_lite.roll, 
            body_pose_lite.pitch, 
            body_pose_lite.yaw
        )
        
        # Set quaternion orientation
        body_pose.orientation.x = quaternion[0]
        body_pose.orientation.y = quaternion[1]
        body_pose.orientation.z = quaternion[2]
        body_pose.orientation.w = quaternion[3]
        
        self.pose_publisher.publish(body_pose)
        
        # Optional: Log published messages
        # self.get_logger().info("Published velocity and pose messages")

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
