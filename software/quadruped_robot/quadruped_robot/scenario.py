#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from tf_transformations import quaternion_from_euler
from champ_msgs.msg import Pose as PoseLite
import time

class ScenarioNode(Node):
    def __init__(self):
        super().__init__('scenario_node')

        # Publishers
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pose_lite_publisher = self.create_publisher(PoseLite, 'body_pose/raw', 1)
        self.pose_publisher = self.create_publisher(Pose, 'body_pose', 1)

        # Declare and get parameters with default values
        self.declare_parameter("speed", 0.2)
        self.declare_parameter("turn_speed", 0.4)
        self.speed = self.get_parameter("speed").value
        self.turn_speed = self.get_parameter("turn_speed").value

    def lay_down(self):
        """Make the robot lay down."""
        # Create PoseLite message
        body_pose_lite = PoseLite()
        body_pose_lite.x = 0.0
        body_pose_lite.y = 0.0
        body_pose_lite.roll = 0.0  # No roll
        body_pose_lite.pitch = 0.0  # No pitch
        body_pose_lite.yaw = 0.0  # No yaw
        body_pose_lite.z = -0.5  # Lower the body for laying down

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

        self.get_logger().info('Laying down')
        time.sleep(6)

    def lay_up(self):
        """Make the robot lay up."""
        # Create PoseLite message
        body_pose_lite = PoseLite()
        body_pose_lite.x = 0.0
        body_pose_lite.y = 0.0
        body_pose_lite.roll = 0.0  # No roll
        body_pose_lite.pitch = 0.0  # No pitch
        body_pose_lite.yaw = 0.0  # No yaw
        body_pose_lite.z = 0.0  # Lower the body for laying down

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

        self.get_logger().info('Laying up')
        time.sleep(3)


    def forward(self):
        """Move the robot forward."""
        twist = Twist()
        twist.linear.x = -self.speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)
        self.get_logger().info('Moving forward')
        time.sleep(5)

    def backward(self):
        """Move the robot backward."""
        twist = Twist()
        twist.linear.x = self.speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)
        self.get_logger().info('Moving backward')
        time.sleep(3)

    def turn_left(self):
        """Turn the robot to the left."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.turn_speed
        self.velocity_publisher.publish(twist)
        self.get_logger().info('Turning left')
        time.sleep(6)

    def stop(self):
        """Stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)
        self.get_logger().info('Stopping')
        time.sleep(1)

    def execute_scenario(self):
        """Execute the predefined scenario."""
        self.lay_down()
        self.lay_up()
        self.forward()
        self.stop()
        self.turn_left()
        self.backward()
        self.stop()
        self.lay_down()

def main(args=None):
    rclpy.init(args=args)
    node = ScenarioNode()

    try:
        node.get_logger().info('Starting scenario...')
        node.execute_scenario()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
