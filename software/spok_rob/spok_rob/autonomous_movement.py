#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from champ_msgs.msg import Pose as PoseLite
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math


class AutonomousMovement(Node):
    def __init__(self):
        super().__init__('autonomous_movement')

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.body_pose_publisher = self.create_publisher(Pose, '/body_pose', 10)
        self.body_pose_lite_publisher = self.create_publisher(PoseLite, '/body_pose/raw', 10)

        # Robot state
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = 0.0
        self.target = {'x': 2.0, 'y': 2.0}  # Example target
        self.linear_speed = 0.2
        self.angular_speed = 0.5

    def odom_callback(self, msg):
        # Update position
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y

        # Update orientation (yaw) from odometry quaternion
        q = msg.pose.pose.orientation
        _, _, self.orientation = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def compute_control(self):
        # Compute control signals
        dx = self.target['x'] - self.position['x']
        dy = self.target['y'] - self.position['y']

        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.orientation

        # Normalize angle to [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        twist = Twist()

        if distance > 0.1:
            # Move towards the target
            twist.linear.x = self.linear_speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.angular_speed * angle_diff
            self.get_logger().info(f'distance={distance}/angle={angle_diff}')
        else:
            # Stop when target is reached
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Target reached!')

        return twist

    def publish_body_pose(self):
        # Publish minimal pose data to /body_pose/raw
        body_pose_lite = PoseLite()
        body_pose_lite.x = 0.0
        body_pose_lite.y = 0.0
        body_pose_lite.roll = 0.0
        body_pose_lite.pitch = 0.0
        body_pose_lite.yaw = self.orientation
        self.body_pose_lite_publisher.publish(body_pose_lite)

        # Publish full pose data to /body_pose
        body_pose = Pose()
        body_pose.position.z = 0.0

        quaternion = quaternion_from_euler(0.0, 0.0, self.orientation)
        body_pose.orientation.x = quaternion[0]
        body_pose.orientation.y = quaternion[1]
        body_pose.orientation.z = quaternion[2]
        body_pose.orientation.w = quaternion[3]
        self.body_pose_publisher.publish(body_pose)

    def move_to_target(self):
        twist = self.compute_control()
        self.cmd_vel_publisher.publish(twist)



def main(args=None):
    rclpy.init(args=args)

    node = AutonomousMovement()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.move_to_target()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
