#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from champ_msgs.msg import Pose as PoseLite
from tf_transformations import quaternion_from_euler
import math


class AutonomousMovement(Node):
    def __init__(self):
        super().__init__('autonomous_movement')

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.body_pose_publisher = self.create_publisher(Pose, '/body_pose', 10)
        self.body_pose_lite_publisher = self.create_publisher(PoseLite, '/body_pose/raw', 10)

        # Robot state
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = 0.0
        self.target = {'x': 10.0, 'y': 10.0}  # Example target
        self.linear_speed = 0.2
        self.angular_speed = 0.5

    def odom_callback(self, msg):
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y

    def imu_callback(self, msg):
        # Extract yaw from the IMU quaternion
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.orientation = math.atan2(siny_cosp, cosy_cosp)

    def compute_control(self):
        # Compute control signals
        dx = self.target['x'] - self.position['x']
        dy = self.target['y'] - self.position['y']

        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.orientation

        # Normalize angle
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        twist = Twist()

        if distance > 0.1:
            twist.linear.x = -self.linear_speed
            twist.angular.z = self.angular_speed * angle_diff
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Target reached!')

        return twist

    def publish_body_pose(self):
        # Create and publish PoseLite message
        body_pose_lite = PoseLite()
        body_pose_lite.x = self.position['x']
        body_pose_lite.y = self.position['y']
        body_pose_lite.roll = 0.0
        body_pose_lite.pitch = 0.0
        body_pose_lite.yaw = self.orientation
        self.body_pose_lite_publisher.publish(body_pose_lite)

        # Create and publish full Pose message
        body_pose = Pose()
        body_pose.position.x = self.position['x']
        body_pose.position.y = self.position['y']
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
        self.publish_body_pose()


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
