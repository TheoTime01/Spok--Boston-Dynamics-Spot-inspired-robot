#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from tf_transformations import quaternion_from_euler
from champ_msgs.msg import Pose as PoseLite
from std_msgs.msg import Float32
import time

class ScenarioNode(Node):
    def __init__(self):
        super().__init__('scenario_node')

        # Publishers
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pose_lite_publisher = self.create_publisher(PoseLite, 'body_pose/raw', 1)
        self.pose_publisher = self.create_publisher(Pose, 'body_pose', 1)

        # Subscribers
        self.sonar_subscriber = self.create_subscription(
            Float32, 
            'pico_publisher', 
            self.sonar_callback, 
            1
        )
        self.sonar_subscriber_bis = self.create_subscription(
            Float32, 
            'pico_publisher_bis', 
            self.sonar_callback, 
            1
        )

        # Declare and get parameters with default values
        self.declare_parameter("speed", 0.2)
        self.declare_parameter("turn_speed", 0.4)
        self.speed = self.get_parameter("speed").value
        self.turn_speed = self.get_parameter("turn_speed").value

        # State variables
        self.obstacle_detected = False
        self.resume_scenario = False

    def sonar_callback(self, msg):
        """Callback for sonar sensors."""
        distance = msg.data
        if distance <= 0.15:
            if not self.obstacle_detected:
                self.get_logger().info('Obstacle detected! Stopping the robot.')
                self.stop()
                self.obstacle_detected = True
        else:
            if self.obstacle_detected:
                self.get_logger().info('Obstacle cleared! Resuming the scenario.')
                self.obstacle_detected = False
                self.resume_scenario = True

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
        time.sleep(3)

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
    
    def lay_down_mov(self):
        self.lay_down()
        self.lay_up()
        self.wait_for_obstacle_clearance(3)


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
        self.wait_for_obstacle_clearance(5)

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
        self.wait_for_obstacle_clearance(3)

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
        self.wait_for_obstacle_clearance(6)

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

    def wait_for_obstacle_clearance(self, duration):
        """Wait for the specified duration, pausing for obstacles."""
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self)
            if self.obstacle_detected:
                self.get_logger().info('Paused due to obstacle. Waiting...')
                while self.obstacle_detected:
                    rclpy.spin_once(self)
                self.get_logger().info('Resuming after obstacle clearance.')

    def execute_scenario(self):
        """Execute the predefined scenario."""
        self.lay_down_mov()
        self.forward()
        self.stop()
        self.turn_left()
        self.stop()
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
