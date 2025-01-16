import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


from cv_bridge import CvBridge
import cv2



class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')

        self.subscription_STT = self.create_subscription(String, 'speech', self.STT_callback, 10)

        self.subscription_gyro = self.create_subscription(Float32MultiArray, 'kalman_angles', self.gyro_callback, 10)

        self.gyro_x = 0
        self.gyro_y = 0

        self.state = 0 #0: couché, 1: assis, 2: debout

        self.shoulderFR_value = 1.5893254712295857e-08
        self.shoulderFL_value = 1.5893254712295857e-08
        self.shoulderRR_value = 1.5893254712295857e-08
        self.shoulderRL_value = 1.5893254712295857e-08
        self.armFR_value = -1.235984206199646
        self.armFL_value = -1.235984206199646
        self.armRR_value = -1.235984206199646
        self.armRL_value = -1.235984206199646
        self.footFR_value = 2.512333393096924
        self.footFL_value = 2.512333393096924
        self.footRR_value = 2.512333393096924
        self.footRL_value = 2.512333393096924

        self.msg_servo = JointTrajectory()
        self.msg_servo.joint_names = ['front_left_shoulder', 'front_right_shoulder', 'rear_left_shoulder', 'rear_right_shoulder', 'front_right_leg', 'front_left_leg', 'rear_right_leg', 'rear_left_leg', 'front_right_foot', 'front_left_foot', 'rear_right_foot', 'rear_left_foot']

        self.publisher = self.create_publisher(JointTrajectory, '/joint_group_effort_controller/joint_trajectory', 10)

        #init assis
        self.point = JointTrajectoryPoint()
        self.point.positions = [1.5893254712295857e-08, 1.5893254712295857e-08, 1.5893254712295857e-08, 1.5893254712295857e-08, -1.235984206199646, -1.235984206199646, -1.235984206199646, -1.235984206199646, 2.512333393096924, 2.512333393096924, 2.512333393096924, 2.512333393096924]
        self.msg_servo.points.append(self.point)
        self.publisher.publish(self.msg_servo)


    def STT_callback(self, msg):
        print(msg.data)

        if msg.data == "stand up":
            self.state = 2
            self.point = JointTrajectoryPoint()
            self.point.positions = [1.5893254712295857e-08, 1.5893254712295857e-08, 1.5893254712295857e-08, 1.5893254712295857e-08, -0.9355980157852173, -0.9355980157852173, -0.9355980157852173, -0.9355980157852173, 1.8437325954437256, 1.8437325954437256, 1.8437325954437256, 1.8437325954437256]
            self.msg_servo.points.append(self.point)
            self.publisher.publish(self.msg_servo)

        if msg.data == "sit down":
            self.state = 1
            self.point = JointTrajectoryPoint()
            self.point.positions = [1.5893254712295857e-08, 1.5893254712295857e-08, 1.5893254712295857e-08, 1.5893254712295857e-08, -1.235984206199646, -1.235984206199646, -1.235984206199646, -1.235984206199646, 2.512333393096924, 2.512333393096924, 2.512333393096924, 2.512333393096924]
            self.msg_servo.points.append(self.point)
            self.publisher.publish(self.msg_servo)

        if msg.data == "lay down":
            self.state = 0
            self.point = JointTrajectoryPoint()
            self.point.positions = [1.5893254712295857e-08, 1.5893254712295857e-08, 1.5893254712295857e-08, 1.5893254712295857e-08, -1.235984206199646, -1.235984206199646, -1.235984206199646, -1.235984206199646, 2.512333393096924, 2.512333393096924, 2.512333393096924, 2.512333393096924]
            self.msg_servo.points.append(self.point)
            self.publisher.publish(self.msg_servo)




    def gyro_callback(self, msg):
        self.gyro_x = msg.data[0]
        self.gyro_y = msg.data[1]

        if self.state == 2:

            if self.gyro_x < -3:
                self.footRL_value += 0.001
                self.footFL_value += 0.001
                self.footRR_value += 0.001
                self.footFR_value += 0.001
            if self.gyro_x > 3:
                self.footRL_value -= 0.001
                self.footFL_value -= 0.001
                self.footRR_value -= 0.001
                self.footFR_value -= 0.001

            if self.gyro_y < -3:
                self.footRL_value += 0.001
                self.footFL_value -= 0.001
                self.footRR_value -= 0.001
                self.footFR_value += 0.001
            if self.gyro_y > 3:
                self.footRL_value -= 0.001
                self.footFL_value += 0.001
                self.footRR_value += 0.001
                self.footFR_value -= 0.001

            self.point = JointTrajectoryPoint()


            self.point.positions = [1.5893254712295857e-08, 1.5893254712295857e-08, 1.5893254712295857e-08, 1.5893254712295857e-08, -0.9355980157852173, -0.9355980157852173, -0.9355980157852173, -0.9355980157852173, self.footFR_value, self.footFL_value, self.footRR_value, self.footRL_value]
            self.msg_servo.points.append(self.point)
            self.publisher.publish(self.msg_servo)



def main(args=None):
    rclpy.init(args=args)
    demo_node = DemoNode()
    rclpy.spin(demo_node)
    demo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
