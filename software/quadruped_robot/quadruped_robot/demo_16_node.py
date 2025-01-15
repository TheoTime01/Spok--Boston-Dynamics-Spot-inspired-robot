import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')

        self.subscription_STT = self.create_subscription(Image, 'speech', self.STT_callback, 10)

        self.subscription_gyro = self.create_subscription(Image, 'kalman_angles', self.gyro_callback, 10)

        self.gyro_x = 0
        self.gyro_y = 0

        self.state = 0 #0: couché, 1: assis, 2: debout



        #self.subscription = self.create_subscription(Empty,'land',self.land_callback,10)


    def STT_callback(self, msg):
        print(self.msg.data)
        if self.msg.data == "stand up":
            self.state = 2

        if self.msg.data == "sit down":
            self.state = 1
            
        if self.msg.data == "lay down":
            self.state = 0



    def gyro_callback(self, msg):
        self.gyro_x = msg.data[0]
        self.gyro_y = msg.data[1]

        if self.state == 2:
            pass



def main(args=None):
    rclpy.init(args=args)
    demo_node = DemoNode()
    rclpy.spin(demo_node)
    demo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
