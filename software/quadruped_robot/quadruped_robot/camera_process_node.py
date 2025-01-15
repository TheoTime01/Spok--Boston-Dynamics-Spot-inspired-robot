import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



class CameraProcess(Node):
    def __init__(self):
        super().__init__('camera_process')

        self.subscription = self.create_subscription(Image, 'video_frames', self.camera_callback, 10)


    def camera_callback(self, msg):
        print("frame received")



def main(args=None):
    rclpy.init(args=args)
    camera_process = CameraProcess()
    rclpy.spin(camera_process)
    camera_process.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
