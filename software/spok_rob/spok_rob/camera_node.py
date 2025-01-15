import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(-1)
        self.bridge = CvBridge()
        

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convertir l'image OpenCV (numpy) en message ROS2
            frame = cv2.resize(frame, (0,0), fx = 0.5, fy = 0.5)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning("Échec de capture d'image depuis la webcam.")

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()  # Libérer la webcam


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
