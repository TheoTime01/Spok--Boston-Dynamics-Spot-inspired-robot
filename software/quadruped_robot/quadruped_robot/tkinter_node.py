import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tkinter as tk
from PIL import Image as PilImage, ImageTk


class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.frame = None

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def update_tkinter_window(subscriber, label):
    if subscriber.frame is not None:
        # Convert OpenCV image (BGR) to PIL format (RGB)
        cv_image = cv2.cvtColor(subscriber.frame, cv2.COLOR_BGR2RGB)
        pil_image = PilImage.fromarray(cv_image)
        tk_image = ImageTk.PhotoImage(image=pil_image)
        # Update the Tkinter label with the new frame
        label.config(image=tk_image)
        label.image = tk_image

    # Schedule the function to run again after 10ms
    label.after(10, update_tkinter_window, subscriber, label)


def ros_spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()

    # Start the ROS 2 spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin_thread, args=(video_subscriber,))
    ros_thread.start()

    # Start Tkinter GUI
    root = tk.Tk()
    root.title("ROS 2 Video Stream")

    # Create a label to display the video frames
    video_label = tk.Label(root)
    video_label.pack()

    # Schedule the video update function
    update_tkinter_window(video_subscriber, video_label)

    # Start the Tkinter main loop
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        video_subscriber.destroy_node()
        rclpy.shutdown()
        ros_thread.join()


if __name__ == '__main__':
    main()
