import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import speech_recognition as sr


class SpeechRecognition(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(String, 'speech', 10)

        recognizer = sr.Recognizer()

        msg = String()
        
        with sr.Microphone() as source:
            recognizer.adjust_for_ambient_noise(source, duration=1)

            try:
                while True:
                    audio = recognizer.listen(source)

                    try:
                        text = recognizer.recognize_google(audio)
                        if "stop" in text.lower():
                            msg.data = "stop"
                            self.publisher_.publish(msg)
                            print("stop detected")
                        if "stand up" in text.lower():
                            msg.data = "stand up"
                            self.publisher_.publish(msg)
                            print("stand up detected")
                        if "sit down" in text.lower():
                            msg.data = "sit down"
                            self.publisher_.publish(msg)
                            print("sit down detected")
                        if "lay down" in text.lower():
                            msg.data = "lay down"
                            self.publisher_.publish(msg)
                            print("lay down detected")

                    except sr.UnknownValueError:
                        print("could not understand")
                        pass
                    except sr.RequestError as e:
                        pass

            except KeyboardInterrupt:
                pass


    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
