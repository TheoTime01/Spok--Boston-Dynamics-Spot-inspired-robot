import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
import os


class ConnectionNode(Node):
    def __init__(self):
        super().__init__('connection_node')

        self.connection_state = True

        self.hostname = "192.168.1.164"

        self.publisher_ = self.create_publisher(Bool, 'connection_state', 10)
        self.publisher_pico = self.create_publisher(Int32, 'connection_pico', 10)

        self.timer_ = self.create_timer(0.5, self.publish_connection)

    def publish_connection(self):

        response = os.system("ping -c 1 -q " + self.hostname)

        if response == 0:
            self.connection_state = True
        elif((response == 0) and (self.connection_state == True)):
            self.connection_state = False
            msg_pico = Int32()
            msg_pico.data = 1
            self.publisher_pico.publish(msg_pico)
        else:
            self.connection_state = False

        msg = Bool()
        msg.data = self.connection_state
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    connection_node = ConnectionNode()
    rclpy.spin(connection_node)
    connection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()