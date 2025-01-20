import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os


class ConnectionNode(Node):
    def __init__(self):
        super().__init__('connection_node')

        self.connction_state = True

        self.hostname = "pc-arnaud.home"

        self.publisher_ = self.create_publisher(Bool, 'connection_state', 10)
        self.timer_ = self.create_timer(0.5, self.publish_connection)

    def publish_connection(self):

        response = os.system("ping -c 1 -q " + self.hostname)

        if response == 0:
            self.connction_state = True
        else:
            self.connction_state = False

        msg = Bool()
        msg.data = self.connction_state
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    connection_node = ConnectionNode()
    rclpy.spin(connection_node)
    connection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()