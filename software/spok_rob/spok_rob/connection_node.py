import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
import subprocess




def is_device_connected(ip_address: str, timeout: int = 1) -> bool:
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", str(timeout), ip_address],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        return result.returncode == 0
    except Exception as e:
        print(f"Error pinging {ip_address}: {e}")
        return False


class ConnectionNode(Node):
    def __init__(self):
        super().__init__('connection_node')

        self.connection_state = True
        self.connection_check = 0

        self.hostname = "192.168.1.221"

        self.publisher_ = self.create_publisher(Bool, 'connection_state', 10)
        self.publisher_pico = self.create_publisher(Int32, 'pico_subscriber', 10)

        self.timer_ = self.create_timer(0.5, self.publish_connection)


    def publish_connection(self):

        if (is_device_connected(self.hostname)) and (self.connection_state == False):
            print("CONNECTED")
            self.connection_state = True
            msg_pico = Int32()
            msg_pico.data = 1
            self.publisher_pico.publish(msg_pico)
        elif (not is_device_connected(self.hostname)) and (self.connection_state == True):
            print("NOT CONNECTED")
            self.connection_state = False
            msg_pico = Int32()
            msg_pico.data = 1
            self.publisher_pico.publish(msg_pico)
        elif (is_device_connected(self.hostname)) and (self.connection_state == True):
            print("CONNECTED")
            self.connection_state = True
        elif (not is_device_connected(self.hostname)) and (self.connection_state == False):
            print("NOT CONNECTED")
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