import time
import math
import smbus
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Int32MultiArray

class PCA9685:
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, address=0x40, debug=False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def setPWMFreq(self, freq):
        prescaleval = 25000000.0 / 4096.0 / float(freq) - 1.0
        prescale = math.floor(prescaleval + 0.5)
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, int(prescale))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    def setServoPulse(self, channel, pulse):
        pulse = pulse * 4096 / 20000
        self.setPWM(channel, 0, int(pulse))


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.pwm = PCA9685()
        self.pwm.setPWMFreq(50)
        self.subscription = self.create_subscription(Int32MultiArray, '/control_servo_node', self.servo_callback, 10)


    def servo_callback(self, msg):
        # Assuming the message contains the values in order of pin and pulse pairs
        pin = None
        pulse = None
        for i, value in enumerate(msg.data):
            if i % 2 == 0:  # Even index is for pin
                pulse = value
            else:  # Odd index is for pulse
                pin = value
                if pin is not None:
                    # Set the servo pulse
                    self.pwm.setServoPulse(pin, pulse)
                    self.get_logger().info(f"Set servo {pin} to pulse {pulse}")



def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    rclpy.spin(servo_controller)
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
