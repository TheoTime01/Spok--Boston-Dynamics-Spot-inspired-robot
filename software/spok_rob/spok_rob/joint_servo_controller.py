import time
import math
import smbus
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

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
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_group_effort_controller/joint_trajectory',
            self.joint_trajectory_callback,
            10
        )
        self.joint_map = {
            'front_left_shoulder': 13,
            'front_right_shoulder': 12,
            'rear_left_shoulder': 2,
            'rear_right_shoulder': 3,
            'front_right_leg': 15,
            'front_left_leg': 14,
            'rear_right_leg': 0,
            'rear_left_leg': 1,
            'front_right_foot': 11,
            'front_left_foot': 10,
            'rear_right_foot': 4,
            'rear_left_foot': 5
        }

        self.start_map = {
            'front_left_shoulder': 1510,
            'front_right_shoulder': 1570,
            'rear_left_shoulder': 1630,
            'rear_right_shoulder': 1570,
            'front_right_leg': 620,
            'front_left_leg': 2490,
            'rear_right_leg': 710,
            'rear_left_leg': 2290,
            'front_right_foot': 2730,
            'front_left_foot': 410,
            'rear_right_foot': 2540,
            'rear_left_foot': 460
        }

        self.factor_map = {
            'front_left_shoulder': 1510,
            'front_right_shoulder': 1570,
            'rear_left_shoulder': 1630,
            'rear_right_shoulder': 1570,
            'front_right_leg': 620/-1.235984206199646,
            'front_left_leg': 2490 / -1.235984206199646,
            'rear_right_leg': 710 / -1.235984206199646,
            'rear_left_leg': 2290 / -1.235984206199646,
            'front_right_foot': 2730 / 2.512333393096924,
            'front_left_foot': 410 / 2.512333393096924,
            'rear_right_foot': 2540 / 2.512333393096924,
            'rear_left_foot': 460 / 2.512333393096924
        }

    def joint_trajectory_callback(self, msg):
        for i, joint_name in enumerate(msg.joint_names):
            if joint_name in self.joint_map:
                position_radians = msg.points[0].positions[i]
                factor = self.factor_map[joint_name]
                if 'shoulder' in joint_name:
                    pulse = factor + position_radians * factor
                else:
                    pulse = position_radians * factor
                pulse_rounded = round(pulse)
                self.pwm.setServoPulse(self.joint_map[joint_name], pulse_rounded)


    def start(self, msg):
        self.get_logger().info(f"Start !")
        for i, joint_name in enumerate(msg.joint_names):
            if joint_name in self.start_map:
                pulse = self.start_map[joint_name]
                self.pwm.setServoPulse(self.joint_map[joint_name], pulse)
        self.get_logger().info(f"Start end...")

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    servo_controller.start()
    rclpy.spin(servo_controller)
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
