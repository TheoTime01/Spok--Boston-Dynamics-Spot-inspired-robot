import rclpy
from rclpy.node import Node
import smbus
import time
import math
from std_msgs.msg import Float32MultiArray


import json



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

        self.shoulderFR = 12
        self.shoulderFL = 13
        self.shoulderRR = 3
        self.shoulderRL = 2
        self.armFR = 15
        self.armFL = 14
        self.armRR = 0
        self.armRL = 1
        self.footFR = 11
        self.footFL = 10
        self.footRR = 4
        self.footRL = 5

        self.shoulderFR_value = 1500
        self.shoulderFL_value = 1540
        self.shoulderRR_value = 1680
        self.shoulderRL_value = 1550
        self.armFR_value = 850
        self.armFL_value = 2180
        self.armRR_value = 910
        self.armRL_value = 2140
        self.footFR_value = 2380
        self.footFL_value = 840
        self.footRR_value = 2300
        self.footRL_value = 840


        self.setServoPulse(self.shoulderFL, self.shoulderFL_value)
        self.setServoPulse(self.shoulderFR, self.shoulderFR_value)
        self.setServoPulse(self.shoulderRL, self.shoulderRL_value)
        self.setServoPulse(self.shoulderRR, self.shoulderRR_value)
        self.setServoPulse(self.armFR, self.armFR_value)
        self.setServoPulse(self.armFL, self.armFL_value)
        self.setServoPulse(self.armRR, self.armRR_value)
        self.setServoPulse(self.armRL, self.armRL_value)
        self.setServoPulse(self.footFR, self.footFR_value)
        self.setServoPulse(self.footFL, self.footFL_value)
        self.setServoPulse(self.footRR, self.footRR_value)
        self.setServoPulse(self.footRL, self.footRL_value)
        

        self.subscription_gyro = self.create_subscription(Float32MultiArray, 'kalman_angles', self.adapt_callback, 10)
        
        self.pwm = PCA9685()
        self.pwm.setPWMFreq(50)

        self.balance()

    def adapt_callback(self, msg):
        self.gyro_x = msg.data[0]
        self.gyro_y = msg.data[1]
        print(self.gyro_x)


    def sequence_forward(self):
        with open('../json/forward.json', 'r') as file:
            data_forward = json.load(file)

        for s in data_forward["sequence"]:
            t = s["time"]
            print(t)

    def balance(self):
        while (self.gyro_x < -3) or (self.gyro_x > 3) or (self.gyro_y < -3) or (self.gyro_y > 3):
            if self.gyro_x < -3:
                self.footRL_value += 3
                self.footFL_value += 3
                self.footRR_value += 3
                self.footFR_value += 3
            if self.gyro_x > 3:
                self.footRL_value -= 3
                self.footFL_value -= 3
                self.footRR_value -= 3
                self.footFR_value -= 3

            if self.gyro_y < -3:
                self.footRL_value += 3
                self.footFL_value -= 3
                self.footRR_value -= 3
                self.footFR_value += 3
            if self.gyro_y > 3:
                self.footRL_value -= 3
                self.footFL_value += 3
                self.footRR_value += 3
                self.footFR_value -= 3
        print("Balanced !")
        

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    rclpy.spin(servo_controller)
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
