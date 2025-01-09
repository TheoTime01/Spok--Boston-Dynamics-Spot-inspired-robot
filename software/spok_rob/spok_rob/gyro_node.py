import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
#from Kalman import KalmanAngle
import smbus
import time
import math



class KalmanAngle:
    def __init__(self):
        self.QAngle = 0.001
        self.QBias = 0.003
        self.RMeasure = 0.03
        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0
        self.P=[[0.0,0.0],[0.0,0.0]]

    '''def kalman():
        QAngle = 0.001
        QBias = 0.003
        RMeasure = 0.03

        angle = 0.0
        bias = 0.0

        P[0][0] = 0.0
        P[0][1] = 0.0
        P[1][0] = 0.0
        P[1][1] = 0.0'''

    def getAngle(self,newAngle, newRate,dt):
        #step 1:
        self.rate = newRate - self.bias;    #new_rate is the latest Gyro measurement
        self.angle += dt * self.rate

        #Step 2:
        self.P[0][0] += dt * (dt*self.P[1][1] -self.P[0][1] - self.P[1][0] + self.QAngle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.QBias * dt

        #Step 3: Innovation
        y = newAngle - self.angle

        #Step 4: Innovation covariance
        s = self.P[0][0] + self.RMeasure

        #Step 5:    Kalman Gain
        K=[0.0,0.0]
        K[0] = self.P[0][0]/s
        K[1] = self.P[1][0]/s

        #Step 6: Update the Angle
        self.angle += K[0] * y
        self.bias  += K[1] * y

        #Step 7: Calculate estimation error covariance - Update the error covariance
        P00Temp = self.P[0][0]
        P01Temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00Temp;
        self.P[0][1] -= K[0] * P01Temp;
        self.P[1][0] -= K[1] * P00Temp;
        self.P[1][1] -= K[1] * P01Temp;

        return self.angle

    def setAngle(self,angle):
        self.angle = angle

    def setQAngle(self,QAngle):
        self.QAngle = QAngle

    def setQBias(self,QBias):
        self.QBias = QBias

    def setRMeasure(self,RMeasure):
        self.RMeasure = RMeasure

    def getRate():
        return self.rate

    def getQAngle():
        return self.QAngle

    def getQBias():
        return self.QBias

    def  getRMeasure():
        return self.RMeasure
    


class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.publisher_ = self.create_publisher(Float32MultiArray, 'kalman_angles', 10)
        self.timer_ = self.create_timer(0.005, self.publish_angles)

        self.kalmanX = KalmanAngle()
        self.kalmanY = KalmanAngle()
        self.kalmanZ = KalmanAngle()

        self.RestrictPitch = False
        self.radToDeg = 57.2957786
        self.kalAngleX = 0
        self.kalAngleY = 0
        self.kalAngleZ = 0

        self.bus = smbus.SMBus(1)
        self.DeviceAddress = 0x68

        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.INT_ENABLE = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47

        self.MPU_Init()
        time.sleep(1)

        self.timer = time.time()

    def MPU_Init(self):
        self.bus.write_byte_data(self.DeviceAddress, self.SMPLRT_DIV, 7)
        self.bus.write_byte_data(self.DeviceAddress, self.PWR_MGMT_1, 1)
        self.bus.write_byte_data(self.DeviceAddress, self.CONFIG, int('0000110', 2))
        self.bus.write_byte_data(self.DeviceAddress, self.GYRO_CONFIG, 24)
        self.bus.write_byte_data(self.DeviceAddress, self.INT_ENABLE, 1)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.DeviceAddress, addr)
        low = self.bus.read_byte_data(self.DeviceAddress, addr + 1)
        value = ((high << 8) | low)
        if value > 32768:
            value -= 65536
        return value

    def publish_angles(self):
        try:
            accX = self.read_raw_data(self.ACCEL_XOUT_H)
            accY = self.read_raw_data(self.ACCEL_YOUT_H)
            accZ = self.read_raw_data(self.ACCEL_ZOUT_H)

            gyroX = self.read_raw_data(self.GYRO_XOUT_H)
            gyroY = self.read_raw_data(self.GYRO_YOUT_H)
            gyroZ = self.read_raw_data(self.GYRO_ZOUT_H)

            dt = time.time() - self.timer
            self.timer = time.time()

            if self.RestrictPitch:
                roll = math.atan2(accY, accZ) * self.radToDeg
                pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * self.radToDeg
            else:
                roll = math.atan2(accY, accZ) * self.radToDeg
                pitch = math.atan2(-accX, accZ) * self.radToDeg

            gyroXRate = gyroX / 131
            gyroYRate = gyroY / 131
            gyroZRate = gyroZ / 131

            self.kalAngleX = self.kalmanX.getAngle(roll, gyroXRate, dt)
            self.kalAngleY = self.kalmanY.getAngle(pitch, gyroYRate, dt)
            self.kalAngleZ = self.kalmanZ.getAngle(0, gyroYRate, dt)

            msg = Float32MultiArray()
            msg.data = [self.kalAngleX, self.kalAngleY, self.kalAngleZ]
            self.publisher_.publish(msg)

            self.get_logger().info(f'Published Angles: X={self.kalAngleX:.2f}, Y={self.kalAngleY:.2f}, Z={self.kalAngleZ:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error reading sensor data: {e}')


def main(args=None):
    rclpy.init(args=args)
    mpu6050_node = MPU6050Node()
    rclpy.spin(mpu6050_node)
    mpu6050_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()