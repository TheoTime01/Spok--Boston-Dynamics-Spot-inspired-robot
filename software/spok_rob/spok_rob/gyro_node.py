import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

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



kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

RestrictPitch = False
radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(DeviceAddress, SMPLRT_DIV, 7)

	#Write to power management register
	bus.write_byte_data(DeviceAddress, PWR_MGMT_1, 1)

	#Write to Configuration register
	#Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.) https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
	bus.write_byte_data(DeviceAddress, CONFIG, int('0000110',2))

	#Write to Gyro configuration register
	bus.write_byte_data(DeviceAddress, GYRO_CONFIG, 24)

	#Write to interrupt enable register
	bus.write_byte_data(DeviceAddress, INT_ENABLE, 1)


def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(DeviceAddress, addr)
        low = bus.read_byte_data(DeviceAddress, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value





bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
DeviceAddress = 0x68   # MPU6050 device address

MPU_Init()

time.sleep(1)
#Read Accelerometer raw value
accX = read_raw_data(ACCEL_XOUT_H)
accY = read_raw_data(ACCEL_YOUT_H)
accZ = read_raw_data(ACCEL_ZOUT_H)

#print(accX,accY,accZ)
#print(math.sqrt((accY**2)+(accZ**2)))
if (RestrictPitch):
    roll = math.atan2(accY,accZ) * radToDeg
    pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
else:
    #roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
    roll = math.atan2(accY,accZ) * radToDeg
    pitch = math.atan2(-accX,accZ) * radToDeg
print(roll)
kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
gyroXAngle = roll
gyroYAngle = pitch
compAngleX = roll
compAngleY = pitch

timer_gyro = time.time()



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('gyro_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'gyro', 10)
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):


        spok_roll = 0.0
        spok_pitch = 0.0

        #try:
        #Read Accelerometer raw value
        accX = read_raw_data(ACCEL_XOUT_H)
        accY = read_raw_data(ACCEL_YOUT_H)
        accZ = read_raw_data(ACCEL_ZOUT_H)

        #Read Gyroscope raw value
        gyroX = read_raw_data(GYRO_XOUT_H)
        gyroY = read_raw_data(GYRO_YOUT_H)
        gyroZ = read_raw_data(GYRO_ZOUT_H)

        dt = time.time() - timer_gyro
        timer_gyro = time.time()

        if (RestrictPitch):
            roll = math.atan2(accY,accZ) * radToDeg
            pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
        else:
            #roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
            roll = math.atan2(accY,accZ) * radToDeg
            pitch = math.atan2(-accX,accZ) * radToDeg

        gyroXRate = gyroX/131
        gyroYRate = gyroY/131

        if (RestrictPitch):

            if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
                kalmanX.setAngle(roll)
                complAngleX = roll
                kalAngleX   = roll
                gyroXAngle  = roll
            else:
                kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

            if(abs(kalAngleX)>90):
                gyroYRate  = -gyroYRate
                kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
        else:

            if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
                kalmanY.setAngle(pitch)
                complAngleY = pitch
                kalAngleY   = pitch
                gyroYAngle  = pitch
            else:
                kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)

            if(abs(kalAngleY)>90):
                gyroXRate  = -gyroXRate
                kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

            if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
                kalmanX.setAngle(roll)
                complAngleX = roll
                kalAngleX   = roll
                gyroXAngle  = roll
            else:
                kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

            if(abs(kalAngleX)>90):
                gyroYRate  = -gyroYRate
                kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)

        #angle = (rate of change of angle) * change in time
        gyroXAngle = gyroXRate * dt
        gyroYAngle = gyroYAngle * dt

        #compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
        compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
        compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

        if ((gyroXAngle < -180) or (gyroXAngle > 180)):
            gyroXAngle = kalAngleX
        if ((gyroYAngle < -180) or (gyroYAngle > 180)):
            gyroYAngle = kalAngleY

        print("Angle X: " + str(kalAngleX)+"   " +"Angle Y: " + str(kalAngleY))
        #print(str(roll)+"  "+str(gyroXAngle)+"  "+str(compAngleX)+"  "+str(kalAngleX)+"  "+str(pitch)+"  "+str(gyroYAngle)+"  "+str(compAngleY)+"  "+str(kalAngleY))
        
        spok_roll = kalAngleX
        spok_pitch = kalAngleY

        #except Exception as exc:
        #    pass


        msg = Float64MultiArray()
        msg.data = [spok_roll, spok_pitch]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
