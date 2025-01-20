import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import math

# MPU6050 Registers and Addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # 10 Hz

        self.bus = SMBus(1)  # Use I2C bus 1
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # Wake up MPU6050

        self.get_logger().info("MPU6050 node started")

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(MPU6050_ADDR, addr)
        low = self.bus.read_byte_data(MPU6050_ADDR, addr + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

    def publish_imu_data(self):
        # Read accelerometer data
        accel_x = self.read_raw_data(ACCEL_XOUT_H)
        accel_y = self.read_raw_data(ACCEL_XOUT_H + 2)
        accel_z = self.read_raw_data(ACCEL_XOUT_H + 4)

        # Read gyroscope data
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = self.read_raw_data(GYRO_XOUT_H + 4)

        # Convert to proper units
        accel_scale = 16384.0  # 2g range
        gyro_scale = 131.0     # 250 deg/s range

        accel_x = accel_x / accel_scale
        accel_y = accel_y / accel_scale
        accel_z = accel_z / accel_scale

        gyro_x = gyro_x / gyro_scale
        gyro_y = gyro_y / gyro_scale
        gyro_z = gyro_z / gyro_scale

        # Populate the IMU message
        imu_msg = Imu()

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Linear acceleration
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        # Angular velocity
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        # Publish the message
        self.publisher_.publish(imu_msg)

        self.get_logger().info(f"Published IMU data: Accel=({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}), Gyro=({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down MPU6050 node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
