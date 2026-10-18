#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from bumperbot_firmware.mpu6050_math import (
    ACCEL_CONFIG_VALUE, ACCEL_VARIANCE, GYRO_CONFIG_VALUE, GYRO_VARIANCE,
    accel_to_m_s2, covariance_diagonal, gyro_to_rad_s, to_signed,
)

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68


class MPU6050_Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")
        
        # I2C Interafce
        self.is_connected_ = False
        self.init_i2c()

        # ROS 2 Interface
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"
        self.imu_msg_.orientation_covariance[0] = -1.0
        self.imu_msg_.angular_velocity_covariance = covariance_diagonal(GYRO_VARIANCE)
        self.imu_msg_.linear_acceleration_covariance = covariance_diagonal(ACCEL_VARIANCE)
        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()
            
            # Read Accelerometer raw value
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            
            # Read Gyroscope raw value
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)
            
            self.imu_msg_.linear_acceleration.x = accel_to_m_s2(acc_x)
            self.imu_msg_.linear_acceleration.y = accel_to_m_s2(acc_y)
            self.imu_msg_.linear_acceleration.z = accel_to_m_s2(acc_z)
            self.imu_msg_.angular_velocity.x = gyro_to_rad_s(gyro_x)
            self.imu_msg_.angular_velocity.y = gyro_to_rad_s(gyro_y)
            self.imu_msg_.angular_velocity.z = gyro_to_rad_s(gyro_z)

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
        except OSError:
            self.is_connected_ = False

    def init_i2c(self):
        try:
            import smbus
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, GYRO_CONFIG_VALUE)
            self.bus_.write_byte_data(DEVICE_ADDRESS, ACCEL_CONFIG, ACCEL_CONFIG_VALUE)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
        except (OSError, ImportError) as exc:
            if self.is_connected_ or not getattr(self, "warned_", False):
                self.get_logger().warn(f"No se pudo iniciar la IMU por I2C: {exc}")
                self.warned_ = True
            self.is_connected_ = False
        
    def read_raw_data(self, addr):
        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
        low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr+1)
        
        value = to_signed(high, low)
        return value


def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Driver()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()