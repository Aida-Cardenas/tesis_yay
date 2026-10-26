#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState

from bumperbot_firmware.mpu6050_math import (
    ACCEL_CONFIG_VALUE, ACCEL_VARIANCE, DLPF_CONFIG_VALUE, GYRO_CONFIG_VALUE, GYRO_VARIANCE,
    GyroBias, accel_to_m_s2, covariance_diagonal, gyro_to_rad_s, parse_burst,
)

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
DEVICE_ADDRESS = 0x68
WHEEL_STILL_RAD_S = 0.05
WHEEL_STILL_HOLD_S = 0.3
I2C_RETRY_S = 1.0


class MPU6050_Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")
        
        # I2C Interafce
        self.is_connected_ = False
        self.bus_ = None
        self.last_init_try_ = None
        self.init_i2c()

        # ROS 2 Interface
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"
        self.imu_msg_.orientation_covariance[0] = -1.0
        self.imu_msg_.angular_velocity_covariance = covariance_diagonal(GYRO_VARIANCE)
        self.imu_msg_.linear_acceleration_covariance = covariance_diagonal(ACCEL_VARIANCE)
        self.frequency_ = 0.01
        calibration_s = self.declare_parameter("gyro_calibration_time", 2.0).value
        self.bias_ = GyroBias(calibration_s / self.frequency_)
        self.reported_restarts_ = 0
        self.last_wheel_motion_ = None
        self.create_subscription(JointState, "/joint_states", self.jointCallback, 10)
        if not self.bias_.ready:
            self.get_logger().info(
                f"Midiendo el cero del giroscopio durante {calibration_s:.1f} s: no muevas el robot")
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def jointCallback(self, msg):
        if any(abs(v) > WHEEL_STILL_RAD_S for v in msg.velocity):
            self.last_wheel_motion_ = self.get_clock().now()

    def wheelsRecentlyMoved(self):
        if self.last_wheel_motion_ is None:
            return False
        return (self.get_clock().now() - self.last_wheel_motion_).nanoseconds * 1e-9 < WHEEL_STILL_HOLD_S

    def timerCallback(self):
        try:
            if not self.is_connected_:
                now = self.get_clock().now()
                if (self.last_init_try_ is not None
                        and (now - self.last_init_try_).nanoseconds * 1e-9 < I2C_RETRY_S):
                    return
                self.init_i2c()
                if not self.is_connected_:
                    return
            
            (acc_x, acc_y, acc_z), (gyro_x, gyro_y, gyro_z) = parse_burst(
                self.bus_.read_i2c_block_data(DEVICE_ADDRESS, ACCEL_XOUT_H, 14))

            self.imu_msg_.linear_acceleration.x = accel_to_m_s2(acc_x)
            self.imu_msg_.linear_acceleration.y = accel_to_m_s2(acc_y)
            self.imu_msg_.linear_acceleration.z = accel_to_m_s2(acc_z)
            gx, gy, gz = gyro_to_rad_s(gyro_x), gyro_to_rad_s(gyro_y), gyro_to_rad_s(gyro_z)
            if not self.bias_.ready:
                if self.wheelsRecentlyMoved():
                    self.bias_.restart()
                    done = False
                else:
                    done = self.bias_.add(gx, gy, gz)
                if self.bias_.restarts > self.reported_restarts_:
                    self.reported_restarts_ = self.bias_.restarts
                    self.get_logger().warn(
                        f"El robot se movió mientras se medía el giroscopio; empiezo otra vez "
                        f"({self.bias_.restarts} intentos)", throttle_duration_sec=5.0)
                if not done:
                    return
                bx, by, bz = self.bias_.bias
                self.get_logger().info(f"Cero del giroscopio: x={bx:.4f} y={by:.4f} z={bz:.4f} rad/s")
                if self.bias_.suspicious:
                    self.get_logger().warn(
                        "Ese cero es demasiado grande para un MPU6050 quieto: probablemente el robot "
                        "giraba al arrancar. Reinicia el driver con el robot quieto.")
            gx, gy, gz = self.bias_.correct(gx, gy, gz)
            self.imu_msg_.angular_velocity.x = gx
            self.imu_msg_.angular_velocity.y = gy
            self.imu_msg_.angular_velocity.z = gz

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
        except (OSError, ValueError) as exc:
            if self.is_connected_:
                self.get_logger().warn(f"Se perdió la IMU por I2C ({exc}); reintentando")
            self.is_connected_ = False

    def init_i2c(self):
        self.last_init_try_ = self.get_clock().now()
        try:
            import smbus
            if self.bus_ is not None:
                try:
                    self.bus_.close()
                except OSError:
                    pass
                self.bus_ = None
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, DLPF_CONFIG_VALUE)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, GYRO_CONFIG_VALUE)
            self.bus_.write_byte_data(DEVICE_ADDRESS, ACCEL_CONFIG, ACCEL_CONFIG_VALUE)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
        except (OSError, ImportError) as exc:
            if not getattr(self, "warned_", False):
                self.get_logger().warn(
                    f"No se pudo iniciar la IMU por I2C ({exc}). Revisa que exista /dev/i2c-1, "
                    "que el usuario esté en el grupo i2c y que `i2cdetect -y 1` muestre 68. "
                    "Reintentando cada segundo")
                self.warned_ = True
            self.is_connected_ = False
            return
        if getattr(self, "warned_", False):
            self.get_logger().info("IMU conectada")
            self.warned_ = False


def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Driver()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()