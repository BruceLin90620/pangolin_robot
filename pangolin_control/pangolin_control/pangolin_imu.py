import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
import sys
sys.path.append('/home/ubuntu/pangolin_ws/build/pangolin_control/driver')
from MPU6050 import MPU6050Base, KalmanIMU

class MPU6050Publisher(Node):

    def __init__(self):
        super().__init__('MPU6050_node')
        self.imu = MPU6050Base(address=0x68, bus=1)
        self.KalmanIMU = KalmanIMU()

        self.publisher_ = self.create_publisher(Imu, '/imu', 1)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.imuData_callback)
        self.data = None
        self.orientation_data = None
        self.frame_name = "imu_link"

    def imuData_callback(self):
        self.imu_msg = Imu()

        self.data = self.KalmanIMU.getKalmanData()
        self.imu_msg.linear_acceleration.x = self.data['accel']['x']
        self.imu_msg.linear_acceleration.y = self.data['accel']['y']
        self.imu_msg.linear_acceleration.z = self.data['accel']['z']

        self.imu_msg.angular_velocity.x = self.data['gyro']['x']
        self.imu_msg.angular_velocity.y = self.data['gyro']['y']
        self.imu_msg.angular_velocity.z = self.data['gyro']['z']

        self.imu_msg.orientation.x = self.data['orientation']['q0']
        self.imu_msg.orientation.y = self.data['orientation']['q1']
        self.imu_msg.orientation.z = self.data['orientation']['q2']
        self.imu_msg.orientation.w = self.data['orientation']['q3']

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = self.frame_name

        self.publisher_.publish(self.imu_msg)


def main(args=None):
    rclpy.init(args=args)

    mpu6050_publisher = MPU6050Publisher()

    rclpy.spin(mpu6050_publisher)

    mpu6050_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()