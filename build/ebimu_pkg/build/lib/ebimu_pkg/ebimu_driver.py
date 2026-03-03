import rclpy
from rclpy.node import Node

import serial
import math

from sensor_msgs.msg import Imu
from std_msgs.msg import Header


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return (qx, qy, qz, qw)


class EbimuDriver(Node):

    def __init__(self):
        super().__init__('ebimu_driver')

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        self.ser = serial.Serial(port, baud, timeout=1)

        self.imu_pub = self.create_publisher(
            Imu,
            "/imu/data",
            10)

        self.timer = self.create_timer(
            0.01,   # 100 Hz
            self.read_serial)

        self.get_logger().info("EBIMU Driver Started")

    # Serial Read
    def read_serial(self):

        if self.ser.in_waiting == 0:
            return

        line = self.ser.readline().decode('utf-8').strip()

        try:
            data = line.split(",")

            roll  = float(data[0])
            pitch = float(data[1])
            yaw   = float(data[2])

            gx = float(data[3])
            gy = float(data[4])
            gz = float(data[5])

            ax = float(data[6])
            ay = float(data[7])
            az = float(data[8])

            self.publish_imu(
                roll, pitch, yaw,
                gx, gy, gz,
                ax, ay, az)

        except:
            self.get_logger().warn("Parsing Error")

    # Publish IMU
    def publish_imu(self,
                    roll, pitch, yaw,
                    gx, gy, gz,
                    ax, ay, az):

        imu_msg = Imu()

        # ===== Header =====
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"  # 반드시 TF 설정할 때 imu_frame에서 설정했던 frame_id와 같게 해야지 서로 연결됩니다.

        # ===== Orientation =====
        q = quaternion_from_euler(
            math.radians(roll),
            math.radians(pitch),
            math.radians(yaw))

        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        # ===== Angular velocity =====
        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)

        # ===== Linear acceleration =====
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        self.imu_pub.publish(imu_msg)
        
def main(args=None):

    rclpy.init(args=args)

    node = EbimuDriver()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
