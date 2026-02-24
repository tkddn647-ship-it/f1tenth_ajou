#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__("fake_sensor_publisher")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("publish_static_tf", False)
        self.declare_parameter("world_type", "racing")

        self.rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.publish_static_tf = self.get_parameter(
            "publish_static_tf"
        ).get_parameter_value().bool_value
        self.world_type = self.get_parameter("world_type").get_parameter_value().string_value
        self.dt = 1.0 / self.rate_hz
        self.t = 0.0

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.imu_pub = self.create_publisher(Imu, "/ebimu/imu", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        tf_qos = QoSProfile(depth=100)
        tf_qos.reliability = ReliabilityPolicy.RELIABLE

        tf_static_qos = QoSProfile(depth=1)
        tf_static_qos.reliability = ReliabilityPolicy.RELIABLE
        tf_static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.tf_pub = self.create_publisher(TFMessage, "/tf", tf_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", tf_static_qos)

        if self.publish_static_tf:
            self._publish_static_tfs()

        self.timer = self.create_timer(self.dt, self._on_timer)
        self.get_logger().info("Fake sensor publisher started.")

    def _yaw_to_quat(self, yaw: float):
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        return 0.0, 0.0, qz, qw

    def _publish_static_tfs(self):
        now = self.get_clock().now().to_msg()

        tf_laser = TransformStamped()
        tf_laser.header.stamp = now
        tf_laser.header.frame_id = "base_link"
        tf_laser.child_frame_id = "laser"
        tf_laser.transform.translation.x = 1.0
        tf_laser.transform.translation.y = 0.0
        tf_laser.transform.translation.z = 1.0
        tf_laser.transform.rotation.w = 1.0

        tf_imu = TransformStamped()
        tf_imu.header.stamp = now
        tf_imu.header.frame_id = "base_link"
        tf_imu.child_frame_id = "imu_link"
        tf_imu.transform.translation.x = 0.6
        tf_imu.transform.translation.y = 0.0
        tf_imu.transform.translation.z = 0.5
        tf_imu.transform.rotation.w = 1.0

        self.tf_static_pub.publish(TFMessage(transforms=[tf_laser, tf_imu]))

    def _ray_circle_hits(
        self, x0: float, y0: float, c: float, s: float, cx: float, cy: float, r: float
    ):
        fx = x0 - cx
        fy = y0 - cy
        b = 2.0 * (fx * c + fy * s)
        cterm = fx * fx + fy * fy - r * r
        disc = b * b - 4.0 * cterm
        if disc < 0.0:
            return []
        sq = math.sqrt(disc)
        t1 = (-b - sq) / 2.0
        t2 = (-b + sq) / 2.0
        return [t for t in (t1, t2) if t > 0.02]

    def _ray_to_room_world(self, x0: float, y0: float, theta: float, max_range: float):
        c = math.cos(theta)
        s = math.sin(theta)
        eps = 1e-9
        hits = []

        # Room bounds: x,y in [-8, 8]
        for xw in (-8.0, 8.0):
            if abs(c) > eps:
                t = (xw - x0) / c
                y = y0 + t * s
                if t > 0.0 and -8.0 <= y <= 8.0:
                    hits.append(t)
        for yw in (-8.0, 8.0):
            if abs(s) > eps:
                t = (yw - y0) / s
                x = x0 + t * c
                if t > 0.0 and -8.0 <= x <= 8.0:
                    hits.append(t)

        hits.extend(self._ray_circle_hits(x0, y0, c, s, 2.0, 1.0, 1.0))

        if not hits:
            return max_range
        return min(max_range, min(hits))

    def _ray_to_racing_world(self, x0: float, y0: float, theta: float, max_range: float):
        c = math.cos(theta)
        s = math.sin(theta)
        hits = []

        # Simple circular race track: annulus between inner/outer walls.
        hits.extend(self._ray_circle_hits(x0, y0, c, s, 0.0, 0.0, 8.0))
        hits.extend(self._ray_circle_hits(x0, y0, c, s, 0.0, 0.0, 4.0))

        # Add two landmark obstacles to reduce symmetry.
        hits.extend(self._ray_circle_hits(x0, y0, c, s, 0.0, 6.2, 0.45))
        hits.extend(self._ray_circle_hits(x0, y0, c, s, -5.8, 0.0, 0.45))

        if not hits:
            return max_range
        return min(max_range, min(hits))

    def _publish_scan(self, x: float, y: float, yaw: float):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.radians(1.0)
        msg.time_increment = 0.0
        msg.scan_time = self.dt
        msg.range_min = 0.12
        msg.range_max = 15.0

        num_points = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        ranges = []
        for i in range(num_points):
            angle = msg.angle_min + i * msg.angle_increment
            world_theta = yaw + angle
            if self.world_type == "room":
                dist = self._ray_to_room_world(x, y, world_theta, msg.range_max)
            else:
                dist = self._ray_to_racing_world(x, y, world_theta, msg.range_max)
            ranges.append(dist)

        msg.ranges = ranges
        self.scan_pub.publish(msg)

    def _publish_imu(self, yaw: float, yaw_rate: float):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        qx, qy, qz, qw = self._yaw_to_quat(yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.z = yaw_rate
        msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(msg)

    def _publish_odom_and_tf(self, x: float, y: float, yaw: float, v: float, yaw_rate: float):
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self._yaw_to_quat(yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = yaw_rate
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_pub.publish(TFMessage(transforms=[tf]))

    def _on_timer(self):
        # Use different trajectories by world type.
        if self.world_type == "room":
            radius = 2.5
            omega = 0.12
        else:
            radius = 6.0
            omega = 0.16
        x = radius * math.cos(omega * self.t)
        y = radius * math.sin(omega * self.t)
        yaw = omega * self.t + math.pi / 2.0
        v = radius * omega
        yaw_rate = omega

        self._publish_scan(x, y, yaw)
        self._publish_imu(yaw, yaw_rate)
        self._publish_odom_and_tf(x, y, yaw, v, yaw_rate)
        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
