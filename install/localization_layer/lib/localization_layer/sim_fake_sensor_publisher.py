#!/usr/bin/env python3

import math
import os
import random

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from PIL import Image
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
        self.declare_parameter("lidar_rate_hz", 40.0)
        self.declare_parameter("imu_rate_hz", 400.0)
        self.declare_parameter("odom_rate_hz", 100.0)
        self.declare_parameter("state_rate_hz", 400.0)
        self.declare_parameter("scan_range_min", 0.12)
        self.declare_parameter("scan_range_max", 30.0)
        self.declare_parameter("scan_angle_min_deg", -180.0)
        self.declare_parameter("scan_angle_max_deg", 180.0)
        self.declare_parameter("scan_angle_increment_deg", 1.0)
        self.declare_parameter("scan_noise_std_m", 0.0)
        self.declare_parameter("scan_dropout_ratio", 0.0)
        self.declare_parameter("scan_outlier_ratio", 0.0)
        self.declare_parameter("scan_outlier_max_m", 0.3)
        self.declare_parameter("publish_static_tf", False)
        self.declare_parameter("world_type", "racing")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("random_seed", 42)
        self.declare_parameter("odom_velocity_scale_error", 0.0)
        self.declare_parameter("odom_yaw_rate_bias_deg", 0.0)
        self.declare_parameter("odom_yaw_rate_noise_std_deg", 0.0)
        self.declare_parameter("imu_yaw_bias_deg", 0.0)
        self.declare_parameter("imu_ang_vel_bias_deg", 0.0)
        self.declare_parameter("imu_ang_vel_noise_std_deg", 0.0)
        self.declare_parameter("speed_modulation_amp", 0.0)
        self.declare_parameter("slip_event_rate", 0.0)
        self.declare_parameter("slip_duration_min_sec", 0.2)
        self.declare_parameter("slip_duration_max_sec", 0.6)
        self.declare_parameter("slip_scale", 0.85)
        self.declare_parameter("map_path_speed_mps", 2.0)
        self.declare_parameter("map_path_points", 720)

        self.lidar_rate_hz = self.get_parameter("lidar_rate_hz").get_parameter_value().double_value
        self.imu_rate_hz = self.get_parameter("imu_rate_hz").get_parameter_value().double_value
        self.odom_rate_hz = self.get_parameter("odom_rate_hz").get_parameter_value().double_value
        self.state_rate_hz = self.get_parameter("state_rate_hz").get_parameter_value().double_value
        self.scan_range_min = self.get_parameter("scan_range_min").get_parameter_value().double_value
        self.scan_range_max = self.get_parameter("scan_range_max").get_parameter_value().double_value
        self.scan_angle_min = math.radians(
            self.get_parameter("scan_angle_min_deg").get_parameter_value().double_value
        )
        self.scan_angle_max = math.radians(
            self.get_parameter("scan_angle_max_deg").get_parameter_value().double_value
        )
        self.scan_angle_increment = math.radians(
            self.get_parameter("scan_angle_increment_deg").get_parameter_value().double_value
        )
        self.scan_noise_std_m = self.get_parameter("scan_noise_std_m").get_parameter_value().double_value
        self.scan_dropout_ratio = self.get_parameter("scan_dropout_ratio").get_parameter_value().double_value
        self.scan_outlier_ratio = self.get_parameter("scan_outlier_ratio").get_parameter_value().double_value
        self.scan_outlier_max_m = self.get_parameter("scan_outlier_max_m").get_parameter_value().double_value
        self.publish_static_tf = self.get_parameter(
            "publish_static_tf"
        ).get_parameter_value().bool_value
        self.world_type = self.get_parameter("world_type").get_parameter_value().string_value
        self.map_yaml_path = self.get_parameter("map_yaml_path").get_parameter_value().string_value
        self.random_seed = self.get_parameter("random_seed").get_parameter_value().integer_value
        self.odom_velocity_scale_error = self.get_parameter(
            "odom_velocity_scale_error"
        ).get_parameter_value().double_value
        self.odom_yaw_rate_bias = math.radians(
            self.get_parameter("odom_yaw_rate_bias_deg").get_parameter_value().double_value
        )
        self.odom_yaw_rate_noise_std = math.radians(
            self.get_parameter("odom_yaw_rate_noise_std_deg").get_parameter_value().double_value
        )
        self.imu_yaw_bias = math.radians(
            self.get_parameter("imu_yaw_bias_deg").get_parameter_value().double_value
        )
        self.imu_ang_vel_bias = math.radians(
            self.get_parameter("imu_ang_vel_bias_deg").get_parameter_value().double_value
        )
        self.imu_ang_vel_noise_std = math.radians(
            self.get_parameter("imu_ang_vel_noise_std_deg").get_parameter_value().double_value
        )
        self.speed_modulation_amp = self.get_parameter(
            "speed_modulation_amp"
        ).get_parameter_value().double_value
        self.slip_event_rate = self.get_parameter("slip_event_rate").get_parameter_value().double_value
        self.slip_duration_min_sec = self.get_parameter(
            "slip_duration_min_sec"
        ).get_parameter_value().double_value
        self.slip_duration_max_sec = self.get_parameter(
            "slip_duration_max_sec"
        ).get_parameter_value().double_value
        self.slip_scale = self.get_parameter("slip_scale").get_parameter_value().double_value
        self.map_path_speed_mps = self.get_parameter("map_path_speed_mps").get_parameter_value().double_value
        self.map_path_points = max(
            180, self.get_parameter("map_path_points").get_parameter_value().integer_value
        )
        self.state_dt = 1.0 / self.state_rate_hz
        self.t = 0.0
        self.rng = random.Random(self.random_seed)
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.last_odom_pub_t = None
        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_yaw = 0.0
        self.gt_v = 0.0
        self.gt_yaw_rate = 0.0
        self.gt_ax_body = 0.0
        self.gt_ay_body = 0.0
        self._slip_until_t = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0
        self.map_occ = None
        self.map_cx = 0.0
        self.map_cy = 0.0
        self.map_radius = 2.0
        self.map_omega = 0.05
        self.map_path_xy = []
        self.map_path_s = []
        self.map_path_total_length = 0.0
        self.map_path_progress = 0.0
        self._prev_gt_yaw_for_rate = 0.0

        if self.map_yaml_path:
            self._load_occupancy_map(self.map_yaml_path)
            self.world_type = "map"

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

        self.state_timer = self.create_timer(self.state_dt, self._on_state_timer)
        self.scan_timer = self.create_timer(1.0 / self.lidar_rate_hz, self._on_scan_timer)
        self.imu_timer = self.create_timer(1.0 / self.imu_rate_hz, self._on_imu_timer)
        self.odom_timer = self.create_timer(1.0 / self.odom_rate_hz, self._on_odom_timer)
        self.get_logger().info(f"Fake sensor publisher started. world_type={self.world_type}")

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

    def _load_occupancy_map(self, yaml_path: str):
        with open(yaml_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        image_path = meta.get("image", "")
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        img = Image.open(image_path).convert("L")
        data = np.array(img, dtype=np.uint8)
        self.map_height, self.map_width = data.shape
        self.map_resolution = float(meta.get("resolution", 0.05))
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        self.map_origin_x = float(origin[0])
        self.map_origin_y = float(origin[1])
        negate = int(meta.get("negate", 0))
        occ_th = float(meta.get("occupied_thresh", 0.65))

        if negate == 0:
            occ_prob = (255.0 - data.astype(np.float32)) / 255.0
        else:
            occ_prob = data.astype(np.float32) / 255.0
        self.map_occ = occ_prob >= occ_th

        width_m = self.map_width * self.map_resolution
        height_m = self.map_height * self.map_resolution

        ys, xs = np.where(self.map_occ)
        if len(xs) > 0:
            min_x = int(xs.min())
            max_x = int(xs.max())
            min_y = int(ys.min())
            max_y = int(ys.max())

            # Convert occupied bbox from image coords to world coords.
            world_x_min = self.map_origin_x + min_x * self.map_resolution
            world_x_max = self.map_origin_x + max_x * self.map_resolution
            map_y_min = self.map_height - 1 - max_y
            map_y_max = self.map_height - 1 - min_y
            world_y_min = self.map_origin_y + map_y_min * self.map_resolution
            world_y_max = self.map_origin_y + map_y_max * self.map_resolution

            span_x = max(1.0, world_x_max - world_x_min)
            span_y = max(1.0, world_y_max - world_y_min)
            self.map_cx = 0.5 * (world_x_min + world_x_max)
            self.map_cy = 0.5 * (world_y_min + world_y_max)
            self.map_radius = 0.32 * min(span_x, span_y)
            self.map_omega = 0.06
            self._build_map_path_from_polar()
        else:
            self.map_cx = self.map_origin_x + 0.5 * width_m
            self.map_cy = self.map_origin_y + 0.5 * height_m
            self.map_radius = 0.25 * min(width_m, height_m)
            self.map_omega = 0.05

    def _occupied_along_ray(self, theta: float, max_range: float):
        """Returns consolidated occupied hit distances from map center along ray."""
        if self.map_occ is None:
            return []
        step = max(0.03, 0.6 * self.map_resolution)
        c = math.cos(theta)
        s = math.sin(theta)
        d = 0.0
        prev_occ = False
        runs = []
        run_start = 0.0
        while d <= max_range:
            x = self.map_cx + d * c
            y = self.map_cy + d * s
            row, col = self._world_to_map(x, y)
            if row < 0 or row >= self.map_height or col < 0 or col >= self.map_width:
                occ = True
            else:
                occ = bool(self.map_occ[row, col])
            if occ and not prev_occ:
                run_start = d
            if prev_occ and not occ:
                runs.append((run_start, d))
            prev_occ = occ
            d += step
        if prev_occ:
            runs.append((run_start, max_range))

        centers = []
        min_run = max(2.0 * self.map_resolution, 0.04)
        for start, end in runs:
            if end - start >= min_run:
                centers.append(0.5 * (start + end))
        return centers

    def _build_map_path_from_polar(self):
        max_range = max(self.map_width, self.map_height) * self.map_resolution
        path = []
        for i in range(self.map_path_points):
            theta = (2.0 * math.pi * i) / self.map_path_points
            hits = self._occupied_along_ray(theta, max_range)
            if len(hits) >= 2:
                r = 0.5 * (hits[0] + hits[1])
                x = self.map_cx + r * math.cos(theta)
                y = self.map_cy + r * math.sin(theta)
                path.append((x, y))

        if len(path) < 60:
            self.get_logger().warn(
                "Map centerline extraction failed. Falling back to circular path."
            )
            self.map_path_xy = []
            self.map_path_s = []
            self.map_path_total_length = 0.0
            return

        # Close path if needed.
        if math.hypot(path[0][0] - path[-1][0], path[0][1] - path[-1][1]) > 0.1:
            path.append(path[0])

        filtered = [path[0]]
        s = [0.0]
        total = 0.0
        for i in range(1, len(path)):
            seg = math.hypot(path[i][0] - filtered[-1][0], path[i][1] - filtered[-1][1])
            if seg < 1e-4:
                continue
            filtered.append(path[i])
            total += seg
            s.append(total)

        if total < 2.0:
            self.map_path_xy = []
            self.map_path_s = []
            self.map_path_total_length = 0.0
            return

        self.map_path_xy = filtered
        self.map_path_s = s
        self.map_path_total_length = total
        self.map_path_progress = 0.0

    def _interpolate_map_path(self, s_query: float):
        if self.map_path_total_length <= 0.0 or len(self.map_path_xy) < 2:
            return None
        s_mod = s_query % self.map_path_total_length
        idx = 1
        while idx < len(self.map_path_s) and self.map_path_s[idx] < s_mod:
            idx += 1
        if idx >= len(self.map_path_s):
            idx = len(self.map_path_s) - 1
        s0 = self.map_path_s[idx - 1]
        s1 = self.map_path_s[idx]
        p0 = self.map_path_xy[idx - 1]
        p1 = self.map_path_xy[idx]
        if s1 - s0 < 1e-6:
            t = 0.0
        else:
            t = (s_mod - s0) / (s1 - s0)
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        return x, y, yaw

    def _world_to_map(self, x: float, y: float):
        mx = int((x - self.map_origin_x) / self.map_resolution)
        my = int((y - self.map_origin_y) / self.map_resolution)
        row = self.map_height - 1 - my
        col = mx
        return row, col

    def _ray_to_map_world(self, x0: float, y0: float, theta: float, max_range: float):
        if self.map_occ is None:
            return max_range
        step = max(0.02, 0.5 * self.map_resolution)
        c = math.cos(theta)
        s = math.sin(theta)
        d = 0.0
        while d <= max_range:
            x = x0 + d * c
            y = y0 + d * s
            row, col = self._world_to_map(x, y)
            if row < 0 or row >= self.map_height or col < 0 or col >= self.map_width:
                return d
            if self.map_occ[row, col]:
                return d
            d += step
        return max_range

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
        msg.angle_min = self.scan_angle_min
        msg.angle_max = self.scan_angle_max
        msg.angle_increment = self.scan_angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.lidar_rate_hz
        msg.range_min = self.scan_range_min
        msg.range_max = self.scan_range_max

        num_points = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        ranges = []
        for i in range(num_points):
            angle = msg.angle_min + i * msg.angle_increment
            world_theta = yaw + angle
            if self.world_type == "room":
                dist = self._ray_to_room_world(x, y, world_theta, msg.range_max)
            elif self.world_type == "map":
                dist = self._ray_to_map_world(x, y, world_theta, msg.range_max)
            else:
                dist = self._ray_to_racing_world(x, y, world_theta, msg.range_max)
            if self.scan_dropout_ratio > 0.0 and self.rng.random() < self.scan_dropout_ratio:
                ranges.append(float("inf"))
                continue

            noisy_dist = dist
            if self.scan_noise_std_m > 0.0:
                noisy_dist += self.rng.gauss(0.0, self.scan_noise_std_m)

            if self.scan_outlier_ratio > 0.0 and self.rng.random() < self.scan_outlier_ratio:
                noisy_dist += self.rng.uniform(-self.scan_outlier_max_m, self.scan_outlier_max_m)

            noisy_dist = max(msg.range_min, min(msg.range_max, noisy_dist))
            ranges.append(noisy_dist)

        msg.ranges = ranges
        self.scan_pub.publish(msg)

    def _publish_imu(self, yaw: float, yaw_rate: float):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        imu_yaw = yaw + self.imu_yaw_bias
        qx, qy, qz, qw = self._yaw_to_quat(imu_yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.z = (
            yaw_rate
            + self.imu_ang_vel_bias
            + self.rng.gauss(0.0, self.imu_ang_vel_noise_std)
        )
        msg.linear_acceleration.x = self.gt_ax_body
        msg.linear_acceleration.y = self.gt_ay_body
        msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(msg)

    def _publish_odom_and_tf(self, x: float, y: float, yaw: float, v: float, yaw_rate: float):
        if self.odom_x is None:
            self.odom_x = x
            self.odom_y = y
            self.odom_yaw = yaw
            self.last_odom_pub_t = self.t

        odom_dt = 1.0 / self.odom_rate_hz
        if self.last_odom_pub_t is not None:
            odom_dt = max(1e-4, self.t - self.last_odom_pub_t)
        self.last_odom_pub_t = self.t

        v_meas = v * (1.0 + self.odom_velocity_scale_error)
        yaw_rate_meas = (
            yaw_rate
            + self.odom_yaw_rate_bias
            + self.rng.gauss(0.0, self.odom_yaw_rate_noise_std)
        )

        # Short slip events emulate tire slip at aggressive corner entries.
        if self.slip_event_rate > 0.0 and self.t >= self._slip_until_t:
            if self.rng.random() < self.slip_event_rate:
                self._slip_until_t = self.t + self.rng.uniform(
                    self.slip_duration_min_sec, self.slip_duration_max_sec
                )
        slip_scale = self.slip_scale if self.t < self._slip_until_t else 1.0

        v_meas *= slip_scale
        yaw_rate_meas *= (2.0 - slip_scale)

        self.odom_yaw += yaw_rate_meas * odom_dt
        self.odom_x += v_meas * math.cos(self.odom_yaw) * odom_dt
        self.odom_y += v_meas * math.sin(self.odom_yaw) * odom_dt

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self._yaw_to_quat(self.odom_yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v_meas
        odom.twist.twist.angular.z = yaw_rate_meas
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.odom_x
        tf.transform.translation.y = self.odom_y
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_pub.publish(TFMessage(transforms=[tf]))

    def _update_ground_truth(self):
        # Use different trajectories by world type.
        if self.world_type == "room":
            radius = 2.5
            omega = 0.12
        elif self.world_type == "map":
            if self.map_path_total_length > 0.0:
                v = max(0.3, self.map_path_speed_mps)
                self.map_path_progress += v * self.state_dt
                interp = self._interpolate_map_path(self.map_path_progress)
                if interp is None:
                    x = self.map_cx + self.map_radius * math.cos(self.map_omega * self.t)
                    y = self.map_cy + self.map_radius * math.sin(self.map_omega * self.t)
                    yaw = self.map_omega * self.t + math.pi / 2.0
                else:
                    x, y, yaw = interp
                if self.t <= self.state_dt:
                    yaw_rate = 0.0
                else:
                    dyaw = math.atan2(
                        math.sin(yaw - self._prev_gt_yaw_for_rate),
                        math.cos(yaw - self._prev_gt_yaw_for_rate),
                    )
                    yaw_rate = dyaw / self.state_dt
                self._prev_gt_yaw_for_rate = yaw
            else:
                radius = self.map_radius
                omega = self.map_omega
                x = self.map_cx + radius * math.cos(omega * self.t)
                y = self.map_cy + radius * math.sin(omega * self.t)
                yaw = omega * self.t + math.pi / 2.0
                v = radius * omega
                yaw_rate = omega
            ax_body = 0.0
            ay_body = v * yaw_rate

            self.gt_x = x
            self.gt_y = y
            self.gt_yaw = yaw
            self.gt_v = v
            self.gt_yaw_rate = yaw_rate
            self.gt_ax_body = ax_body
            self.gt_ay_body = ay_body
            self.t += self.state_dt
            return
        else:
            radius = 6.0
            omega = 0.16
        # Add mild speed modulation to better emulate real driving.
        omega_mod = omega * (1.0 + self.speed_modulation_amp * math.sin(0.25 * self.t))
        x = radius * math.cos(omega_mod * self.t)
        y = radius * math.sin(omega_mod * self.t)
        yaw = omega_mod * self.t + math.pi / 2.0
        v = radius * omega_mod
        yaw_rate = omega_mod
        ax_body = 0.0
        ay_body = v * yaw_rate

        self.gt_x = x
        self.gt_y = y
        self.gt_yaw = yaw
        self.gt_v = v
        self.gt_yaw_rate = yaw_rate
        self.gt_ax_body = ax_body
        self.gt_ay_body = ay_body
        self.t += self.state_dt

    def _on_state_timer(self):
        self._update_ground_truth()

    def _on_scan_timer(self):
        self._publish_scan(self.gt_x, self.gt_y, self.gt_yaw)

    def _on_imu_timer(self):
        self._publish_imu(self.gt_yaw, self.gt_yaw_rate)

    def _on_odom_timer(self):
        self._publish_odom_and_tf(
            self.gt_x,
            self.gt_y,
            self.gt_yaw,
            self.gt_v,
            self.gt_yaw_rate,
        )


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
