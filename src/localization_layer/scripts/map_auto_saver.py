#!/usr/bin/env python3

import os
import signal
import subprocess
import threading
from datetime import datetime

import rclpy
from cartographer_ros_msgs.srv import WriteState
from rclpy.node import Node


class MapAutoSaver(Node):
    def __init__(self):
        super().__init__('map_auto_saver')

        self.declare_parameter('map_save_dir', '/home/tkddn647/test/maps') # 파일 경로는 Jeston Nano의 절대 경로로 지정. 예시 : /home/username/maps
        self.declare_parameter('map_file_prefix', 'cartographer_map')
        self.declare_parameter('include_unfinished_submaps', True)
        self.declare_parameter('save_on_shutdown', True)
        self.declare_parameter('save_interval_sec', 0.0)
        self.declare_parameter('export_ros_map', True)
        self.declare_parameter('ros_map_topic', '/map')
        self.declare_parameter('ros_map_format', 'png')
        self.declare_parameter('ros_map_mode', 'trinary')
        self.declare_parameter('ros_map_timeout_sec', 20.0)

        self.map_save_dir = self.get_parameter('map_save_dir').get_parameter_value().string_value
        if not os.path.isabs(self.map_save_dir):
            self.map_save_dir = os.path.abspath(self.map_save_dir)
        self.map_file_prefix = self.get_parameter('map_file_prefix').get_parameter_value().string_value
        self.include_unfinished_submaps = self.get_parameter(
            'include_unfinished_submaps'
        ).get_parameter_value().bool_value
        self.save_on_shutdown = self.get_parameter('save_on_shutdown').get_parameter_value().bool_value
        self.save_interval_sec = self.get_parameter('save_interval_sec').get_parameter_value().double_value
        self.export_ros_map = self.get_parameter('export_ros_map').get_parameter_value().bool_value
        self.ros_map_topic = self.get_parameter('ros_map_topic').get_parameter_value().string_value
        self.ros_map_format = self.get_parameter('ros_map_format').get_parameter_value().string_value
        self.ros_map_mode = self.get_parameter('ros_map_mode').get_parameter_value().string_value
        self.ros_map_timeout_sec = self.get_parameter('ros_map_timeout_sec').get_parameter_value().double_value

        os.makedirs(self.map_save_dir, exist_ok=True)
        self.ros_log_dir = os.path.join(self.map_save_dir, '.roslog')
        os.makedirs(self.ros_log_dir, exist_ok=True)

        self.write_state_client = self.create_client(WriteState, '/write_state')
        self._save_lock = threading.Lock()
        self._shutdown_save_done = False

        if self.save_interval_sec > 0.0:
            self.create_timer(self.save_interval_sec, self._periodic_save_callback)
            self.get_logger().info(
                f'Periodic auto-save enabled: every {self.save_interval_sec:.1f}s -> {self.map_save_dir}'
            )

    def _timestamped_stem(self) -> str:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        return os.path.join(self.map_save_dir, f'{self.map_file_prefix}_{ts}')

    def _save_ros_map(self, map_filestem: str, reason: str) -> bool:
        if not self.export_ros_map:
            return True

        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-t', self.ros_map_topic,
            '-f', map_filestem,
            '--fmt', self.ros_map_format,
            '--mode', self.ros_map_mode,
        ]
        env = os.environ.copy()
        env['ROS_LOG_DIR'] = self.ros_log_dir

        self.get_logger().info(
            f'Exporting ROS map ({reason}) -> {map_filestem}.{self.ros_map_format} + .yaml'
        )
        try:
            completed = subprocess.run(
                cmd,
                check=False,
                timeout=self.ros_map_timeout_sec,
                env=env,
                capture_output=True,
                text=True,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error('map_saver_cli timed out.')
            return False
        except Exception as exc:
            self.get_logger().error(f'Failed to run map_saver_cli: {exc}')
            return False

        if completed.returncode == 0:
            self.get_logger().info('ROS map export succeeded.')
            return True

        self.get_logger().error(
            f'ROS map export failed (code={completed.returncode}). '
            f'stdout="{completed.stdout.strip()}" stderr="{completed.stderr.strip()}"'
        )
        return False

    def save_map(self, reason: str) -> bool:
        with self._save_lock:
            if not self.write_state_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error('/write_state service is not available.')
                return False

            map_filestem = self._timestamped_stem()
            output_path = f'{map_filestem}.pbstream'
            req = WriteState.Request()
            req.filename = output_path
            req.include_unfinished_submaps = self.include_unfinished_submaps

            self.get_logger().info(f'Saving map ({reason}) -> {output_path}')
            future = self.write_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

            if future.done() and future.result() is not None:
                self.get_logger().info('Map saved successfully.')
                self._save_ros_map(map_filestem, reason)
                return True

            self.get_logger().error('Map save failed or timed out.')
            return False

    def _periodic_save_callback(self):
        self.save_map('periodic')

    def save_once_on_shutdown(self):
        if not self.save_on_shutdown or self._shutdown_save_done:
            return
        self._shutdown_save_done = True
        self.save_map('shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = MapAutoSaver()

    def _handle_signal(signum, _frame):
        node.get_logger().info(f'Received signal {signum}, running final map save.')
        node.save_once_on_shutdown()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.save_once_on_shutdown()
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
