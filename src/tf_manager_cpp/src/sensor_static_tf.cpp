#include <cmath>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

class SensorStaticTF : public rclcpp::Node
{
public:
  SensorStaticTF() : Node("sensor_static_tf_node")
  {
    broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_lidar_tf();
    publish_imu_tf();
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;

  void publish_lidar_tf()
  {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "laser";

    tf.transform.translation.x = 0.20;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    broadcaster_->sendTransform(tf);
  }

  void publish_imu_tf()
  {
    // 차체 기준(뒤에서 앞): IMU +X 오른쪽, +Y 뒤, +Z 아래.
    // ROS base_link: +X 앞, +Y 왼쪽, +Z 위.
    // R = Rz(-pi/2) * Rx(pi)  →  roll=pi, pitch=0, yaw=-pi/2
    const double roll = declare_parameter("imu_roll", M_PI);
    const double pitch = declare_parameter("imu_pitch", 0.0);
    const double yaw = declare_parameter("imu_yaw", -M_PI / 2.0);

    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "imu_link";

    tf.transform.translation.x = declare_parameter("imu_x", 0.10);
    tf.transform.translation.y = declare_parameter("imu_y", 0.00);
    tf.transform.translation.z = declare_parameter("imu_z", 0.00);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    RCLCPP_INFO(
      get_logger(),
      "IMU TF base_link->imu_link xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
      tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z,
      roll, pitch, yaw);

    broadcaster_->sendTransform(tf);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorStaticTF>());
  rclcpp::shutdown();
  return 0;
}
