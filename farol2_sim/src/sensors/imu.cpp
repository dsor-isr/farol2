#include "farol2_sim/sensors.hpp"
#include "farol2_utils/geometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace farol2_sim {

ImuSensor::ImuSensor(rclcpp::Node & node)
: MountedSensor(node, "imu", "imu_link"), noise_(node, "imu", 1), orientation_noise_(noise_.load(node, "orientation_noise")),
  rate_noise_(noise_.load(node, "angular_velocity_noise")),
  publisher_(node.create_publisher<sensor_msgs::msg::Imu>("imu", 1)) {}

void ImuSensor::publish(const SensorState & sample) {
  const auto rpy = farol2_utils::rpyFromRotation(sensorAttitude(sample.state, mount_));
  // Keep draw order explicit so seeded noise is reproducible.
  const double roll = rpy.x() + noise_.sample(orientation_noise_, 0);
  const double pitch = rpy.y() + noise_.sample(orientation_noise_, 1);
  const double yaw = rpy.z() + noise_.sample(orientation_noise_, 2);
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  sensor_msgs::msg::Imu message;
  message.header.stamp = sample.stamp;
  message.header.frame_id = frame_;
  const auto omega = sensorAngularVelocity(sample.state, mount_);
  const auto specific_force = sensorSpecificForce(sample.state, sample.acceleration, sample.environment, mount_);
  message.orientation = tf2::toMsg(q);
  message.angular_velocity.x = omega[0] + noise_.sample(rate_noise_, 0);
  message.angular_velocity.y = omega[1] + noise_.sample(rate_noise_, 1);
  message.angular_velocity.z = omega[2] + noise_.sample(rate_noise_, 2);
  // Accelerometer specific force, including gravity and rigid-body offset terms.
  message.linear_acceleration.x = specific_force[0];
  message.linear_acceleration.y = specific_force[1];
  message.linear_acceleration.z = specific_force[2];
  publisher_->publish(message);
}

}  // namespace farol2_sim
