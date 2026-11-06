#include "farol2_sim/sensors.hpp"

namespace farol2_sim {

OdometerSensor::OdometerSensor(rclcpp::Node & node)
: MountedSensor(node, "odometer", "odometer_link"), noise_(node, "odometer", 4),
  velocity_noise_(noise_.load(node, "velocity_noise")),
  publisher_(node.create_publisher<geometry_msgs::msg::Vector3Stamped>("velocity_through_water", 1)) {}

void OdometerSensor::publish(const SensorState & sample) {
  const auto velocity = sensorVelocity(sample.state, sample.environment, mount_, VelocityReference::Water);
  geometry_msgs::msg::Vector3Stamped message;
  message.header.stamp = sample.stamp;
  message.header.frame_id = frame_;
  message.vector.x = velocity.x() + noise_.sample(velocity_noise_, 0);
  message.vector.y = velocity.y() + noise_.sample(velocity_noise_, 1);
  message.vector.z = velocity.z() + noise_.sample(velocity_noise_, 2);
  publisher_->publish(message);
}

}  // namespace farol2_sim
