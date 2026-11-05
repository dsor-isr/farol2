#include "farol2_sim/sensors.hpp"

namespace farol2_sim {

DepthCellSensor::DepthCellSensor(rclcpp::Node & node)
: MountedSensor(node, "depth_cell", "depth_link"), noise_(node, "depth_cell", 3), position_noise_(noise_.load(node, "position_noise")),
  publisher_(node.create_publisher<farol2_interfaces::msg::Depth>("depth", 1)) {}

void DepthCellSensor::publish(const SensorState & sample) {
  farol2_interfaces::msg::Depth message;
  message.header.stamp = sample.stamp;
  message.header.frame_id = frame_;
  message.depth = sensorPosition(sample.state, mount_).z() - sample.environment.water_surface_z +
    noise_.sample(position_noise_, 2);
  // Preserve the legacy field value until noise/covariance semantics are revised.
  message.depth_variance = noise_.enabled() ? position_noise_.standard_deviation[2] : 0.0;
  publisher_->publish(message);
}

}  // namespace farol2_sim
