#include "farol2_sim/sensors.hpp"

namespace farol2_sim {

ThrusterRpmSensor::ThrusterRpmSensor(rclcpp::Node & node, const std::string & base_frame)
: frame_(base_frame),
  publisher_(node.create_publisher<farol2_interfaces::msg::ThrusterRPM>("thruster_rpm", 1)) {}

void ThrusterRpmSensor::publish(const SensorState & sample) {
  farol2_interfaces::msg::ThrusterRPM message;
  message.header.stamp = sample.stamp;
  message.header.frame_id = frame_;
  message.rpm = sample.thruster_rpm;
  publisher_->publish(message);
}

}  // namespace farol2_sim
