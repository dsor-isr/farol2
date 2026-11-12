#include "farol2_sim/sensors.hpp"
#include "farol2_utils/angles.hpp"

namespace farol2_sim {

ControlSurfaceAngleSensor::ControlSurfaceAngleSensor(rclcpp::Node & node, const std::string & base_frame)
: frame_(base_frame),
  publisher_(node.create_publisher<farol2_interfaces::msg::ControlSurfaceAngle>("control_surface_angle", 1)) {}

void ControlSurfaceAngleSensor::publish(const SensorState & sample) {
  farol2_interfaces::msg::ControlSurfaceAngle message;
  message.header.stamp = sample.stamp;
  message.header.frame_id = frame_;
  for (const double angle : sample.control_surface_angles) {
    message.angle.push_back(farol2_utils::rad2deg(angle));
  }
  publisher_->publish(message);
}

}  // namespace farol2_sim
