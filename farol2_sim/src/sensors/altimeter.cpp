#include "farol2_sim/sensors.hpp"
#include "farol2_utils/validation.hpp"
#include <cmath>

namespace farol2_sim {

AltimeterSensor::AltimeterSensor(rclcpp::Node & node)
: MountedSensor(node, "altimeter", "altimeter_link"),
  period_(node.declare_parameter<double>("sensor.altimeter.period", 20.0)) {
  farol2_utils::requirePositive(period_, "sensor.altimeter.period");
  publisher_ = node.create_publisher<sensor_msgs::msg::Range>("altimeter", 1);
}

void AltimeterSensor::publish(const SensorState & sample) {
  constexpr double two_pi = 6.28318530717958647692;
  sensor_msgs::msg::Range message;
  message.header.stamp = sample.stamp;
  message.header.frame_id = frame_;
  message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  message.field_of_view = 0.0f;
  message.min_range = 0.0f;
  message.max_range = 10.0f;
  const double phase = two_pi * std::fmod(sample.stamp.seconds(), period_) / period_;
  message.range = static_cast<float>(3.0 + std::sin(phase));
  publisher_->publish(message);
}

}  // namespace farol2_sim
