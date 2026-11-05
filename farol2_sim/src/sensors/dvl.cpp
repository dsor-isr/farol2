#include "farol2_sim/sensors.hpp"
#include "farol2_sim/kinematics.hpp"
#include <stdexcept>

namespace farol2_sim {

DvlSensor::DvlSensor(rclcpp::Node & node)
: MountedSensor(node, "dvl", "dvl_link"), noise_(node, "dvl", 2) {
  const auto mode = node.declare_parameter<std::string>("sensor.dvl.mode", "bottom_track");
  output_frame_ = node.declare_parameter<std::string>("sensor.dvl.output_frame", "sensor");
  if (mode != "bottom_track" && mode != "water_track") {
    throw std::invalid_argument("sensor.dvl.mode must be bottom_track or water_track");
  }
  if (output_frame_ != "sensor" && output_frame_ != "body" && output_frame_ != "inertial") {
    throw std::invalid_argument("sensor.dvl.output_frame must be sensor, body or inertial");
  }
  bottom_track_ = mode == "bottom_track";
  velocity_noise_ = noise_.load(node, "velocity_noise");
  const auto base = node.get_parameter("vehicle.actuators.base_frame").as_string();
  const auto prefix = node.get_parameter("frame_prefix").as_string();
  base_frame_ = base.rfind(prefix, 0) == 0 ? base : prefix + base;
  publisher_ = node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
    bottom_track_ ? "velocity_over_ground" : "velocity_through_water", 1);
}

void DvlSensor::publish(const SensorState & sample) {
  const auto reference = bottom_track_ ? VelocityReference::Ground : VelocityReference::Water;
  Eigen::Vector3d velocity = sensorVelocity(sample.state, sample.environment, mount_, reference);
  std::string output_frame = frame_;
  if (output_frame_ == "body") {
    // Explicit body output is compensated to the body origin as well as axes.
    velocity = linearVelocity(sample.state, sample.environment, reference, VelocityFrame::Body);
    output_frame = base_frame_;
  } else if (output_frame_ == "inertial") {
    velocity = sensorAttitude(sample.state, mount_) * velocity;
    output_frame = "world";
  }
  geometry_msgs::msg::Vector3Stamped message;
  message.header.stamp = sample.stamp;
  message.header.frame_id = output_frame;
  message.vector.x = velocity.x() + noise_.sample(velocity_noise_, 0);
  message.vector.y = velocity.y() + noise_.sample(velocity_noise_, 1);
  message.vector.z = velocity.z() + noise_.sample(velocity_noise_, 2);
  publisher_->publish(message);
}

}  // namespace farol2_sim
