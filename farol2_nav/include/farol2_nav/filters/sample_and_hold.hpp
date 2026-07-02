#pragma once

#include <farol2_nav/filters/base_filter.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <vector>

namespace farol2_nav
{
namespace filters
{

class SampleAndHoldFilter : public BaseFilter
{
public:
  std::string name() const override { return "sample_and_hold"; }
  void configure(rclcpp::Node & node) override;
  void compute(double dt_s, const MeasurementSnapshot & measurements, State & state) override;
  bool initialized() const override { return initialized_; }

private:
  bool has_measurement(const std::string & name) const;
  void mark_received(const std::string & name);
  bool all_required_measurements_received() const;
  bool lookup_sensor_to_base_transform(
    const std::string & sensor_frame,
    const rclcpp::Time & stamp,
    Eigen::Isometry3d & transform_base_sensor) const;

  bool initialized_{false};
  std::vector<std::string> required_measurements_{};
  std::vector<std::string> received_measurements_{};
  State s_{};
  std::string frame_prefix_ = std::string{};
  std::string base_frame_ = "base_link";
  rclcpp::Clock::SharedPtr clock_{};
  rclcpp::Logger logger_{rclcpp::get_logger("sample_and_hold")};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{};
};

}  // namespace filters
}  // namespace farol2_nav
