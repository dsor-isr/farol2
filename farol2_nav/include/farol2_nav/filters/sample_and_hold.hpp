#pragma once

#include <farol2_nav/filters/base_filter.hpp>
#include <farol2_utils/tf_utils.hpp>

#include <rclcpp/rclcpp.hpp>

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
  bool has_initializer_measurement(const std::string & name) const;
  void mark_received(const std::string & name);
  bool all_initializer_measurements_received() const;

  bool initialized_{false};
  std::vector<std::string> initializer_measurements_{};
  std::vector<std::string> received_initializer_measurements_{};
  State s_{};
  std::string frame_prefix_ = std::string{};
  std::string base_frame_ = "base_link";
  std::unique_ptr<farol2_utils::StaticTransformLookup> static_tf_lookup_{};
};

}  // namespace filters
}  // namespace farol2_nav
