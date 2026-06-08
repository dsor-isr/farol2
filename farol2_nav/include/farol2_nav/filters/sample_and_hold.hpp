#pragma once

#include <farol2_nav/filters/base_filter.hpp>

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

  bool initialized_{false};
  std::vector<std::string> required_measurements_{};
  std::vector<std::string> received_measurements_{};
  State s_{};
};

}  // namespace filters
}  // namespace farol2_nav
