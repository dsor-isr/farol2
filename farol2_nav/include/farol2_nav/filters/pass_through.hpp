#pragma once

#include <farol2_nav/filters/base_filter.hpp>

namespace farol2_nav
{
namespace filters
{

class PassThroughFilter : public BaseFilter
{
public:
  std::string name() const override { return "sample_and_hold"; }
  void configure(rclcpp::Node & node) override;
  void update(double dt_s, const MeasurementSnapshot & measurements, State & state) override;
};

}  // namespace filters
}  // namespace farol2_nav
