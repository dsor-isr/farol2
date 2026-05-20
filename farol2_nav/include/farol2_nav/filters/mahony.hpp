#pragma once

#include <farol2_nav/filters/base_filter.hpp>

namespace farol2_nav
{
namespace filters
{

class MahonyFilter : public BaseFilter
{
public:
  std::string name() const override { return "orientation_mahony"; }
  void configure(rclcpp::Node & node) override;
  void update(double dt_s, const MeasurementSnapshot & measurements, State & state) override;

private:
  double blend_{0.15};
};

}  // namespace filters
}  // namespace farol2_nav
