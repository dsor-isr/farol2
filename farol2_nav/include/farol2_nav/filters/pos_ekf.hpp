#pragma once

#include <farol2_nav/filters/base_filter.hpp>

namespace farol2_nav
{
namespace filters
{

class PosEkfFilter : public BaseFilter
{
public:
  std::string name() const override { return "position_ekf"; }
  void configure(rclcpp::Node & node) override;
  void update(double dt_s, const MeasurementSnapshot & measurements, State & state) override;

private:
  double alpha_{0.25};
};

}  // namespace filters
}  // namespace farol2_nav
