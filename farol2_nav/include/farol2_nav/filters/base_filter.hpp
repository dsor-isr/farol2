#pragma once

#include <farol2_nav/measurement_snapshot.hpp>
#include <farol2_nav/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace farol2_nav
{
namespace filters
{

class BaseFilter
{
public:
  virtual ~BaseFilter() = default;

  virtual std::string name() const = 0;
  virtual void configure(rclcpp::Node & node) = 0;
  virtual void compute(double dt_s, const MeasurementSnapshot & measurements, State & state) = 0;
};

}  // namespace filters
}  // namespace farol2_nav
