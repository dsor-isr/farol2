#include <farol2_nav/filters/pos_ekf.hpp>

#include <algorithm>

namespace farol2_nav
{
namespace filters
{

void PosEkfFilter::configure(rclcpp::Node & node)
{
  const double alpha = node.declare_parameter<double>("plugins.position_ekf.alpha", 0.25);
  alpha_ = std::clamp(alpha, 0.0, 1.0);
}

void PosEkfFilter::update(double, const MeasurementSnapshot & m, State & s)
{
  if (m.utm_ned == nullptr) {
    return;
  }

  s.northing_m = alpha_ * m.utm_ned->vector.x + (1.0 - alpha_) * s.northing_m;
  s.easting_m = alpha_ * m.utm_ned->vector.y + (1.0 - alpha_) * s.easting_m;
  s.utm_zone = static_cast<int32_t>(m.utm_ned->vector.z);
}

}  // namespace filters
}  // namespace farol2_nav
