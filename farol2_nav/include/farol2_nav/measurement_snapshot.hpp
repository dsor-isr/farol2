#pragma once

#include <farol2_interfaces/msg/control_surface_deflection.hpp>
#include <farol2_interfaces/msg/depth.hpp>
#include <farol2_interfaces/msg/thruster_rpm.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace farol2_nav
{
namespace filters
{

struct MeasurementSnapshot
{
  sensor_msgs::msg::Imu::SharedPtr imu{};
  sensor_msgs::msg::NavSatFix::SharedPtr gnss{};
  geometry_msgs::msg::Vector3Stamped::SharedPtr utm_ned{};
  geometry_msgs::msg::Vector3Stamped::SharedPtr velocity_over_ground{};
  geometry_msgs::msg::Vector3Stamped::SharedPtr velocity_through_water{};
  farol2_interfaces::msg::Depth::SharedPtr depth{};
  sensor_msgs::msg::Range::SharedPtr altimeter{};
  farol2_interfaces::msg::ThrusterRPM::SharedPtr thruster_rpm{};
  farol2_interfaces::msg::ControlSurfaceDeflection::SharedPtr control_surface_deflection{};
};

}  // namespace filters
}  // namespace farol2_nav
