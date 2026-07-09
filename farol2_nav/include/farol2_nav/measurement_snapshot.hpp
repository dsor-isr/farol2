#pragma once

#include <farol2_interfaces/msg/control_surface_angle.hpp>
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

/*
  This is used to hold all the measuremnt messages that are received so that they can be easily passed to all the filter plugins in an effiecient way. 
  TODO: Right now only one message of each kind is stored, consider changing to make this a buffer of messages for each type so that the filters can use all the available measurements.
*/

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
  farol2_interfaces::msg::ControlSurfaceAngle::SharedPtr control_surface_angle{};
};

}  // namespace filters
}  // namespace farol2_nav
