#pragma once

#include <farol2_interfaces/msg/thruster_rpm.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>

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
  std_msgs::msg::Float32::SharedPtr depth{};
  std_msgs::msg::Float32::SharedPtr altimeter{};
  std_msgs::msg::Float32::SharedPtr rudder_angle{};
  farol2_interfaces::msg::ThrusterRPM::SharedPtr thruster_rpm{};

  rclcpp::Time imu_stamp{};
  rclcpp::Time gnss_stamp{};
  rclcpp::Time utm_ned_stamp{};
  rclcpp::Time velocity_over_ground_stamp{};
  rclcpp::Time velocity_through_water_stamp{};
  rclcpp::Time depth_stamp{};
  rclcpp::Time altimeter_stamp{};
  rclcpp::Time rudder_angle_stamp{};
  rclcpp::Time thruster_rpm_stamp{};
};

}  // namespace filters
}  // namespace farol2_nav
