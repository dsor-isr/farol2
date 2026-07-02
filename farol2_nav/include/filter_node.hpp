#pragma once

#include <farol2_nav/filters/base_filter.hpp>
#include <farol2_nav/measurement_snapshot.hpp>
#include <farol2_nav/state.hpp>

#include <farol2_interfaces/msg/control_surface_deflection.hpp>
#include <farol2_interfaces/msg/depth.hpp>
#include <farol2_interfaces/msg/thruster_rpm.hpp>
#include <farol2_interfaces/msg/navigation_state.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <memory>
#include <string>
#include <vector>

// Topic names (short form, remapped in launch file)
// Subscribers
static constexpr char TOPIC_SUB_IMU[] = "imu";
static constexpr char TOPIC_SUB_GNSS[] = "gnss";
static constexpr char TOPIC_SUB_UTM_NED[] = "ned_utm";
static constexpr char TOPIC_SUB_VELOCITY_OVER_GROUND[] = "velocity_over_ground";
static constexpr char TOPIC_SUB_VELOCITY_THROUGH_WATER[] = "velocity_through_water";
static constexpr char TOPIC_SUB_CURRENT_NED[] = "current_velocity";
static constexpr char TOPIC_SUB_DEPTH[] = "depth";
static constexpr char TOPIC_SUB_ALTIMETER[] = "altimeter";
static constexpr char TOPIC_SUB_CONTROL_SURFACE_DEFLECTION[] = "control_surface_deflection";
static constexpr char TOPIC_SUB_THRUSTER_RPM[] = "thruster_rpm";
// Publishers
static constexpr char TOPIC_PUB_STATE[] = "state";
class FilterNode : public rclcpp::Node
{
public:
  FilterNode();

private:
  void load_params();
  void initialise_publishers();
  void initialise_subscribers();
  void build_pipeline();
  void initialise_timer();
  void on_timer();
  void fill_state_msg(const rclcpp::Time & stamp);

  double node_frequency_{10.0};
  bool publish_all_steps_{true};
  std::vector<std::string> filters_{};
  std::vector<std::string> measurements_{};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr utm_ned_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_over_ground_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_through_water_sub_;
  rclcpp::Subscription<farol2_interfaces::msg::Depth>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altimeter_sub_;
  rclcpp::Subscription<farol2_interfaces::msg::ControlSurfaceDeflection>::SharedPtr control_surface_deflection_sub_;
  rclcpp::Subscription<farol2_interfaces::msg::ThrusterRPM>::SharedPtr thruster_rpm_sub_;

  rclcpp::Publisher<farol2_interfaces::msg::NavigationState>::SharedPtr final_state_pub_;
  std::vector<rclcpp::Publisher<farol2_interfaces::msg::NavigationState>::SharedPtr> stage_pubs_{};
  rclcpp::TimerBase::SharedPtr timer_;

  farol2_nav::filters::MeasurementSnapshot snapshot_{};
  farol2_nav::filters::State state_{};
  std::vector<std::unique_ptr<farol2_nav::filters::BaseFilter>> pipeline_{};

  farol2_interfaces::msg::NavigationState msg_{};
  double last_tick_s_{-1.0};
};
