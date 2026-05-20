#pragma once

#include <farol2_nav/filters/base_filter.hpp>
#include <farol2_nav/filters/measurement_snapshot.hpp>
#include <farol2_nav/filters/state.hpp>

#include <farol2_allocation/msg/thruster_rpm.hpp>
#include <farol2_interfaces/msg/navigation_state.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>

#include <memory>
#include <string>
#include <vector>

// Topic names (short form, remapped in launch file)
// Subscribers
#define TOPIC_SUB_IMU "imu"
#define TOPIC_SUB_GNSS "gnss"
#define TOPIC_SUB_UTM_NED "ned_utm"
#define TOPIC_SUB_VELOCITY_OVER_GROUND "velocity_over_ground"
#define TOPIC_SUB_VELOCITY_THROUGH_WATER "velocity_through_water"
#define TOPIC_SUB_CURRENT_NED "current_velocity"
#define TOPIC_SUB_DEPTH "depth"
#define TOPIC_SUB_ALTIMETER "altimeter"
#define TOPIC_SUB_RUDDER_ANGLE "rudder_angle"
#define TOPIC_SUB_RPM_COMMAND "rpm_command"
// Publishers
#define TOPIC_PUB_STATE "state"

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
  bool is_fresh(const rclcpp::Time & stamp, double timeout_s) const;
  void on_timer();
  void fill_state_msg(const rclcpp::Time & stamp);

  double node_frequency_{10.0};
  bool publish_all_steps_{true};
  std::vector<std::string> filters_{};
  double imu_timeout_s_{1.0};
  double navsat_timeout_s_{2.0};
  double utm_timeout_s_{2.0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr utm_ned_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_over_ground_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_through_water_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr altimeter_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_sub_;
  rclcpp::Subscription<farol2_allocation::msg::ThrusterRPM>::SharedPtr rpm_sub_;

  rclcpp::Publisher<farol2_interfaces::msg::NavigationState>::SharedPtr final_state_pub_;
  std::vector<rclcpp::Publisher<farol2_interfaces::msg::NavigationState>::SharedPtr> stage_pubs_{};
  rclcpp::TimerBase::SharedPtr timer_;

  farol2_nav::filters::MeasurementSnapshot snapshot_{};
  farol2_nav::filters::State state_{};
  std::vector<std::unique_ptr<farol2_nav::filters::BaseFilter>> pipeline_{};

  farol2_interfaces::msg::NavigationState msg_{};
  double last_tick_s_{-1.0};
};
