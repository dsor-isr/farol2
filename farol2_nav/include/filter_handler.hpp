#pragma once

#include <rclcpp/rclcpp.hpp>

#include <farol2_interfaces/msg/navigation_state.hpp>
#include <farol2_nav/srv/change_filter.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <string>
#include <vector>
#include <unordered_map>

class FilterHandler : public rclcpp::Node
{
public:
  FilterHandler();
  ~FilterHandler() override;

private:
  // Init
  void loadParams();
  void initialisePublishers();
  void initialiseServices();
  void initialiseSubscribers();

  // Core
  bool switchSubscription(const std::string & filter_name);

  // Service
  void changeFilterCallback(
      const std::shared_ptr<farol2_nav::srv::ChangeFilter::Request> request,
      std::shared_ptr<farol2_nav::srv::ChangeFilter::Response> response);

  // Params / config
  std::vector<std::string> filters_;
  std::string current_filter_;
  std::unordered_map<std::string, std::string> topic_by_filter_;  // filter -> topic

  // ROS interfaces
  rclcpp::Subscription<farol2_interfaces::msg::NavigationState>::SharedPtr active_sub_;
  rclcpp::Publisher<farol2_interfaces::msg::NavigationState>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;
  rclcpp::Service<farol2_nav::srv::ChangeFilter>::SharedPtr change_filter_srv_;

  // Messages
  sensor_msgs::msg::NavSatFix nav_sat_fix_msg_;
};
