#pragma once
#include <wp_controller.h>

/**
 * @brief  Waypoint controller using surge, surge and yaw rate.
 * Not only can go to waypoint and hold its position but can also maintain
 * heading.
 */
class WpHeading : public WaypointController {
private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surge_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sway_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_rate_pub_;

  void calculateRef(Vehicle_t state, WPref_t wp_ref, bool turn_radius_flag);

  void publish();

public:
  WpHeading(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surge_pub, 
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sway_pub,
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_rate_pub);
  virtual ~WpHeading() {}
};
