#pragma once
#include <wp_controller.h>

/**
 * @brief  Waypoint controller similar to standard, using surge and yaw (nose of
 * the vehicle points to the desired position). The difference is that this one
 * limits the rate of yaw reference.
 */
class WpLoose : public WaypointController {
private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surge_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;

  void calculateRef(Vehicle_t state, WPref_t wp_ref, bool turn_radius_flag);

  void publish();

public:
  WpLoose(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surge_pub, 
          rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub);
  virtual ~WpLoose() {}
};
