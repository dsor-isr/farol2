#pragma once
#include <wp_controller.h>

/**
 * @brief  Waypoint controller using surge and yaw, where the nose of
 * the vehicle points to the desired position
 */
class WpStandard : public WaypointController {
private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surge_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;

  void calculateRef(Vehicle_t state, WPref_t wp_ref, bool turn_radius_flag) override;

  void publish() override;

public:
  WpStandard(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surge_pub, 
             rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub);
  virtual ~WpStandard() {}
};
