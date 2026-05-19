#pragma once
#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <variant>
#define _USE_MATH_DEFINES
#include <math.h>
#include "stdlib.h"
#include "vector"
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

#include "farol2_waypoint/srv/send_wp_type1.hpp"
#include "farol2_interfaces/msg/navigation_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"

#include "wp_heading.h"
#include "wp_loose.h"
#include "wp_standard.h"

// Topic/service names (short form, remapped in launch file)
#define TOPIC_SUB_MISSION_STATUS "mission_status"
#define TOPIC_SUB_STATE "state"
#define TOPIC_SUB_TURN_RADIUS_FLAG "turn_radius_flag"
#define TOPIC_PUB_YAW_REF "yaw_ref"
#define TOPIC_PUB_YAW_RATE_REF "yaw_rate_ref"
#define TOPIC_PUB_U_REF "u_ref"
#define TOPIC_PUB_V_REF "v_ref"
#define TOPIC_PUB_MISSION_STATUS "mission_status"
#define SERVICE_WP_STANDARD "wp_standard"
#define SERVICE_WP_LOOSE "wp_loose"
#define SERVICE_WP_HEADING "wp_heading"

/**
 * @brief  ROS node class
 */
class Waypoint : public rclcpp::Node {
public:
  Waypoint();

  ~Waypoint();

private:
  WaypointController *wp_controller_; ///< Pointer to farol2_waypoint controller
  WPref_t wp_ref_;                    ///< Desired farol2_waypoint
  Vehicle_t veh_state_;               ///< Vehicle state
  double node_frequency_;             ///< node main loop frequency
  std_msgs::msg::Int8 mission_status_msg_;

  /* Subscribers */
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_status_sub_;  ///< flag subscriber
  rclcpp::Subscription<farol2_interfaces::msg::NavigationState>::SharedPtr state_sub_; ///< state subscriber
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr turn_radius_flag_sub_; ///< turn radius limiter flag subscriber

  /* Publishers */
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_ref_pub_;      ///< sway publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_rate_ref_pub_; ///< yaw rate publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr u_ref_pub_;        ///< surge publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr v_ref_pub_;        ///< sway publisher
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_status_pub_; ///< mission_status publisher

  /* Services */
  rclcpp::Service<farol2_waypoint::srv::SendWpType1>::SharedPtr wp_standard_srv_; ///< standard farol2_waypoint service
  rclcpp::Service<farol2_waypoint::srv::SendWpType1>::SharedPtr wp_loose_srv_;    ///< loose farol2_waypoint service
  rclcpp::Service<farol2_waypoint::srv::SendWpType1>::SharedPtr wp_heading_srv_;  ///< heading farol2_waypoint service

  /* Timer for node's callbacks */
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters from Yaml
  double cdist_;

  // Type 1 (Standard or Loose)
  double ku_;
  double ks_;
  double delta_t_;
  double speed_turn_ = 0.0;

  // Type 2 (Heading)
  double k1_;
  double k2_;
  double k3_;

  // Topic names
  std::string state_topic_;
  std::string flag_topic_;
  std::string turn_radius_flag_topic_;
  std::string wp_ref_topic_;
  std::string u_ref_topic_;
  std::string v_ref_topic_;
  std::string yaw_ref_topic_;
  std::string yaw_rate_ref_topic_;
  std::string wp_standard_topic_;
  std::string wp_loose_topic_;
  std::string wp_heading_topic_;

  // Turn Radius Flag Boolean
  bool turn_radius_flag_{false};

  /**
   * @brief  Function to initialise subscribers
   */
  void initialiseSubscribers();

  /**
   * @brief  Function to initialise publishers
   */
  void initialisePublishers();

  /**
   * @brief  Function to initialise services
   */
  void initialiseServices();

  /**
   * @brief  Function to initialise the node main loop timer
   */
  void initialiseTimer();

  /**
   * @brief  Loads parameters from parameter server
   */
  void loadParams();

  /**
   * @brief  Callback function of the state topic.
   * Updates the vehicle state object to be then used in the farol2_waypoint
   * controllers
   *
   * @param msg contains the state info
   */
  void stateCallback(farol2_interfaces::msg::NavigationState::SharedPtr msg);

  /**
   * @brief  Callback function of the flag topic.
   * Stops the main loop if flag is different from 4 (4 equals WP mode)
   *
   * @param msg contains the flag info
   */
  void missionStatusCallback(std_msgs::msg::Int8::SharedPtr msg);

  /**
   * @brief  Callback function of the turn radius flag topic.
   *
   * @param msg contains the flag info
   */
  void turnRadiusFlagCallback(std_msgs::msg::Bool::SharedPtr msg);

  /* Timer callback */
  void timerCallback();

  /**
   * @brief  Callback function of standard waypoint service. Starts main loop
   *
   * @param req
   * @param res
   *
   * @return
   */
  void sendWpStandardService(const std::shared_ptr<farol2_waypoint::srv::SendWpType1::Request> req,
                             std::shared_ptr<farol2_waypoint::srv::SendWpType1::Response> res);
  /**
   * @brief  Callback function of loose waypoint service. Starts main loop
   *
   * @param req
   * @param res
   *
   * @return
   */
  void sendWpLooseService(const std::shared_ptr<farol2_waypoint::srv::SendWpType1::Request> req,
                          std::shared_ptr<farol2_waypoint::srv::SendWpType1::Response> res);

  /**
   * @brief  Callback function of waypoint with heading control service. Starts
   * main loop
   *
   * @param req
   * @param res
   *
   * @return
   */
  void sendWpHeadingService(const std::shared_ptr<farol2_waypoint::srv::SendWpType1::Request> req,
                            std::shared_ptr<farol2_waypoint::srv::SendWpType1::Response> res);

  /**
   * @brief  Substitutes the waypoint controller pointer
   *
   * @param new_wp
   */
  void createWaypoint(WaypointController *new_wp);

  bool decodeWaypoint(double x, double y);
};
