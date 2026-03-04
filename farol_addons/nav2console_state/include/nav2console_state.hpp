#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "farol_msgs/msg/navigation_state.hpp"
#include "farol_msgs/msg/state_console.hpp"

/**
 * @brief   Nav State To Console State
 * @author  Eduardo Cunha
 */
class Nav2ConsoleState : public rclcpp::Node {
  public:
    /* Constructor */
    Nav2ConsoleState();

    /* Destructor */
    ~Nav2ConsoleState();

  private:
    /* Load parameters */
    void loadParams();

    /* Initialise Subscribers */
    void initialiseSubscribers();

    /* Initialise Publishers */
    void initialisePublishers();
    
    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<farol_msgs::msg::StateConsole>::SharedPtr console_state_pub_;
    
    rclcpp::Subscription<farol_msgs::msg::NavigationState>::SharedPtr nav_state_sub_;

    /* Callbacks */
    void nav_state_callback(const farol_msgs::msg::NavigationState &msg);

    /* Other variables */
    farol_msgs::msg::StateConsole console_state_msg_;
    rclcpp::Clock clock_;
};