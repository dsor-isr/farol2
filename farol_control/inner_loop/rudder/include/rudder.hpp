#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

/**
 * @brief   Rudder Control
 * @author  Eduardo Cunha
 */
class Rudder : public rclcpp::Node {
  public:
    /* Constructor */
    Rudder();

    /* Destructor */
    ~Rudder();

  private:
    /* Load parameters */
    void loadParams();

    /* Initialise Subscribers */
    void initialiseSubscribers();

    /* Initialise Publishers */
    void initialisePublishers();

    /* Initialise Services */
    void initialiseServices();

    /* Initialise Timers */
    void initialiseTimers();

    /* Timer callback */
    void timerCallback();

    /* Timer for node's callbacks */
    rclcpp::TimerBase::SharedPtr timer_;
    
    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_command_pub_;
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_angle_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_angle_sub_;

    /* Callbacks */
    void rudderAngleRefCallback(const std_msgs::msg::Float32 &msg);
    void rudderAngleCallback(const std_msgs::msg::Float32 &msg);

    /* Other variables */
    std_msgs::msg::Float32 rudder_command_msg_;
    rclcpp::Clock clock_;
    double deadzone_; /* rad */
    double rudder_angle_ref_, rudder_angle_;
};