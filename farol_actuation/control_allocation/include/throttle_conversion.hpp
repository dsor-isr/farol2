#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "std_msgs/msg/float32.hpp"

#include "control_allocation/msg/thruster_rpm.hpp"
#include "farol_msgs/msg/thruster.hpp"

/**
 * @brief   RPM Conversion
 * @author  Eduardo Cunha
 */
class ThrottleConversion : public rclcpp::Node {
  public:
    /* Constructor */
    ThrottleConversion();

    /* Destructor */
    ~ThrottleConversion();

    /* Load parameters */
    void loadParams();

    /* Initialise Subscribers */
    void initialiseSubscribers();

    /* Initialise Publishers */
    void initialisePublishers();

    /* Initialise Services */
    void initialiseServices();

  private:

    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Subscription<control_allocation::msg::ThrusterRPM>::SharedPtr rpm_command_sub_;

    rclcpp::Publisher<farol_msgs::msg::Thruster>::SharedPtr throttle_command_pub_;
    
    /* Callbacks */
    void rpmCommandCallback(const control_allocation::msg::ThrusterRPM &msg);

    /* Other functions */
    
    /* Other variables */
    rclcpp::Clock clock_;
    farol_msgs::msg::Thruster throttle_command_msg_;
    double k_;
    // double rpm_value_, max_rpm_, min_rpm_;
};