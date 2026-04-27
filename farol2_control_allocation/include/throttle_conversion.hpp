#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "std_msgs/msg/float32.hpp"

#include "farol2_control_allocation/msg/thruster_rpm.hpp"
#include "farol2_interfaces/msg/thruster.hpp"

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
    rclcpp::Subscription<farol2_control_allocation::msg::ThrusterRPM>::SharedPtr rpm_command_sub_;

    rclcpp::Publisher<farol2_interfaces::msg::Thruster>::SharedPtr throttle_command_pub_;
    
    /* Callbacks */
    void rpmCommandCallback(farol2_control_allocation::msg::ThrusterRPM::SharedPtr msg);

    /* Other functions */
    
    /* Other variables */
    rclcpp::Clock::SharedPtr clock_;
    farol2_interfaces::msg::Thruster throttle_command_msg_;
    double k_;
    // double rpm_value_, max_rpm_, min_rpm_;
};