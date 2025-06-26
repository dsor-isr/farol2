#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "std_msgs/msg/float32.hpp"
#include "control_allocation/msg/thruster_force.hpp"
#include "control_allocation/msg/thruster_rpm.hpp"

/**
 * @brief   RPM Conversion
 * @author  Eduardo Cunha
 */
class RPMConversion : public rclcpp::Node {
  public:
    /* Constructor */
    RPMConversion();

    /* Destructor */
    ~RPMConversion();

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

  private:
    /* Timer for node's callbacks */
    rclcpp::TimerBase::SharedPtr timer_;

    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Subscription<control_allocation::msg::ThrusterForce>::SharedPtr thruster_force_sub_;
    rclcpp::Publisher<control_allocation::msg::ThrusterRPM>::SharedPtr rpm_command_pub_;
    
    /* Callbacks */
    void thrusterForceCallback(const control_allocation::msg::ThrusterForce &msg);

    /* Other functions */
    
    /* Other variables */
    control_allocation::msg::ThrusterRPM rpm_command_msg_;
    std::vector<double> coef_fwd_, coef_bwd_;
    double rpm_value_, max_rpm_, min_rpm_;
};