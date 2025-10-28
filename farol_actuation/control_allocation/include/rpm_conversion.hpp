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
#include "farol_msgs/msg/navigation_state.hpp"

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

  private:

    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Subscription<control_allocation::msg::ThrusterForce>::SharedPtr thruster_force_sub_;
    rclcpp::Subscription<farol_msgs::msg::NavigationState>::SharedPtr nav_state_sub_;

    rclcpp::Publisher<control_allocation::msg::ThrusterRPM>::SharedPtr rpm_command_pub_;
    
    /* Callbacks */
    void thrusterForceCallback(const control_allocation::msg::ThrusterForce &msg);
    void navStateCallback(const farol_msgs::msg::NavigationState &msg);

    /* Other functions */
    
    /* Other variables */
    control_allocation::msg::ThrusterRPM rpm_command_msg_;
    int mode_ = 1;
    std::vector<double> coef_fwd_, coef_bwd_;
    double rpm_value_, max_rpm_, min_rpm_;
    double rho_;
    double K_T_BP_;
    double prop_pitch_;
    double D_;
    double surge_ = 0.0;
};