#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <variant>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "control_allocation/msg/body_wrench_request.hpp"
#include "control_allocation/msg/thruster_force.hpp"
#include "farol_msgs/msg/navigation_state.hpp"

#include <actuation_utils.hpp>

/**
 * @brief   Thruster Rudder Allocation
 * @author  Eduardo Cunha
 */
class ThrusterRudderAllocation : public rclcpp::Node {
  public:
    /* Constructor */
    ThrusterRudderAllocation();

    /* Destructor */
    ~ThrusterRudderAllocation();

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
    rclcpp::Publisher<control_allocation::msg::ThrusterForce>::SharedPtr thruster_force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_angle_ref_pub_;
    rclcpp::Subscription<control_allocation::msg::BodyWrenchRequest>::SharedPtr body_wrench_request_sub_;
    rclcpp::Subscription<farol_msgs::msg::NavigationState>::SharedPtr nav_state_sub_;
    
    /* Callbacks */
    void bodyWrenchRequestCallback(const control_allocation::msg::BodyWrenchRequest &msg);
    void navStateCallback(const farol_msgs::msg::NavigationState &msg);

    /* Other functions */
    void computeRudderAngle(double tau_r);
    
    /* Other variables */
    rclcpp::Clock clock_;
    control_allocation::msg::ThrusterForce thruster_force_msg_;
    std_msgs::msg::Float32 rudder_angle_ref_msg_;
    std::vector<std::map<std::string, std::variant<std::string, std::vector<double>>>> thruster_configuration_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> thrust_allocation_matrix_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> thrust_allocation_matrix_pseudo_inv_;
    Eigen::Vector<double, 6> tau_;
    Eigen::Vector<double, 6> tau_common_mode_;
    Eigen::Vector<double, Eigen::Dynamic> forces_;
    double rudder_angle_, rudder_x_body_drag_ = 0; /* [rad], [N] */
    int nr_thrusters_;
    farol_msgs::msg::NavigationState nav_state_;
    double rudder_angle_max_, rudder_angle_min_; /* [rad] */
    double rudder_cm_distance_;
    double course_angle_, sideslip_angle_, flow_to_rudder_angle_, V_s_angle_;
    Eigen::Vector2d V_cm_, V_r_, V_s_;
    double K_s_, K_L_, K_D0_, K_D1_;
    double L, D;
};