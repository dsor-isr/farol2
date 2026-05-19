#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <variant>
#include <Eigen/Dense>
#include <cmath>
#include <optional>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "farol2_allocation/msg/thruster_force.hpp"
#include "farol2_interfaces/msg/navigation_state.hpp"

#include <actuation_utils.hpp>
#include <farol2_utils/angles.hpp>

// Topic names (short form, remapped in launch file)
#define TOPIC_SUB_BODY_WRENCH_REQUEST "body_wrench_request"
#define TOPIC_SUB_NAV_STATE "nav_state"
#define TOPIC_SUB_MISSION_STATUS "mission_status"
#define TOPIC_PUB_THRUSTER_FORCE "thruster_force"
#define TOPIC_PUB_RUDDER_COMMAND "rudder_command"

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
    rclcpp::Publisher<farol2_allocation::msg::ThrusterForce>::SharedPtr thruster_force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_command_pub_, debug1_pub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr body_wrench_request_sub_;
    rclcpp::Subscription<farol2_interfaces::msg::NavigationState>::SharedPtr nav_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_status_sub_;
    
    /* Callbacks */
    void bodyWrenchRequestCallback(geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void navStateCallback(farol2_interfaces::msg::NavigationState::SharedPtr msg);
    void missionStatusCallback(std_msgs::msg::Int8::SharedPtr msg);

    /* Other functions */
    void computeRudderAngle(double tau_r);
    double solve_delta_from_tau(double tau_r, double gamma, double V);
    
    /* Other variables */
    rclcpp::Clock::SharedPtr clock_;
    farol2_allocation::msg::ThrusterForce thruster_force_msg_;

    std_msgs::msg::Float32 rudder_command_msg_, debug1_msg_;
    int mission_status_ = 0;

    std::vector<std::map<std::string, std::variant<std::string, std::vector<double>>>> thruster_configuration_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> thrust_allocation_matrix_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> thrust_allocation_matrix_pseudo_inv_;
    Eigen::Vector<double, 6> tau_;
    Eigen::Vector<double, 6> tau_common_mode_;
    Eigen::Vector<double, Eigen::Dynamic> forces_;
    double rudder_angle_, rudder_x_body_drag_ = 0; /* [rad], [N] */
    size_t nr_thrusters_;
    farol2_interfaces::msg::NavigationState nav_state_;
    double rudder_angle_max_, rudder_angle_min_; /* [rad] */
    double rudder_cm_distance_;
    double course_angle_, sideslip_angle_, flow_to_rudder_angle_, V_s_angle_;
    Eigen::Vector2d V_cm_, V_r_, V_s_;
    double K_s_, K_L_, K_D0_, K_D1_;
    double L, D;
    double gamma_, rudder_angle_prev_{0.0};
    double node_frequency_;
    bool open_loop_{false};
};