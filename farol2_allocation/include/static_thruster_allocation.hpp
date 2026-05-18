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
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "farol2_allocation/msg/thruster_force.hpp"

#include <actuation_utils.hpp>

/**
 * @brief   Static Thruster Allocation
 * @author  DSOR Team
 */
class StaticThrusterAllocation : public rclcpp::Node {
  public:
    /* Constructor */
    StaticThrusterAllocation();

    /* Destructor */
    ~StaticThrusterAllocation();

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
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr body_wrench_request_sub_;
    
    /* Callbacks */
    void bodyWrenchRequestCallback(geometry_msgs::msg::WrenchStamped::SharedPtr msg);

    /* Other functions */
    
    
    /* Other variables */
    rclcpp::Clock::SharedPtr clock_;
    farol2_allocation::msg::ThrusterForce msg_;
    std::vector<std::map<std::string, std::variant<std::string, std::vector<double>>>> thruster_configuration_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> thrust_allocation_matrix_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> thrust_allocation_matrix_pseudo_inv_;
    Eigen::Vector<double, 6> tau_;
    Eigen::Vector<double, Eigen::Dynamic> forces_;
    size_t nr_thrusters_;
    double node_frequency_;
};