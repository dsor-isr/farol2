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
#include "control_allocation/msg/body_wrench_request.hpp"
#include "control_allocation/msg/thruster_force.hpp"

/**
 * @brief   Static Thruster Allocation
 * @author  Eduardo Cunha
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
    rclcpp::Publisher<control_allocation::msg::ThrusterForce>::SharedPtr thruster_force_pub_;
    rclcpp::Subscription<control_allocation::msg::BodyWrenchRequest>::SharedPtr body_wrench_request_sub_;
    

    /* Callbacks */
    void bodyWrenchRequestCallback(const control_allocation::msg::BodyWrenchRequest &msg);

    /* Other functions */
    std::variant<std::string, std::vector<double>> getFieldFromParameter(rclcpp::Parameter param, std::string key_name);
    void buildThrustAllocationMatrix(const int nr_thrusters);
    Eigen::Matrix3d getRotationMatrixThruster2Body(double roll, double pitch, double yaw);
    
    /* Other variables */
    rclcpp::Clock clock_;
    control_allocation::msg::ThrusterForce msg_;
    std::vector<std::map<std::string, std::variant<std::string, std::vector<double>>>> thruster_configuration_;
    std::map<std::string, rclcpp::Parameter> raw_thruster_configuration_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> thrust_allocation_matrix_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> thrust_allocation_matrix_pseudo_inv_;
    Eigen::Vector<double, 6> tau_;
    Eigen::Vector<double, Eigen::Dynamic> forces_;
    Eigen::Vector3d f_, l_;
};