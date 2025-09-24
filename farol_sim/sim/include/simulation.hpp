#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "control_allocation/msg/thruster_rpm.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "AUV.hpp"
#include "Utilis.hpp"
#include <Eigen/Dense>

#include "sim/msg/placeholder.hpp"

/**
 * @brief   Sim
 * @author  Eduardo Cunha
 */
class Simulation : public rclcpp::Node {
  public:
    /* Constructor */
    Simulation();

    /* Destructor */
    ~Simulation();

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
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr orientation_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angular_velocity_pub_;

    rclcpp::Subscription<control_allocation::msg::ThrusterRPM>::SharedPtr thrust_sub_;
    
    /* Callbacks */
    void thrustCallback(const control_allocation::msg::ThrusterRPM::SharedPtr msg);

    std::unique_ptr<AUV> auv_;

    int freq_;
    double node_period_;

    Eigen::VectorXd thrust; 


    double mass;
    double zg;
    double fluid_density;
    double vehicle_density;
    
    std::vector<double> inertia;
    std::vector<double> Dl ;
    std::vector<double> Dq;
    std::vector<double> added_mass;
    std::vector<double> allocation_flat;
    std::vector<double> lump_pos;
    std::vector<double> lump_neg;
    std::vector<double> minmax_input;

    double thruster_gain;
    double thruster_pole;
    double thruster_delay;
    double sampling_period;

    std::vector<double> disturbance_mean;
    std::vector<double> disturbance_sigma;
    std::vector<double> disturbance_min;
    std::vector<double> disturbance_max;


};