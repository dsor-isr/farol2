#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include <rosgraph_msgs/msg/clock.hpp> 
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "control_allocation/msg/thruster_rpm.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sim_utilis/AUV.hpp"
#include "sim_utilis/Utilis.hpp"
#include <Eigen/Dense>
#include "farol_interfaces/msg/utm.hpp"
#include "std_msgs/msg/float32.hpp"



/**
 * @brief   Sim
 * @author  Eduardo Cunha
 */
class AuvSim : public rclcpp::Node {
  public:
    /* Constructor */
    AuvSim();

    /* Destructor */
    ~AuvSim();

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

    // wall-time callback: advance sim time & publish /clock
    void onClockTick();     

    // ROS-time callback: run one physics step and publish outputs
    void onSimTick();

  private:
    /* Timer for node's callbacks */

    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<farol_interfaces::msg::UTM>::SharedPtr utm_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr body_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr orientation_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr orientation_rate_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr body_acceleration_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angular_acceleration_pub_;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

    rclcpp::Subscription<control_allocation::msg::ThrusterRPM>::SharedPtr rpm_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;   
    rclcpp::Clock::SharedPtr clock_;
  

    /* Callbacks */
    void rpmCallback(const control_allocation::msg::ThrusterRPM::SharedPtr msg);

    std::unique_ptr<AUV> auv_;

    int freq_;
    double node_period_;
    Eigen::VectorXd rpm_; 

    double originLat;
    double originLon;

    double originNorthing;
    double originEasting;
    int UTMZone;
    bool northp;

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