#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include <rosgraph_msgs/msg/clock.hpp> 
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "farol2_allocation/msg/thruster_rpm.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sim_utilis/AUV.hpp"
#include "sim_utilis/Utilis.hpp"
#include <Eigen/Dense>
#include "farol2_interfaces/msg/utm.hpp"
#include "farol2_interfaces/msg/measurement.hpp"
#include "std_msgs/msg/float32.hpp"
#include <GeographicLib/UTMUPS.hpp>



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

    void tickClock();

    // wall-time callback: advance sim time & publish /clock
    void onClockTick();     

    // ROS-time callback: run one physics step and publish outputs
    void onSimTick();

  private:
    /* Timer for node's callbacks */

    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<farol2_interfaces::msg::UTM>::SharedPtr utm_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr body_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr orientation_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr orientation_rate_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr body_acceleration_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angular_acceleration_pub_;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Publisher<farol2_interfaces::msg::Measurement>::SharedPtr meas_pub_;

    rclcpp::Subscription<farol2_allocation::msg::ThrusterRPM>::SharedPtr rpm_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;   
    rclcpp::Clock::SharedPtr clock_;
    uint64_t sim_time_ns_{0};
    uint64_t dt_ns_{0};
  

    /* Callbacks */
    void rpmCallback(const farol2_allocation::msg::ThrusterRPM::SharedPtr msg);

    std::unique_ptr<AUV> auv_;

    double speedup_;
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

    double thruster_pole;
    double thruster_delay;
    double sampling_period;

    std::vector<double> disturbance_mean;
    std::vector<double> disturbance_sigma;
    std::vector<double> disturbance_min;
    std::vector<double> disturbance_max;

    // Sensor / measurement publishing
    void publishMeasurements();
    double randn(double mu, double sigma);

    bool gnss_activate_;
    bool depth_sensor_activate_;
    bool imu_activate_;
    bool noise_activate_;

    int    utm_zone_;
    bool   northp_;
    double northing_, easting_;

    std::array<double,3> pos_bias{};
    std::array<double,3> pos_variance{};
    std::array<double,3> ori_bias{};
    std::array<double,3> ori_variance{};
    std::array<double,3> vel_bias{};
    std::array<double,3> vel_variance{};
    std::array<double,3> fluid_vel_bias{};
    std::array<double,3> fluid_vel_variance{};
    std::array<double,3> ori_rate_bias{};
    std::array<double,3> ori_rate_variance{};
};
