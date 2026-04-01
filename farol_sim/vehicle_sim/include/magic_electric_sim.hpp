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
#include <Eigen/Dense>
#include "farol2_interfaces/msg/utm.hpp"
#include "farol2_interfaces/msg/measurement.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sim_utilis/Utilis.hpp"
#include <GeographicLib/UTMUPS.hpp>



/**
 * @brief   Magic Electric Simulation
 * @author  André Carvalho
 */
class MagicElectricSim : public rclcpp::Node {
  public:
    /* Constructor */
    MagicElectricSim();

    /* Destructor */
    ~MagicElectricSim();

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


    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<farol2_interfaces::msg::UTM>::SharedPtr utm_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr orientation_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr oreintation_rate_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angular_acceleration_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr body_acceleration_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Publisher<farol2_interfaces::msg::Measurement>::SharedPtr meas_pub_;

    rclcpp::Subscription<control_allocation::msg::ThrusterRPM>::SharedPtr rpm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_ref_sub_;

    rclcpp::TimerBase::SharedPtr timer_;  
    rclcpp::Clock::SharedPtr clock_;

    uint64_t sim_time_ns_{0};
    uint64_t dt_ns_{0};
    
    // std::vector<double> position_;
    // std::vector<double> position_dot_;
    // std::vector<double> body_velocity_;
    // std::vector<double> body_acceleration_;
    // std::vector<double> angular_acceleration_;
    // std::vector<double> orientation_;
    // std::vector<double> orientation_rate_;
    // std::vector<double> ruder_velocity_;
    // std::vector<double> rpm;
    // std::vector<double> rudder_velocity_;


    std::array<double,3> position_{};
    std::array<double,3> position_dot_{};
    std::array<double,3> body_velocity_{};
    std::array<double,3> body_acceleration_{};
    std::array<double,3> angular_acceleration_{};
    std::array<double,3> orientation_{};
    std::array<double,3> orientation_rate_{};
    std::array<double,3> rudder_velocity_{};
    std::array<double,1> rpm_{};

    void rudderAngleCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void rpmCallback(const control_allocation::msg::ThrusterRPM::SharedPtr msg);
    void updateRudder(double command, double dt);
    void updateState();
    void tickClock();
    double rudder_command_{0.0};

    double node_frequency_;
    double node_period_;
    double speedup_;

    double origin_latitude_, origin_longitude_;
    double fluid_density;
    double m_u, zg, mu, X_u, X_uu, m_uv, m_v, N_r, N_rr, K_L, K_D0, K_D1, m_r, Y_v, Y_vv;

    double tau_u_;
    double tau_r_;
    double sideslip_angle_;
  
    double rudder_angle_;
    double rps_;
    double course_angle_;
    double speed_;
    double max_rudder_angle_;
    double min_rudder_angle_;
    double rudder_ang_vel_;

    double gamma_;
    double alpha_;
    double lift_;
    double drag_;
    double K_T_BP, D, prop_pitch, l_;
    double mod_ruder_velocity_;

    double curr_vel_;
    double curr_dir_;

    bool fixed_rpm_;
    double fixed_rpm_value_;
    double max_rpm_;
    double min_rpm_;
    double deadzone_propeller_pos_;
    double deadzone_propeller_neg_;

    bool rudder_actuation_sim_;

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

    std::array<double,3> meas_pos_{};
    std::array<double,3> pos_bias{};
    std::array<double,3> pos_variance{};
    std::array<double,3> ori_bias{};
    std::array<double,3> ori_variance{};
    std::array<double,3> vel_bias{};
    std::array<double,3> vel_variance{};
    std::array<double,3> ori_rate_bias{};
    std::array<double,3> ori_rate_variance{};
};