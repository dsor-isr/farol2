#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "farol2_interfaces/msg/thruster_rpm.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <Eigen/Dense>
#include "farol2_interfaces/msg/utm.hpp"
#include "farol2_interfaces/msg/velocity.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sim_utilis/Utilis.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <GeographicLib/UTMUPS.hpp>



// Topic names (short form, remapped in launch file)
static constexpr char TOPIC_SUB_RPM_COMMAND[] = "rpm_command";
static constexpr char TOPIC_SUB_RUDDER_CMD[] = "rudder_cmd";
static constexpr char TOPIC_SUB_RUDDER_REF[] = "rudder_ref";
static constexpr char TOPIC_PUB_POSITION[] = "position";
static constexpr char TOPIC_PUB_BODY_VELOCITY[] = "body_velocity";
static constexpr char TOPIC_PUB_ORIENTATION[] = "orientation";
static constexpr char TOPIC_PUB_ORIENTATION_RATE[] = "orientation_rate";
static constexpr char TOPIC_PUB_BODY_ACCELERATION[] = "body_acceleration";
static constexpr char TOPIC_PUB_ANGULAR_ACCELERATION[] = "angular_acceleration";
static constexpr char TOPIC_PUB_RUDDER_ANGLE[] = "rudder_angle";
static constexpr char TOPIC_PUB_JOINT_STATES[] = "joint_states";
static constexpr char TOPIC_PUB_IMU[] = "imu";
static constexpr char TOPIC_PUB_GNSS[] = "gnss";
static constexpr char TOPIC_PUB_UTM_NED[] = "ned_utm";
static constexpr char TOPIC_PUB_VELOCITY_OVER_GROUND[] = "velocity_over_ground";
static constexpr char TOPIC_PUB_VELOCITY_THROUGH_WATER[] = "velocity_through_water";
static constexpr char TOPIC_PUB_DEPTH[] = "depth";
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
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr utm_ned_pub_;
    rclcpp::Publisher<farol2_interfaces::msg::Velocity>::SharedPtr velocity_over_ground_pub_;
    rclcpp::Publisher<farol2_interfaces::msg::Velocity>::SharedPtr velocity_through_water_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<farol2_interfaces::msg::ThrusterRPM>::SharedPtr rpm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_ref_sub_;

    rclcpp::TimerBase::SharedPtr timer_;  
    
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
    void rpmCallback(const farol2_interfaces::msg::ThrusterRPM::SharedPtr msg);
    void updateRudder(double command, double dt);
    void updateState();
    double rudder_command_{0.0};

    int node_frequency_;
    double node_period_;

    double origin_latitude_, origin_longitude_;
    double fluid_density;
    double m_u, zg, mu, X_u, X_uu, m_uv, m_v, N_r, N_rr, K_L, K_D0, K_D1, m_r, Y_v, Y_vv;

    double tau_u_;
    double tau_r_;
    double sideslip_angle_;
  
    double rudder_angle_{0.0};
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
    void publishWorldTransform(const rclcpp::Time & stamp);
    double randn(double mu, double sigma);

    bool gnss_activate_;
    bool gnss_velocity_over_ground_activate_;
    bool depth_sensor_activate_;
    bool imu_activate_;
    bool noise_activate_;
    int    utm_zone_;
    bool   northp_;
    double northing_, easting_;
    std::string frame_prefix_;

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
