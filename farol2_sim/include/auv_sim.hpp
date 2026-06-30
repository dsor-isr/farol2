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
#include "sim_utilis/AUV.hpp"
#include "sim_utilis/Utilis.hpp"
#include <Eigen/Dense>
#include "farol2_interfaces/msg/utm.hpp"
#include "farol2_interfaces/msg/velocity.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thruster_geometry.hpp>
#include <GeographicLib/UTMUPS.hpp>

// Topic names (short form, remapped in launch file)
static constexpr char TOPIC_SUB_RPM_COMMAND[] = "rpm_command";
static constexpr char TOPIC_PUB_UTM[] = "utm";
static constexpr char TOPIC_PUB_POSITION[] = "position";
static constexpr char TOPIC_PUB_BODY_VELOCITY[] = "body_velocity";
static constexpr char TOPIC_PUB_ORIENTATION[] = "orientation";
static constexpr char TOPIC_PUB_ORIENTATION_RATE[] = "orientation_rate";
static constexpr char TOPIC_PUB_BODY_ACCELERATION[] = "body_acceleration";
static constexpr char TOPIC_PUB_ANGULAR_ACCELERATION[] = "angular_acceleration";
static constexpr char TOPIC_PUB_IMU[] = "imu";
static constexpr char TOPIC_PUB_GNSS[] = "gnss";
static constexpr char TOPIC_PUB_UTM_NED[] = "ned_utm";
static constexpr char TOPIC_PUB_VELOCITY_OVER_GROUND[] = "velocity_over_ground";
static constexpr char TOPIC_PUB_VELOCITY_THROUGH_WATER[] = "velocity_through_water";
static constexpr char TOPIC_PUB_DEPTH[] = "depth";
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

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr utm_ned_pub_;
    rclcpp::Publisher<farol2_interfaces::msg::Velocity>::SharedPtr velocity_over_ground_pub_;
    rclcpp::Publisher<farol2_interfaces::msg::Velocity>::SharedPtr velocity_through_water_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<farol2_interfaces::msg::ThrusterRPM>::SharedPtr rpm_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;   
    rclcpp::TimerBase::SharedPtr tf_initialisation_timer_;
    rclcpp::Clock::SharedPtr clock_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  

    /* Callbacks */
    void rpmCallback(const farol2_interfaces::msg::ThrusterRPM::SharedPtr msg);
    void initialiseAuv(const Eigen::MatrixXd &allocation_matrix);
    void initialiseAuvFromTF();

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
    std::string base_frame_;
    std::vector<std::string> thruster_frames_;
    bool use_tf_allocation_{false};
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
    std::vector<double> initial_position_;
    std::vector<double> initial_body_velocity_;
    std::vector<double> initial_orientation_;
    std::vector<double> initial_orientation_rate_;

    // Sensor / measurement publishing
    void publishMeasurements();
    void publishWorldTransform(const rclcpp::Time & stamp);
    double randn(double mu, double sigma);

    bool gnss_activate_;
    bool gnss_velocity_over_ground_activate_;
    bool depth_sensor_activate_;
    bool imu_activate_;
    bool dvl_activate_;
    std::string dvl_mode_;
    std::string dvl_output_frame_;
    bool noise_activate_;

    int    utm_zone_;
    bool   northp_;
    double northing_, easting_;
    std::string frame_prefix_;

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
