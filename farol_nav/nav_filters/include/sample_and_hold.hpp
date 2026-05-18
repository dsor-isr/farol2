#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "farol2_interfaces/msg/navigation_state.hpp"
#include "farol2_interfaces/msg/measurement.hpp"
#include "control_allocation/msg/thruster_rpm.hpp"
#include "nav_filters/srv/tune_position_ekf.hpp"
#include "farol2_utils/angles.hpp"
#include "farol2_utils/filters/low_pass_filter.hpp"
#include "farol2_utils/filters/notch_filter.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <GeographicLib/UTMUPS.hpp>

/**
 * @brief   Sample and Hold navigation filter
 * @author  Eduardo Cunha
 * @author  Ravi Regalo
 */
class SampleAndHold : public rclcpp::Node {
  public:
    /* Constructor */
    SampleAndHold();

    /* Destructor */
    ~SampleAndHold();

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
    rclcpp::Publisher<farol2_interfaces::msg::NavigationState>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_raw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr model_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub1_;
    
    rclcpp::Subscription<farol2_interfaces::msg::Measurement>::SharedPtr measurement_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_sub_;
    rclcpp::Subscription<control_allocation::msg::ThrusterRPM>::SharedPtr rpm_command_sub_;
    rclcpp::Service<nav_filters::srv::TunePositionEkf>::SharedPtr tune_position_ekf_srv_;

    /* Callbacks */
    void measurement_callback(farol2_interfaces::msg::Measurement::ConstSharedPtr msg);
    void rpm_command_callback(control_allocation::msg::ThrusterRPM::ConstSharedPtr msg);
    void tune_position_ekf_callback(
      const std::shared_ptr<nav_filters::srv::TunePositionEkf::Request> request,
      std::shared_ptr<nav_filters::srv::TunePositionEkf::Response> response);

    void predict_position_ekf(double dt);
    void update_position_ekf(double northing, double easting);
    void publish_position_from_ekf();
    double rpm_to_body_speed_mps(double dt);


    /* Other variables */
    farol2_interfaces::msg::NavigationState filter_state_msg_;
    rclcpp::Clock::SharedPtr clock_;
    bool neglect_current_;
    double node_frequency_;
    farol2_utils::LowPassFilter yaw_rate_lpf_;
    farol2_utils::NotchFilter yaw_rate_notch_filter_;
    double last_time_{-1.0};
    double course_angle_est_{0.0};
    bool use_yaw_rate_lpf_{false};
    bool use_yaw_rate_kf_{false};
    bool use_course_cf_{false};
    bool use_yaw_rate_notch_filter_{false};
    double yaw_rate_notch_f0_hz_{1.0};
    double yaw_rate_notch_q_{2.0};
    double last_yaw_rate_meas_{0.0};
    double course_angle_cutoff_frequency_{0.0};

    // Kalman filter states
    Eigen::Vector2d x_;  // State vector: [yaw_rate (rad/s), yaw (rad)]
    Eigen::Matrix2d P_;  // State covariance
    Eigen::Matrix2d A_;  // State transition matrix
    Eigen::Vector2d B_;  // Input matrix (for torque)
    Eigen::RowVector2d C_;  // Measurement matrix (for yaw)
    Eigen::Matrix2d Q_;  // Process noise covariance
    Eigen::Matrix<double, 1, 1> R_;  // Measurement noise covariance

    // Parameters for Kalman filter
    double time_constant_;  // Time constant T for first-order system
    double rudder_gain_;    // Gain to convert rudder angle to torque
    double process_noise_q_;  // Process noise variance
    double measurement_noise_r_;  // Measurement noise variance
    

    double rudder_angle_;  // Latest rudder angle measurement

  // Position EKF state: [northing, easting, current_n, current_e]
  bool use_position_ekf_{false};
  bool position_ekf_initialized_{false};
  double gps_timeout_s_{3.0};
  double rpm_timeout_s_{1.0};
  double ekf_q_pos_{0.05};
  double ekf_q_current_{0.01};
  double ekf_r_pos_{4.0};
  double ekf_p0_pos_{25.0};
  double ekf_p0_current_{1.0};
  // RPM-to-surge dynamics model parameters
  double rpm_min_{-2000.0};
  double rpm_max_{2000.0};
  double rpm_rate_limit_{800.0};
  double rho_{1025.0};
  double prop_pitch_{0.25};
  double prop_diameter_{0.3};
  double k_t_bp_{0.12};
  double m_u_{250.0};
  double x_u_{-40.0};
  double x_uu_{-120.0};
  // Override velocity at steady-state RPM
  double override_velocity_{0.0};
  double override_rpms_{600.0};
  double override_timeout_s_{5.0};
  double latest_rpm_command_{0.0};

  double rpm_model_state_{0.0};
  double u_estimated_{0.0}; // Surge state for dynamics model
  double time_in_override_zone_s_{0.0}; // Time RPM has been in override zone
  bool rpm_model_initialized_{false};
  double last_rpm_command_time_s_{-1.0};
  double last_gps_update_time_s_{-1.0};
  int latest_utm_zone_{29};

  Eigen::Vector4d x_pos_ = Eigen::Vector4d::Zero();
  Eigen::Matrix4d P_pos_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Q_pos_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix2d R_pos_ = Eigen::Matrix2d::Identity();
  Eigen::Matrix<double, 2, 4> H_pos_;


};