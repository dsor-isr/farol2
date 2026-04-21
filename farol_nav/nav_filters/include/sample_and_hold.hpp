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
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub1_;
    
    rclcpp::Subscription<farol2_interfaces::msg::Measurement>::SharedPtr measurement_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_sub_;

    /* Callbacks */
    void measurement_callback(farol2_interfaces::msg::Measurement::ConstSharedPtr msg);


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


};