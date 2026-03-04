#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "farol_msgs/msg/navigation_state.hpp"
#include "farol_msgs/msg/measurement.hpp"
#include "farol_utils/angles.hpp"
#include "farol_utils/filters/low_pass_filter.hpp"
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
    rclcpp::Publisher<farol_msgs::msg::NavigationState>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub2_;
    
    rclcpp::Subscription<farol_msgs::msg::Measurement>::SharedPtr measurement_sub_;

    /* Callbacks */
    void measurement_callback(farol_msgs::msg::Measurement::ConstSharedPtr msg);


    /* Other variables */
    farol_msgs::msg::NavigationState filter_state_msg_;
    rclcpp::Clock clock_;
    bool neglect_current_;
    double node_frequency_;
    farol_utils::LowPassFilter course_lpf_;
    double last_time_;
    double course_angle_est_{0.0};

};