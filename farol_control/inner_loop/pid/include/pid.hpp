#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/wrench.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "farol_interfaces/msg/navigation_state.hpp"
#include "pid/srv/change_params.hpp"
#include "pid/msg/pid_debug.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <farol_utils/filters/low_pass_filter.hpp>
#include <farol_utils/angles.hpp>

#include "controller_pi.hpp"
#include "controller_pid.hpp"

enum ControllerType {
  SURGE = 0,
  SWAY = 1,
  HEAVE = 2,
  YAW = 3,
  PITCH = 4,
  ROLL = 5,
  YAW_RATE = 6,
  PITCH_RATE = 7,
  ROLL_RATE = 8,
  ATTITUDE = 9,
};

// Use the reusable controller classes from farol_control namespace
using farol_control::ControllerPI;
using farol_control::ControllerPID;

/**
 * @brief   PID
 * @author  Eduardo Cunha
 */
class PID : public rclcpp::Node {
  public:
    /* Constructor */
    PID();

    /* Destructor */
    ~PID();

  private:
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




    /* Timer for node's callbacks */
    rclcpp::TimerBase::SharedPtr timer_;
    
    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thrust_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thrust_y_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thrust_z_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_y_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_z_pub_;

    rclcpp::Subscription<farol_interfaces::msg::NavigationState>::SharedPtr nav_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr surge_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sway_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heave_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr roll_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_rate_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_rate_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr roll_rate_ref_sub_;

    rclcpp::Publisher<pid::msg::PidDebug>::SharedPtr surge_debug_pub_;
    rclcpp::Publisher<pid::msg::PidDebug>::SharedPtr sway_debug_pub_;
    rclcpp::Publisher<pid::msg::PidDebug>::SharedPtr heave_debug_pub_;
    rclcpp::Publisher<pid::msg::PidDebug>::SharedPtr yaw_debug_pub_;
    rclcpp::Publisher<pid::msg::PidDebug>::SharedPtr pitch_debug_pub_;
    rclcpp::Publisher<pid::msg::PidDebug>::SharedPtr roll_debug_pub_;
    rclcpp::Publisher<pid::msg::PidDebug>::SharedPtr yaw_rate_debug_pub_;
    rclcpp::Publisher<pid::msg::PidDebug>::SharedPtr pitch_rate_debug_pub_;
    rclcpp::Publisher<pid::msg::PidDebug>::SharedPtr roll_rate_debug_pub_;
    rclcpp::Publisher<pid::msg::PidDebug>::SharedPtr attitude_debug_pub_;

    rclcpp::Service<pid::srv::ChangeParams>::SharedPtr change_params_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr course_control_srv_;

    /* Callbacks */
    void navStateCallback(const farol_interfaces::msg::NavigationState &msg);
    void surgeRefCallback(const std_msgs::msg::Float32 &msg);
    void swayRefCallback(const std_msgs::msg::Float32 &msg);
    void heaveRefCallback(const std_msgs::msg::Float32 &msg);
    void yawRefCallback(const std_msgs::msg::Float32 &msg);
    void pitchRefCallback(const std_msgs::msg::Float32 &msg);
    void rollRefCallback(const std_msgs::msg::Float32 &msg);
    void yawRateRefCallback(const std_msgs::msg::Float32 &msg);
    void pitchRateRefCallback(const std_msgs::msg::Float32 &msg);
    void rollRateRefCallback(const std_msgs::msg::Float32 &msg);
    void changeParamsCallback(const std::shared_ptr<pid::srv::ChangeParams::Request> request,
                              std::shared_ptr<pid::srv::ChangeParams::Response> response);
    void courseControlCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /* Map to relate controller names to enum type */
    std::map<std::string, int> controller_map_ = {
      {"surge", SURGE},
      {"sway", SWAY},
      {"heave", HEAVE},
      {"yaw", YAW},
      {"pitch", PITCH},
      {"roll", ROLL},
      {"yaw_rate", YAW_RATE},
      {"pitch_rate", PITCH_RATE},
      {"roll_rate", ROLL_RATE},
      {"attitude", ATTITUDE}
    };

    /* Map with last received reference timestamps for each controller */
    std::map<std::string, rclcpp::Time> controller_last_reference_;
    // std::map<std::string, rclcpp::Time> controller_last_reference_ = {
    //   {"surge", rclcpp::Time(0,1)}, /* 1 nanosecond */
    //   {"sway", rclcpp::Time(0,1)},
    //   {"heave", rclcpp::Time(0,1)},
    //   {"yaw", rclcpp::Time(0,1)},
    //   {"pitch", rclcpp::Time(0,1)},
    //   {"roll", rclcpp::Time(0,1)},
    //   {"yaw_rate", rclcpp::Time(0,1)},
    //   {"pitch_rate", rclcpp::Time(0,1)},
    //   {"roll_rate", rclcpp::Time(0,1)},
    //   {"attitude", rclcpp::Time(0,1)}
    // };

    /* Other variables */
    double node_frequency_;
    geometry_msgs::msg::WrenchStamped body_wrench_request_msg_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time last_update_time_;
    std_msgs::msg::Float32 float32_msg_;
    farol_interfaces::msg::NavigationState nav_state_;
    std::set<std::string> controller_names_;
    std::map<std::string, bool> controller_debug_;
    std::map<std::string, std::map<std::string, double>> controller_parameters_;
    std::map<std::string, rclcpp::Publisher<pid::msg::PidDebug>::SharedPtr> debug_publishers_;
    double surge_ref_ = 0.0, sway_ref_ = 0.0, heave_ref_ = 0.0,
           yaw_ref_ = 0.0, pitch_ref_ = 0.0, roll_ref_ = 0.0,
           yaw_rate_ref_ = 0.0, pitch_rate_ref_ = 0.0, roll_rate_ref_ = 0.0;
    double tau_;
    bool course_control_{false}; // flag to switch between heading or course control
    int lpf_order_;
    std::string lpf_method_, lpf_design_;

    std::array<bool, 10> debug_mode_{}; 

    ControllerPI controller_surge_;
    ControllerPI controller_sway_;
    ControllerPI controller_heave_;
    ControllerPID controller_yaw_;
    ControllerPID controller_pitch_;
    ControllerPID controller_roll_;
    ControllerPI controller_yaw_rate_;
    ControllerPI controller_pitch_rate_;
    ControllerPI controller_roll_rate_;

    /* Other functions */
    void createControllers();
    bool hasRecentReference(const rclcpp::Time &last_reference_timestamp, const int &node_frequency);
    void callControllers();
    void callControllerSurge();
    void callControllerSway();
    void callControllerHeave();
    void callControllerYaw();
    void callControllerPitch();
    void callControllerRoll();
    void callControllerYawRate();
    void callControllerPitchRate();
    void callControllerRollRate();
    void callControllerAttitude();
    void resetBodyWrenchRequest();
};