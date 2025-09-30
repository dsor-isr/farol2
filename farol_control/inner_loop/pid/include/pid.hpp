#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/wrench.hpp"

#include "control_allocation/msg/body_wrench_request.hpp"
#include "farol_msgs/msg/navigation_state.hpp"

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
    rclcpp::Publisher<control_allocation::msg::BodyWrenchRequest>::SharedPtr body_wrench_request_pub_;

    rclcpp::Subscription<farol_msgs::msg::NavigationState>::SharedPtr nav_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr surge_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sway_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heave_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr roll_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_rate_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_rate_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr roll_rate_ref_sub_;

    /* Callbacks */
    void navStateCallback(const farol_msgs::msg::NavigationState &msg);
    void surgeRefCallback(const std_msgs::msg::Float32 &msg);
    void swayRefCallback(const std_msgs::msg::Float32 &msg);
    void heaveRefCallback(const std_msgs::msg::Float32 &msg);
    void yawRefCallback(const std_msgs::msg::Float32 &msg);
    void pitchRefCallback(const std_msgs::msg::Float32 &msg);
    void rollRefCallback(const std_msgs::msg::Float32 &msg);
    void yawRateRefCallback(const std_msgs::msg::Float32 &msg);
    void pitchRateRefCallback(const std_msgs::msg::Float32 &msg);
    void rollRateRefCallback(const std_msgs::msg::Float32 &msg);

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
    std::map<std::string, rclcpp::Time> controller_last_reference_ = {
      {"surge", rclcpp::Time(0,1)}, /* 1 nanosecond */
      {"sway", rclcpp::Time(0,1)},
      {"heave", rclcpp::Time(0,1)},
      {"yaw", rclcpp::Time(0,1)},
      {"pitch", rclcpp::Time(0,1)},
      {"roll", rclcpp::Time(0,1)},
      {"yaw_rate", rclcpp::Time(0,1)},
      {"pitch_rate", rclcpp::Time(0,1)},
      {"roll_rate", rclcpp::Time(0,1)},
      {"attitude", rclcpp::Time(0,1)}
    };

    /* Other variables */
    rclcpp::Clock clock_;
    int freq_;
    control_allocation::msg::BodyWrenchRequest body_wrench_request_msg_;
    farol_msgs::msg::NavigationState nav_state_;
    std::set<std::string> controller_names_;
    std::map<std::string, std::map<std::string, double>> controller_parameters_;
    double surge_ref_ = 0.0, sway_ref_ = 0.0, heave_ref_ = 0.0,
           yaw_ref_ = 0.0, pitch_ref_ = 0.0, roll_ref_ = 0.0,
           yaw_rate_ref_ = 0.0, pitch_rate_ref_ = 0.0, roll_rate_ref_ = 0.0;
    double tau_;

    /* Other functions */
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

class ControllerPI {
  public:
    /* Constructor */
    ControllerPI(double kp, double ki, double lpf_pole, double tau_min, double tau_max);

    /* Call for controller */
    double callController(double state, double state_ref, double dt);

    /* Methods for setting parameters */
    void setGainKp(double kp);
    void setGainKi(double ki);
    void setLPFPole(double lpf_pole);
  
  private:
    double kp_;
    double ki_;
    double lpf_pole_;
    double tau_min_;
    double tau_max_;
};

class ControllerPID {
  public:
    /* Constructor */
    ControllerPID(double kp, double ki, double kd, double lpf_pole, double tau_min, double tau_max, bool wrapToPi);

    /* Call for controller */
    double callController(double state, double state_ref, double state_rate, double dt);

    /* Methods for setting parameters */
    void setGainKp(double kp);
    void setGainKi(double ki);
    void setGainKd(double kd);
    void setLPFPole(double lpf_pole);
  
  private:
    double kp_;
    double ki_;
    double kd_;
    double lpf_pole_;
    double tau_min_;
    double tau_max_;
    bool wrapToPi_;
};