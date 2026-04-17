#include <cstdio>
#include <functional>
#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/wrench.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "farol2_interfaces/msg/navigation_state.hpp"
#include "farol2_pid_controller/srv/change_params.hpp"
#include "farol2_pid_controller/msg/pid_debug.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <farol2_utils/filters/low_pass_filter.hpp>
#include <farol2_utils/angles.hpp>

#include "farol2_pid_controller/controller_pi.hpp"
#include "farol2_pid_controller/controller_pid.hpp"

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
};

// Use the reusable controller classes from farol_control namespace
using farol_control::ControllerPI;
using farol_control::ControllerPID;

/**
 * @brief Runtime descriptor for one enabled controller channel.
 *
 * This structure stores the callbacks needed by the generic execution path
 * (`executeController`) so per-axis behavior can be configured once and then
 * dispatched uniformly.
 */
struct ControllerConfig {
  /** Controller name key (e.g. "yaw", "surge"). */
  std::string name;
  /** Controller type used in switch-based dispatch. */
  ControllerType type;
  /** True for PID channels that require a state-rate input. */
  bool has_state_rate;
  /** Required gain/limit parameter names for this channel. */
  std::vector<std::string> required_params;
  /** Returns the current measured state for this channel. */
  std::function<double()> get_state;
  /** Returns the latest reference for this channel. */
  std::function<double()> get_ref;
  /** Returns the measured state rate (used by PID channels). */
  std::function<double()> get_rate;
  /** Adds this channel's output to the wrench accumulator. */
  std::function<void(double)> accumulate_output;
  /** Fills debug message fields from controller internals. */
    std::function<void(farol2_pid_controller::msg::PidDebug &)> fill_debug;
};

/**
 * @brief   PID
 * @author  Eduardo Cunha
 */
class PID : public rclcpp::Node {
  public:
    /**
     * @brief Construct the PID node and initialize runtime resources.
     *
     * Initialization order is intentionally fixed:
     * load parameters -> create ROS interfaces -> create controllers ->
     * build controller configuration table.
     */
    PID();

    /**
     * @brief Destroy the node and stop the periodic timer.
     */
    ~PID();

  private:
    /**
     * @brief Load and parse node/controller parameters.
     *
     * This function builds `controller_parameters_`, `controller_debug_`, and
     * `controller_names_` (enabled controllers only).
     */
    void loadParams();

    /**
     * @brief Create navigation state and per-controller reference subscribers.
     */
    void initialiseSubscribers();

    /**
     * @brief Create force/torque publishers and optional debug publishers.
     */
    void initialisePublishers();

    /**
     * @brief Create parameter-update and course-control services.
     */
    void initialiseServices();

    /**
     * @brief Create the periodic timer using `node_frequency_`.
     */
    void initialiseTimers();
    
    /**
     * @brief Main periodic loop: safety checks, control update, publish outputs.
     */
    void timerCallback();




    /** Timer that drives the control loop. */
    rclcpp::TimerBase::SharedPtr timer_;
    
    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thrust_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thrust_y_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thrust_z_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_y_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_z_pub_;

    rclcpp::Subscription<farol2_interfaces::msg::NavigationState>::SharedPtr nav_state_sub_;
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> reference_subscribers_;

    rclcpp::Service<farol2_pid_controller::srv::ChangeParams>::SharedPtr change_params_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr course_control_srv_;

    /** @brief Store latest navigation state sample. */
    void navStateCallback(const farol2_interfaces::msg::NavigationState &msg);

    /**
     * @brief Update reference value for one controller and timestamp it.
     * @param controller_name Controller key (e.g. "yaw", "surge").
     * @param raw_value Incoming reference value in message units.
     */
    void referenceCallback(const std::string &controller_name, double raw_value);

    /**
     * @brief Service callback to update yaw-controller gains online.
     */
    void changeParamsCallback(const std::shared_ptr<farol2_pid_controller::srv::ChangeParams::Request> request,
                              std::shared_ptr<farol2_pid_controller::srv::ChangeParams::Response> response);

    /**
     * @brief Service callback to select yaw-angle or course-angle control.
     */
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
      {"roll_rate", ROLL_RATE}
    };

        /** Last reference timestamp for each controller channel. */
    std::map<std::string, rclcpp::Time> controller_last_reference_;

        /** Tracks whether each controller has received at least one real reference. */
    std::map<std::string, bool> controller_has_reference_;

        /** True once the first NavigationState message has been received. */
    bool has_nav_state_ = false;

        /** Control-loop frequency in Hz. */
    double node_frequency_;

        /** Wrench accumulator updated by active controllers each cycle. */
    geometry_msgs::msg::WrenchStamped body_wrench_request_msg_;

        /** Shared node clock used for dt and timestamp checks. */
    rclcpp::Clock::SharedPtr clock_;

        /** Timestamp of previous timer callback, used to compute dt. */
    rclcpp::Time last_update_time_;

        /** Reused scalar message for force/torque publications. */
    std_msgs::msg::Float32 float32_msg_;

        /** Most recent navigation estimate. */
    farol2_interfaces::msg::NavigationState nav_state_;

        /** Enabled controller names after parsing/filtering configuration. */
    std::set<std::string> controller_names_;

        /** Optional allow-list from `controllers` parameter. */
    std::vector<std::string> configured_controllers_;

        /** Debug-enable flag per controller. */
    std::map<std::string, bool> controller_debug_;

        /** Flattened numeric parameters per controller. */
    std::map<std::string, std::map<std::string, double>> controller_parameters_;

        /** Runtime configuration table used by generic execution. */
    std::map<std::string, ControllerConfig> controller_configs_;

        /** Optional per-controller debug publishers. */
    std::map<std::string, rclcpp::Publisher<farol2_pid_controller::msg::PidDebug>::SharedPtr> debug_publishers_;

        /** Latest references for all channels (internally stored in SI units). */
    double surge_ref_ = 0.0, sway_ref_ = 0.0, heave_ref_ = 0.0,
           yaw_ref_ = 0.0, pitch_ref_ = 0.0, roll_ref_ = 0.0,
           yaw_rate_ref_ = 0.0, pitch_rate_ref_ = 0.0, roll_rate_ref_ = 0.0;

        /** Last controller output (used for debug publication). */
    double tau_;

        /** Selects yaw state source: heading (`false`) or course angle (`true`). */
    bool course_control_{false}; // flag to switch between heading or course control

        /** Low-pass filter configuration passed to PID controllers. */
    int lpf_order_;
    std::string lpf_method_, lpf_design_;

        /** Per-axis controller instances (allocated only when enabled). */
    std::unique_ptr<ControllerPI> controller_surge_;
    std::unique_ptr<ControllerPI> controller_sway_;
    std::unique_ptr<ControllerPI> controller_heave_;
    std::unique_ptr<ControllerPID> controller_yaw_;
    std::unique_ptr<ControllerPID> controller_pitch_;
    std::unique_ptr<ControllerPID> controller_roll_;
    std::unique_ptr<ControllerPI> controller_yaw_rate_;
    std::unique_ptr<ControllerPI> controller_pitch_rate_;
    std::unique_ptr<ControllerPI> controller_roll_rate_;

    /**
     * @brief Instantiate enabled controllers after parameter validation.
     */
    void createControllers();

    /**
     * @brief Validate the required numeric parameters for one controller.
     */
    bool validateControllerParams(const std::string &controller_name,
                    const std::vector<std::string> &required_params);

    /**
     * @brief Build controller configuration entries for the generic runtime path.
     */
    void initializeControllerConfigs();

    /**
     * @brief Execute one controller and accumulate its wrench contribution.
     */
    void executeController(const ControllerConfig &cfg, double dt);

    /**
     * @brief Check if the controller reference is still considered recent.
     */
    bool hasRecentReference(const rclcpp::Time &last_reference_timestamp, const int &node_frequency);

    /**
     * @brief Execute all enabled controllers that pass recent-reference gating.
     */
    void callControllers(double dt);

    /**
     * @brief Clear the accumulated wrench before/after each control cycle.
     */
    void resetBodyWrenchRequest();
};