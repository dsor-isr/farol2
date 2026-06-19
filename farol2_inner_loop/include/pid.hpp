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
#include "farol2_inner_loop/srv/change_params.hpp"
#include "farol2_inner_loop/msg/pid_debug.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <farol2_utils/filters/low_pass_filter.hpp>
#include <farol2_utils/angles.hpp>

#include "farol2_inner_loop/controller_pid.hpp"
#include "farol2_inner_loop/reference_generator.hpp"

// Topic/service names (short form, remapped in launch file)
static constexpr char TOPIC_SUB_NAV_STATE[] = "nav_state";
static constexpr char TOPIC_SUB_SIM_STATE[] = "body_velocity";
static constexpr char TOPIC_SUB_SURGE_REF[] = "surge_ref";
static constexpr char TOPIC_SUB_SWAY_REF[] = "sway_ref";
static constexpr char TOPIC_SUB_HEAVE_REF[] = "heave_ref";
static constexpr char TOPIC_SUB_DEPTH_REF[] = "depth_ref";
static constexpr char TOPIC_SUB_ALTITUDE_REF[] = "altitude_ref";
static constexpr char TOPIC_SUB_YAW_REF[] = "yaw_ref";
static constexpr char TOPIC_SUB_PITCH_REF[] = "pitch_ref";
static constexpr char TOPIC_SUB_ROLL_REF[] = "roll_ref";
static constexpr char TOPIC_SUB_YAW_RATE_REF[] = "yaw_rate_ref";
static constexpr char TOPIC_SUB_PITCH_RATE_REF[] = "pitch_rate_ref";
static constexpr char TOPIC_SUB_ROLL_RATE_REF[] = "roll_rate_ref";
static constexpr char TOPIC_PUB_THRUST_X[] = "thrust_x";
static constexpr char TOPIC_PUB_THRUST_Y[] = "thrust_y";
static constexpr char TOPIC_PUB_THRUST_Z[] = "thrust_z";
static constexpr char TOPIC_PUB_TORQUE_X[] = "torque_x";
static constexpr char TOPIC_PUB_TORQUE_Y[] = "torque_y";
static constexpr char TOPIC_PUB_TORQUE_Z[] = "torque_z";
static constexpr char TOPIC_PUB_DEBUG_SURGE[] = "debug_surge";
static constexpr char TOPIC_PUB_DEBUG_SWAY[] = "debug_sway";
static constexpr char TOPIC_PUB_DEBUG_HEAVE[] = "debug_heave";
static constexpr char TOPIC_PUB_DEBUG_DEPTH[] = "debug_depth";
static constexpr char TOPIC_PUB_DEBUG_ALTITUDE[] = "debug_altitude";
static constexpr char TOPIC_PUB_DEBUG_YAW[] = "debug_yaw";
static constexpr char TOPIC_PUB_DEBUG_PITCH[] = "debug_pitch";
static constexpr char TOPIC_PUB_DEBUG_ROLL[] = "debug_roll";
static constexpr char TOPIC_PUB_DEBUG_YAW_RATE[] = "debug_yaw_rate";
static constexpr char TOPIC_PUB_DEBUG_PITCH_RATE[] = "debug_pitch_rate";
static constexpr char TOPIC_PUB_DEBUG_ROLL_RATE[] = "debug_roll_rate";
static constexpr char SERVICE_CHANGE_PARAMS[] = "change_params";
static constexpr char SERVICE_COURSE_CONTROL[] = "course_instead_of_yaw";
enum ControllerType {
  SURGE = 0,
  SWAY = 1,
  HEAVE = 2,
  DEPTH = 3,
  ALTITUDE = 4,
  YAW = 5,
  PITCH = 6,
  ROLL = 7,
  YAW_RATE = 8,
  PITCH_RATE = 9,
  ROLL_RATE = 10,
};

// Use the reusable controller classes from farol_control namespace
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
  /** Returns the current measured state for this channel. */
  std::function<double()> get_state;
  /** Returns the latest reference for this channel. */
  std::function<double()> get_ref;
  /** Returns the measured state rate (used by PID channels). */
  std::function<double()> get_rate;
  /** Adds this channel's output to the wrench accumulator. */
  std::function<void(double)> accumulate_output;
  /** Fills debug message fields from controller internals. */
    std::function<void(farol2_inner_loop::msg::PidDebug &)> fill_debug;
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
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sim_state_sub_;
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> reference_subscribers_;

    rclcpp::Service<farol2_inner_loop::srv::ChangeParams>::SharedPtr change_params_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr course_instead_of_yaw_srv_;

    std::map<std::string, ControllerPID*> controller_ptrs_;

    /** @brief Store latest navigation state sample. */
    void navStateCallback(const farol2_interfaces::msg::NavigationState &msg);
    void simStateCallback(const geometry_msgs::msg::Vector3 &msg);

    /**
     * @brief Update reference value for one controller and timestamp it.
     * @param controller_name Controller key (e.g. "yaw", "surge").
     * @param raw_value Incoming reference value in message units.
     */
    void referenceCallback(const std::string &controller_name, double raw_value);

    /**
     * @brief Service callback to update yaw-controller gains online.
     */
    void changeParamsCallback(const std::shared_ptr<farol2_inner_loop::srv::ChangeParams::Request> request,
                              std::shared_ptr<farol2_inner_loop::srv::ChangeParams::Response> response);

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
      {"depth", DEPTH},
      {"altitude", ALTITUDE},
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

        /** Most recent simulation state estimate. */
    geometry_msgs::msg::Vector3 body_velocity_;

        /** Enabled controller names after parsing/filtering configuration. */
    std::set<std::string> controller_names_;

        /** Optional allow-list from `controllers` parameter. */
    std::vector<std::string> configured_controllers_;

        /** Debug-enable flag per controller. */
    std::map<std::string, bool> controller_debug_;

        /** Flattened numeric parameters per controller. */
    std::map<std::string, std::map<std::string, double>> controller_parameters_;

        /** Reference preprocessing block per controller channel. */
    std::map<std::string, std::unique_ptr<farol_control::ReferenceGenerator>> reference_generators_;

        /** Latest preprocessed reference signals per controller channel. */
    std::map<std::string, farol_control::ReferenceGeneratorOutput> reference_outputs_;

        /** Runtime configuration table used by generic execution. */
    std::map<std::string, ControllerConfig> controller_configs_;

        /** Optional per-controller debug publishers. */
    std::map<std::string, rclcpp::Publisher<farol2_inner_loop::msg::PidDebug>::SharedPtr> debug_publishers_;

        /** Latest references for all channels (internally stored in SI units). */
    double surge_ref_ = 0.0, sway_ref_ = 0.0, heave_ref_ = 0.0,
           depth_ref_ = 0.0, altitude_ref_ = 0.0,
           yaw_ref_ = 0.0, pitch_ref_ = 0.0, roll_ref_ = 0.0,
           yaw_rate_ref_ = 0.0, pitch_rate_ref_ = 0.0, roll_rate_ref_ = 0.0;

        /** Last controller output (used for debug publication). */
    double tau_;

        /** Selects yaw state source: heading (`false`) or course angle (`true`). */
    bool course_instead_of_yaw_{false}; // flag to switch between heading or course control
    bool body_vel_instead_of_nav_{false}; // flag to switch between body velocity or navigation state velocity
    bool use_heading_rate_as_yaw_rate_{false};

        /** Low-pass filter configuration passed to PID controllers. */
    int lpf_order_;
    std::string lpf_method_, lpf_design_;

        /** Per-axis controller instances (allocated only when enabled). */
    std::unique_ptr<ControllerPID> controller_surge_;
    std::unique_ptr<ControllerPID> controller_sway_;
    std::unique_ptr<ControllerPID> controller_heave_;
    std::unique_ptr<ControllerPID> controller_depth_;
    std::unique_ptr<ControllerPID> controller_altitude_;
    std::unique_ptr<ControllerPID> controller_yaw_;
    std::unique_ptr<ControllerPID> controller_pitch_;
    std::unique_ptr<ControllerPID> controller_roll_;
    std::unique_ptr<ControllerPID> controller_yaw_rate_;
    std::unique_ptr<ControllerPID> controller_pitch_rate_;
    std::unique_ptr<ControllerPID> controller_roll_rate_;

    /**
     * @brief Instantiate enabled controllers after parameter validation.
     */
    void createControllers();

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
