#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <optional>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "farol2_interfaces/msg/control_surface_angle.hpp"
#include "farol2_interfaces/msg/navigation_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <rudder_allocator.hpp>
#include <static_thruster_allocator.hpp>
#include <thruster_rpm_converter.hpp>

static constexpr char TOPIC_SUB_THRUST_X[] = "thrust_x";
static constexpr char TOPIC_SUB_THRUST_Y[] = "thrust_y";
static constexpr char TOPIC_SUB_THRUST_Z[] = "thrust_z";
static constexpr char TOPIC_SUB_TORQUE_X[] = "torque_x";
static constexpr char TOPIC_SUB_TORQUE_Y[] = "torque_y";
static constexpr char TOPIC_SUB_TORQUE_Z[] = "torque_z";
static constexpr char TOPIC_SUB_NAV_STATE[] = "nav_state";
static constexpr char TOPIC_SUB_MISSION_STATUS[] = "mission_status";
static constexpr char TOPIC_PUB_CONTROL_SURFACE_ANGLE[] = "control_surface_angle";

class AllocationNode : public rclcpp::Node {
  public:
    /**
     * @brief Create the allocation node and initialise its ROS interfaces.
     */
    AllocationNode();

    /**
     * @brief Destroy the allocation node.
     */
    ~AllocationNode();

  private:
    enum class AllocationType {
      THRUST,
      THRUST_RUDDER
    };

    /**
     * @brief Load and validate allocation and actuator parameters.
     */
    void loadParams();

    /**
     * @brief Create subscriptions for wrench components and vehicle state.
     */
    void initialiseSubscribers();

    /**
     * @brief Create actuator command publishers required by the selected mode.
     */
    void initialisePublishers();

    /**
     * @brief Start the TF initialisation and allocation timers.
     */
    void initialiseTimers();

    /**
     * @brief Build the thruster allocator once all required TF frames are available.
     */
    void initialiseAllocationFromTF();

    /**
     * @brief Combine fresh wrench components and process one allocation cycle.
     */
    void allocationTimerCallback();

    /**
     * @brief Convert an allocation type parameter to its internal representation.
     */
    AllocationType parseAllocationType(const std::string & allocation_type) const;

    /**
     * @brief Allocate a body wrench and publish the resulting actuator commands.
     */
    void processBodyWrenchRequest(const geometry_msgs::msg::WrenchStamped & msg);

    rclcpp::Clock::SharedPtr clock_;  ///< Clock used for timestamps and freshness checks.
    rclcpp::TimerBase::SharedPtr tf_initialisation_timer_;  ///< Retries allocation setup from TF.
    rclcpp::TimerBase::SharedPtr allocation_timer_;  ///< Triggers periodic wrench allocation.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;  ///< Stores transforms used for thruster geometry.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  ///< Populates the TF buffer.

    rclcpp::Publisher<farol2_interfaces::msg::ThrusterRPM>::SharedPtr rpm_command_pub_;  ///< Publishes thruster RPM commands.
    rclcpp::Publisher<farol2_interfaces::msg::ControlSurfaceAngle>::SharedPtr control_surface_angle_pub_;  ///< Publishes rudder commands.
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thrust_x_sub_;  ///< Receives surge force requests.
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thrust_y_sub_;  ///< Receives sway force requests.
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thrust_z_sub_;  ///< Receives heave force requests.
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_x_sub_;  ///< Receives roll torque requests.
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_y_sub_;  ///< Receives pitch torque requests.
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_z_sub_;  ///< Receives yaw torque requests.
    rclcpp::Subscription<farol2_interfaces::msg::NavigationState>::SharedPtr nav_state_sub_;  ///< Receives vehicle state feedback.
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_status_sub_;  ///< Receives mission activation state.

    std::unique_ptr<StaticThrusterAllocator> static_thruster_allocator_;  ///< Maps body wrenches to thruster forces.
    std::unique_ptr<RudderAllocator> rudder_allocator_;  ///< Maps yaw torque to rudder deflection.
    std::unique_ptr<ThrusterRpmConverter> rpm_converter_;  ///< Converts thruster forces to RPM.

    farol2_interfaces::msg::ControlSurfaceAngle control_surface_angle_msg_;  ///< Reused rudder command message.
    farol2_interfaces::msg::NavigationState nav_state_;  ///< Latest navigation state.

    AllocationType allocation_type_{AllocationType::THRUST};  ///< Selected actuator allocation mode.
    bool allocation_ready_{false};  ///< Whether TF-based allocation is initialised.
    bool open_loop_{false};  ///< Whether RPM publication is disabled in rudder mode.
    int mission_status_{0};  ///< Latest mission activation status.

    std::string base_frame_;  ///< Body frame used for thruster geometry.
    std::string frame_prefix_;  ///< Vehicle namespace prepended to TF frames.
    std::vector<std::string> thruster_frames_;  ///< TF frames of the configured thrusters.
    size_t nr_thrusters_{0};  ///< Number of configured thrusters.
    Eigen::Vector<double, 6> tau_;  ///< Requested body wrench.
    Eigen::Vector<double, 6> tau_common_mode_;  ///< Wrench assigned to thrusters in rudder mode.
    Eigen::Vector<double, Eigen::Dynamic> forces_;  ///< Allocated force for each thruster.
    std::array<double, 6> wrench_input_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  ///< Latest independently received wrench components.
    std::array<std::optional<rclcpp::Time>, 6> last_received_;  ///< Reception time of each wrench component.

    double node_frequency_{10.0};  ///< Allocation update frequency in hertz.
    double rudder_angle_{0.0};  ///< Latest allocated rudder angle in radians.
    double rudder_x_body_drag_{0.0};  ///< Surge drag produced by the rudder.
    double rudder_angle_max_{0.0};  ///< Maximum rudder angle in radians.
    double rudder_angle_min_{0.0};  ///< Minimum rudder angle in radians.
    double rudder_cm_distance_{0.0};  ///< Distance from the vehicle centre to the rudder.
    double K_s_{0.0};  ///< Static rudder torque coefficient.
    double K_L_{0.0};  ///< Rudder lift coefficient.
    double K_D0_{0.0};  ///< Constant rudder drag coefficient.
    double K_D1_{0.0};  ///< Quadratic rudder drag coefficient.
};
