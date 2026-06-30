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
static constexpr char TOPIC_PUB_RUDDER_COMMAND[] = "rudder_command";

class AllocationNode : public rclcpp::Node {
  public:
    AllocationNode();
    ~AllocationNode();

  private:
    enum class AllocationType {
      THRUST,
      THRUST_RUDDER
    };

    void loadParams();
    void initialiseSubscribers();
    void initialisePublishers();
    void initialiseTimers();
    void initialiseAllocationFromTF();
    void allocationTimerCallback();
    AllocationType parseAllocationType(const std::string & allocation_type) const;
    void processBodyWrenchRequest(const geometry_msgs::msg::WrenchStamped & msg);

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::TimerBase::SharedPtr tf_initialisation_timer_;
    rclcpp::TimerBase::SharedPtr allocation_timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<farol2_interfaces::msg::ThrusterRPM>::SharedPtr rpm_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_command_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thrust_x_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thrust_y_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thrust_z_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_x_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_y_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_z_sub_;
    rclcpp::Subscription<farol2_interfaces::msg::NavigationState>::SharedPtr nav_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_status_sub_;

    std::unique_ptr<StaticThrusterAllocator> static_thruster_allocator_;
    std::unique_ptr<RudderAllocator> rudder_allocator_;
    std::unique_ptr<ThrusterRpmConverter> rpm_converter_;

    std_msgs::msg::Float32 rudder_command_msg_;
    farol2_interfaces::msg::NavigationState nav_state_;

    AllocationType allocation_type_{AllocationType::THRUST};
    bool allocation_ready_{false};
    bool open_loop_{false};
    int mission_status_{0};

    std::string base_frame_;
    std::string frame_prefix_;
    Eigen::Vector3d thrust_axis_{1.0, 0.0, 0.0};
    std::vector<std::string> thruster_frames_;
    size_t nr_thrusters_{0};
    Eigen::Vector<double, 6> tau_;
    Eigen::Vector<double, 6> tau_common_mode_;
    Eigen::Vector<double, Eigen::Dynamic> forces_;
    std::array<double, 6> wrench_input_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<std::optional<rclcpp::Time>, 6> last_received_;

    double node_frequency_{10.0};
    double rudder_angle_{0.0};
    double rudder_x_body_drag_{0.0};
    double rudder_angle_max_{0.0};
    double rudder_angle_min_{0.0};
    double rudder_cm_distance_{0.0};
    double K_s_{0.0};
    double K_L_{0.0};
    double K_D0_{0.0};
    double K_D1_{0.0};
};