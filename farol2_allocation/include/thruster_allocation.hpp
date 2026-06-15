#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "farol2_allocation/msg/thruster_force.hpp"
#include "farol2_interfaces/msg/navigation_state.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <farol2_utils/angles.hpp>

#define TOPIC_SUB_BODY_WRENCH_REQUEST "body_wrench_request"
#define TOPIC_SUB_NAV_STATE "nav_state"
#define TOPIC_SUB_MISSION_STATUS "mission_status"
#define TOPIC_PUB_THRUSTER_FORCE "thruster_force"
#define TOPIC_PUB_RUDDER_COMMAND "rudder_command"

class ThrusterAllocation : public rclcpp::Node {
  public:
    ThrusterAllocation();
    ~ThrusterAllocation();

    void loadParams();
    void initialiseSubscribers();
    void initialisePublishers();
    void initialiseTimers();

  private:
    enum class AllocationType {
      THRUST,
      THRUST_RUDDER
    };

    void initialiseAllocationFromTF();
    bool buildThrusterFramesFromTF();
    bool buildAllocationMatrixFromTF();
    AllocationType parseAllocationType(const std::string & allocation_type) const;

    void bodyWrenchRequestCallback(geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void computeRudderAngle(double tau_r);
    double solve_delta_from_tau(double tau_r, double gamma, double V);

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::TimerBase::SharedPtr tf_initialisation_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<farol2_allocation::msg::ThrusterForce>::SharedPtr thruster_force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr body_wrench_request_sub_;
    rclcpp::Subscription<farol2_interfaces::msg::NavigationState>::SharedPtr nav_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_status_sub_;

    farol2_allocation::msg::ThrusterForce thruster_force_msg_;
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

    Eigen::Matrix<double, 6, Eigen::Dynamic> thrust_allocation_matrix_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> thrust_allocation_matrix_pseudo_inv_;
    Eigen::Vector<double, 6> tau_;
    Eigen::Vector<double, 6> tau_common_mode_;
    Eigen::Vector<double, Eigen::Dynamic> forces_;

    double node_frequency_{10.0};

    double rudder_angle_{0.0};
    double rudder_x_body_drag_{0.0};
    double rudder_angle_max_{0.0};
    double rudder_angle_min_{0.0};
    double rudder_cm_distance_{0.0};
    double course_angle_{0.0};
    double sideslip_angle_{0.0};
    double flow_to_rudder_angle_{0.0};
    double V_s_angle_{0.0};
    Eigen::Vector2d V_cm_{0.0, 0.0};
    Eigen::Vector2d V_r_{0.0, 0.0};
    Eigen::Vector2d V_s_{0.0, 0.0};
    double K_s_{0.0};
    double K_L_{0.0};
    double K_D0_{0.0};
    double K_D1_{0.0};
    double L{0.0};
    double D{0.0};
    double gamma_{0.0};
};
