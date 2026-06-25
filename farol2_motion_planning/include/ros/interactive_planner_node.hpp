#ifndef INTERACTIVE_PLANNER_NODE_HPP
#define INTERACTIVE_PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "algorithms/multiple_vehicle_planner.hpp"
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <regex>

// Custom message headers
#include <farol2_motion_planning/msg/stamped_matrix.hpp>
#include <farol2_motion_planning/srv/set_state.hpp>
#include <farol2_motion_planning/srv/set_bounds_and_gains.hpp>
#include <farol2_motion_planning/srv/set_bezier_params.hpp>
#include <farol2_motion_planning/srv/set_obstacles.hpp>
#include <farol2_motion_planning/srv/run_optimization.hpp>
#include <farol2_motion_planning/srv/get_planner_config.hpp>

constexpr char TOPIC_PUB_ALIVE[] = "node_alive";
constexpr char TOPIC_PUB_LOG[] = "mission_log";

constexpr char TOPIC_SRV_SET_GOAL[] = "set_goal";
constexpr char TOPIC_SRV_SET_STATES[] = "set_states";
constexpr char TOPIC_SRV_SET_BOUNDS[] = "set_bounds";
constexpr char TOPIC_SRV_SET_BEZIER_PARAMS[] = "set_bezier_params";
constexpr char TOPIC_SRV_SET_OBSTACLES[] = "set_obstacles";
constexpr char TOPIC_SRV_RUN_OPTIMIZATION[] = "run_optimization";
constexpr char TOPIC_SRV_CANCEL_OPTIMIZATION[] = "cancel_optimization";
constexpr char TOPIC_SRV_GET_PLANNER_CONFIG[] = "get_planner_config";

struct BoundsAndGains
{
    double vel_min, vel_max;
    double acc_min, acc_max;
    double ang_vel_min, ang_vel_max;
    double ang_acc_min, ang_acc_max;
    double obs_min, obs_max;
    double radius;
    double alpha, beta, gamma;

    bool is_set = false;
};

void printControlPoints(const Eigen::Tensor<double, 3>& cp);

/**
 * @brief ROS2 interface node for the motion planning framework.
 *
 * This class manages communication between the motion planner and other
 * ROS2 nodes. It handles vehicle state subscriptions, trajectory publishing,
 * and service interfaces for configuring goals, bounds, and other planning
 * parameters.
 */
class InteractivePlannerNode : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    InteractivePlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~InteractivePlannerNode();

private:
    // Publishers
    std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> traj_pubs_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planning_alive_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_log_pub_;
    std::unordered_map<std::string, rclcpp::Publisher<farol2_motion_planning::msg::StampedMatrix>::SharedPtr> control_points_pubs_;
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> mission_pubs_;
    // Timer for alive status
    rclcpp::TimerBase::SharedPtr alive_timer_;

    // Synchronization
    std::atomic<bool> cancel_flag_{false};
    std::mutex state_mutex_;

    // Vehicle tracking
    std::vector<std::string> vehicle_names_;
    std::vector<std::string> selected_vehicle_names_;
    std::map<std::string, vehicle_state> vehicle_states_;
    std::map<std::string, vehicle_state> goal_states_map_;
    std::set<std::string> vehicle_names_set_;

    // Obstacles
    Eigen::Matrix<double, 3, Eigen::Dynamic> circ_obs_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> line_obs_;
    bool obstacles_updated_;

    // Planning parameters
    int bezier_degree_;
    int guess_degree_;
    std::vector<int64_t> nSplit_;
    std::vector<bool> constr_flags_;
    int number_sample_Pts_;
    BoundsAndGains bounds_;

    // Optimization thread management
    std::thread opt_thread_;
    std::atomic<bool> opt_running_{false};

    // Results
    Eigen::Tensor<double, 3> last_control_points_;
    double last_Tf_;
    double RADIUS = 1.5;

    // ROS2 Services
    rclcpp::Service<farol2_motion_planning::srv::SetState>::SharedPtr set_goal_srv_;
    rclcpp::Service<farol2_motion_planning::srv::SetState>::SharedPtr set_states_srv_;
    rclcpp::Service<farol2_motion_planning::srv::RunOptimization>::SharedPtr run_optimization_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_optimization_srv_;
    rclcpp::Service<farol2_motion_planning::srv::SetBoundsAndGains>::SharedPtr set_bounds_srv_;
    rclcpp::Service<farol2_motion_planning::srv::SetBezierParams>::SharedPtr set_bezier_params_srv_;
    rclcpp::Service<farol2_motion_planning::srv::GetPlannerConfig>::SharedPtr get_planner_config_srv_;
    rclcpp::Service<farol2_motion_planning::srv::SetObstacles>::SharedPtr set_obstacles_srv_;

    // Load parameters from config file
    void LoadParameters();
    void initialisePublishers();
    void initialiseServices();

    // Service callbacks
    void setGoalService(const std::shared_ptr<farol2_motion_planning::srv::SetState::Request> req,
                       std::shared_ptr<farol2_motion_planning::srv::SetState::Response> res);
    void setStatesService(const std::shared_ptr<farol2_motion_planning::srv::SetState::Request> req,
                          std::shared_ptr<farol2_motion_planning::srv::SetState::Response> res);
    void runOptimizationService(const std::shared_ptr<farol2_motion_planning::srv::RunOptimization::Request> req,
                                std::shared_ptr<farol2_motion_planning::srv::RunOptimization::Response> res);
    void setObstaclesService(const std::shared_ptr<farol2_motion_planning::srv::SetObstacles::Request> req,
                             std::shared_ptr<farol2_motion_planning::srv::SetObstacles::Response> res);
    void cancelOptimizationService(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void setBoundsService(const std::shared_ptr<farol2_motion_planning::srv::SetBoundsAndGains::Request> req,
                          std::shared_ptr<farol2_motion_planning::srv::SetBoundsAndGains::Response> res);
    void setBezierParamsService(const std::shared_ptr<farol2_motion_planning::srv::SetBezierParams::Request> req,
                                std::shared_ptr<farol2_motion_planning::srv::SetBezierParams::Response> res);
    void getPlannerConfigService(const std::shared_ptr<farol2_motion_planning::srv::GetPlannerConfig::Request> req,
                                 std::shared_ptr<farol2_motion_planning::srv::GetPlannerConfig::Response> res);

    // Helper methods
    void processState(const vehicle_state msg, const std::string& vehicle_name);
    void publishMissions();
    std::vector<int> selectTrajectoriesToRemove(int NVehicles, const std::vector<std::pair<int,int>>& collisions, const std::vector<double>& Tf_values);
    std::string formatMissionString(const Eigen::Tensor<double, 3> &controlPoints, int vehicleIndex, double Tf_opt);
    void publishLog(const std::string& text);
    void publishControlPoints(const Eigen::Tensor<double, 3>& tensor, const std::vector<double>& tf_values, bool is_guess);
    void publishAliveStatus();
};

#endif
