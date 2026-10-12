#ifndef DISTRIBUTED_PLANNER_NODE_HPP
#define DISTRIBUTED_PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "algorithms/multiple_vehicle_planner.hpp"
#include "underway/distributed_utils.hpp"
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <regex>

// Custom message headers
#include <farol2_motion_planning/msg/stamped_matrix.hpp>
#include <farol2_motion_planning/msg/neighbor_position.hpp>
#include <farol2_motion_planning/srv/set_state.hpp>
#include <farol2_motion_planning/srv/set_bounds_and_gains.hpp>
#include <farol2_motion_planning/srv/set_bezier_params.hpp>
#include <farol2_motion_planning/srv/set_obstacles.hpp>
#include <farol2_motion_planning/srv/run_optimization.hpp>
#include <farol2_motion_planning/srv/get_planner_config.hpp>

constexpr char TOPIC_PUB_ALIVE[] = "node_alive";
constexpr char TOPIC_PUB_LOG[] = "mission_log";
constexpr char TOPIC_PUB_STATE[] = "current_state";

constexpr char TOPIC_SUB_STATE[] = "neighbor_state";

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

bool isClose(const vehicle_state& a, const vehicle_state& b, double pos_tol, double theta_tol, double v_tol)
{
    return std::abs(a.x - b.x) <= pos_tol &&
           std::abs(a.y - b.y) <= pos_tol &&
           std::abs(a.theta - b.theta) <= theta_tol &&
           std::abs(a.v - b.v) <= v_tol;
}

void printControlPoints(const Eigen::Tensor<double, 3>& cp);

/**
 * @brief ROS2 interface node for the motion planning framework.
 *
 * This class manages communication between the motion planner and other
 * ROS2 nodes. It handles vehicle state subscriptions, trajectory publishing,
 * and service interfaces for configuring goals, bounds, and other planning
 * parameters.
 */
class DistributedPlannerNode : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DistributedPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~DistributedPlannerNode();

private:
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planning_alive_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_log_pub_;

    // Timer for alive status
    rclcpp::TimerBase::SharedPtr alive_timer_;

    // Synchronization
    std::atomic<bool> cancel_flag_{false};
    std::mutex state_mutex_;

    // Vehicle tracking
    std::vector<std::string> selected_vehicle_names_;
    std::map<std::string, vehicle_state> vehicle_states_;

    // Obstacles
    Eigen::Matrix<double, 3, Eigen::Dynamic> circ_obs_;
    Eigen::Matrix<double, 5, Eigen::Dynamic> elip_obs_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> line_obs_;
    bool obstacles_updated_;

    // Planning parameters
    int bezier_degree_;
    int guess_degree_;
    std::vector<int64_t> nSplit_;
    std::vector<bool> constr_flags_;
    int number_sample_Pts_;
    BoundsAndGains bounds_;
    bool use_collision_method_{false};

    // Optimization thread management
    std::thread opt_thread_;
    std::atomic<bool> opt_running_{false};

    // Results
    Eigen::Tensor<double, 3> last_control_points_;
    double last_Tf_;
    double RADIUS = 1.5;

    
    
    // ###################################################################################### //
    // ############################ Distributed scheme variables ############################ //
    // ###################################################################################### //
    
    //Subscribers
    rclcpp::Subscription<farol2_motion_planning::msg::NeighborPosition>::SharedPtr neighbor_sub_;

    //Publishers
    rclcpp::Publisher<farol2_motion_planning::msg::NeighborPosition>::SharedPtr state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;
    rclcpp::Publisher<farol2_motion_planning::msg::StampedMatrix>::SharedPtr control_points_pub_;



    // Variables
    bool distributed_scheme_running_{false};
    int current_iteration_{0};
    bool use_goal_straight_line_;
    bool use_goal_closer_point_;
    double cell_margin_; // m

    std::string vehicle_name_;
    vehicle_state initial_state_;
    vehicle_state goal_state_;
    std::vector<vehicle_state> states_history_;
    std::vector<VoronoiCell> cell_history_;

    // Relative to multiple vehicle stuff...
    std::vector<std::string> known_vehicle_names_;
    std::map<uint32_t, std::map<std::string, Eigen::Vector2d>> neighbors_history_;

    // Global definitions
    double pos_tol_{0.1};    // m
    double theta_tol_{0.02}; // rad
    double v_tol_{0.01};     // m/s
    

    // ###################################################################################### //
    // ###################################################################################### //
    // ###################################################################################### //
    
    
    
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
    void initialiseSubscribers();

    // Subscriber callbacks
    void neighborCallback(const farol2_motion_planning::msg::NeighborPosition::ConstSharedPtr msg);

    // Service callbacks
    void setGoalService(const std::shared_ptr<farol2_motion_planning::srv::SetState::Request> req,
                       std::shared_ptr<farol2_motion_planning::srv::SetState::Response> res);
    void setStatesService(const std::shared_ptr<farol2_motion_planning::srv::SetState::Request> req,
                          std::shared_ptr<farol2_motion_planning::srv::SetState::Response> res);
    void setObstaclesService(const std::shared_ptr<farol2_motion_planning::srv::SetObstacles::Request> req,
                             std::shared_ptr<farol2_motion_planning::srv::SetObstacles::Response> res);
    void setBoundsService(const std::shared_ptr<farol2_motion_planning::srv::SetBoundsAndGains::Request> req,
                          std::shared_ptr<farol2_motion_planning::srv::SetBoundsAndGains::Response> res);
    void setBezierParamsService(const std::shared_ptr<farol2_motion_planning::srv::SetBezierParams::Request> req,
                                std::shared_ptr<farol2_motion_planning::srv::SetBezierParams::Response> res);
    void getPlannerConfigService(const std::shared_ptr<farol2_motion_planning::srv::GetPlannerConfig::Request> req,
                                 std::shared_ptr<farol2_motion_planning::srv::GetPlannerConfig::Response> res);
    
    void runDistributedSchemeIteration();

    // Helper methods
    void processState(const vehicle_state msg, const std::string& vehicle_name);
    void publishMissions();
    std::string formatMissionString(const Eigen::Tensor<double, 3> &controlPoints, double Tf_opt);
    void publishLog(const std::string& text);
    void publishControlPoints(const Eigen::Tensor<double, 3>& tensor, const std::vector<double>& tf_values, bool is_guess);
    void publishAliveStatus();
};

#endif


//########################################################################//
//##################### Usage of neighbor_history_ #######################//
//########################################################################//

// Neighbor positions received by the subscriber, excluding this vehicle.
// neighbors_history_[iteration][vehicle_name] = position (x, y)
//
// An entry for an iteration is created when the iteration starts, so an
// iteration where no neighbor was heard exists with an empty inner map.
// Only the current iteration is filled. Access requires state_mutex_.
//
// Number of iterations stored:
//     neighbors_history_.size()
//
// Names of the vehicles heard at iteration k:
//     for (const auto& [name, position] : neighbors_history_.at(k))
//     {
//         names.push_back(name);
//     }
//
// Positions of the vehicles heard at iteration k (same order as the names):
//     for (const auto& [name, position] : neighbors_history_.at(k))
//     {
//         positions.push_back(position);
//     }
//
// Number of vehicles heard at iteration k:
//     neighbors_history_.at(k).size()
//
// Position of vehicle "name" at iteration k, if it was heard:
//     const auto& row = neighbors_history_.at(k);
//     const auto it = row.find(name);
//     if (it != row.end()) { it->second }
//
// Iterations are visited in ascending order:
//     for (const auto& [iteration, row] : neighbors_history_)