#ifndef INTERACTIVE_MOTION_PLANNING_NODE_H
#define INTERACTIVE_MOTION_PLANNING_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <farol_msgs/mState.h>
#include <tf/tf.h>
#include "MultipleVehicleMotionPlan.h"
#include <ros/master.h>
#include <regex>
#include <std_srvs/Trigger.h>
#include <motion_planning_cpp/SetState.h>
#include <motion_planning_cpp/SetBoundsandGains.h>
#include <motion_planning_cpp/SetBezierParams.h>
#include <motion_planning_cpp/SetObstacles.h>
#include <motion_planning_cpp/RunOptimization.h>
#include <motion_planning_cpp/GetPlannerConfig.h>
#include "motion_planning_cpp/StampedMatrix.h"



#include <memory>
#include <mutex>
#include <atomic>
#include <thread>




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
 * @brief ROS interface node for the motion planning framework.
 *
 * This class manages communication between the motion planner and other
 * ROS nodes. It handles vehicle state subscriptions, trajectory publishing,
 * and service interfaces for configuring goals, bounds, and other planning
 * parameters.
 */
class InteractiveMotionPlanningNode {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    InteractiveMotionPlanningNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);
    ~InteractiveMotionPlanningNode();

private:
    ros::NodeHandle nh_, nh_private_;
    std::unordered_map<std::string, ros::Publisher> traj_pubs_;
    ros::Publisher planning_alive_pub_;
    ros::Publisher mission_log_pub_;
    std::unordered_map<std::string, ros::Publisher> control_points_pubs_;

    ros::Timer alive_timer_;

    std::atomic<bool> cancel_flag_{false};
    std::mutex state_mutex_;

    std::vector<std::string> vehicle_names_;
    std::vector<std::string> selected_vehicle_names_;
    std::map<std::string, ros::Subscriber> subscribers_;
    std::map<std::string, ros::Publisher> publishers_;
    std::map<std::string, vehicle_State> vehicle_states_;
    std::map<std::string, bool> received_flags_;
    std::string reference_vehicle_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> circ_obs_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> line_obs_;
    bool obstacles_updated_;
    int number_sample_Pts_;

    std::set<std::string> vehicle_names_set_;

    double p_node_frequency_;

    int bezier_degree_;
    int guess_degree_;
    std::vector<int> nSplit_;
    std::vector<uint8_t> constr_flags_;

    BoundsAndGains bounds_;

    std::thread opt_thread_;
    std::atomic<bool> opt_running_{false};
    std::map<std::string, vehicle_State> goal_states_map_;

    Eigen::Tensor<double, 3> last_control_points_;
    double last_Tf_;

    // ROS services
    ros::ServiceServer set_goal_srv_;
    ros::ServiceServer set_states_srv_;
    ros::ServiceServer run_optimization_srv_;
    ros::ServiceServer cancel_optimization_srv_;
    ros::ServiceServer set_bounds_srv_;
    ros::ServiceServer set_bezier_params_srv_;
    ros::ServiceServer get_planner_config_srv_;
    ros::ServiceServer set_obstacles_srv_;

    // Service callbacks (MANDATORY)
    bool setGoalService(motion_planning_cpp::SetState::Request &req, motion_planning_cpp::SetState::Response &res);
    bool setStatesService(motion_planning_cpp::SetState::Request &req, motion_planning_cpp::SetState::Response &res);
    bool runOptimizationService(motion_planning_cpp::RunOptimization::Request &req, motion_planning_cpp::RunOptimization::Response &res);
    // Service callbacks (OPTIONAL)
    bool setObstaclesService(motion_planning_cpp::SetObstacles::Request &req, motion_planning_cpp::SetObstacles::Response &res);
    bool cancelOptimizationService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool setBoundsService(motion_planning_cpp::SetBoundsandGains::Request &req, motion_planning_cpp::SetBoundsandGains::Response &res);
    bool setBezierParamsService(motion_planning_cpp::SetBezierParams::Request &req, motion_planning_cpp::SetBezierParams::Response &res);
    bool getPlannerConfigService(motion_planning_cpp::GetPlannerConfig::Request &req, motion_planning_cpp::GetPlannerConfig::Response &res);
    
    // Other definitions
    void processState(const vehicle_State msg, const std::string& vehicle_name);
    void publishMissions();
    std::vector<int> selectTrajectoriesToRemove(int NVehicles, const std::vector<std::pair<int,int>>& collisions, const std::vector<double>& Tf_values);
    std::string formatMissionString(const Eigen::Tensor<double, 3> &controlPoints, int vehicleIndex, double Tf_opt);
    double RADIUS = 1.5;
    void publishLog(const std::string& text);
    void publishControlPoints(const Eigen::Tensor<double, 3>& tensor, const std::vector<double>& tf_values, bool is_guess);
};

#endif
