#ifndef DIST_MOTION_PLANNING_CPP_NODE_H
#define DIST_MOTION_PLANNING_CPP_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <farol_msgs/mState.h>
#include <your_msgs/FutureState.h>  // <-- Replace with your actual message
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <tf/tf.h>

#include "BezierOptimization.h"
#include <unordered_map>
#include <vector>
#include <string>

class DistMotionPlanningCppNode {
public:
    DistMotionPlanningCppNode(ros::NodeHandle* nh, ros::NodeHandle* nh_private);
    ~DistMotionPlanningCppNode();

private:
    // ROS
    ros::NodeHandle nh_, nh_private_;
    ros::Subscriber state_sub_;
    ros::Subscriber start_sub_;
    std::vector<ros::Subscriber> future_state_subs_;
    ros::Publisher mission_pub_;
    ros::Publisher future_state_pub_;
    ros::Timer computation_timer_;

    // Config & Parameters
    std::string self_name_;
    int expected_vehicles_;
    double p_node_frequency_;

    // State & Timing
    bool planning_started_ = false;
    bool initial_trajectory_computed_ = false;
    bool state_received_ = false;

    State current_state_;
    double current_velocity_ = 0.0;
    std::unordered_map<std::string, State> future_states_;

    // Core Planning Functions
    void loadParams();
    void initializeSubscribers();
    void initializePublishers();

    void startCallback(const std_msgs::Empty::ConstPtr& msg);
    void vehicleCallback(const farol_msgs::mState::ConstPtr& msg);
    void futureStateCallback(const your_msgs::FutureState::ConstPtr& msg);

    void computeInitialTrajectory();
    void computeNextTrajectory();  // Optional separate handler
    void computationLoop(const ros::TimerEvent&);

    std::string formatMissionString(const Eigen::Tensor<double, 3>& controlPoints, int vehicleIndex, double Tf_opt);
};

#endif
