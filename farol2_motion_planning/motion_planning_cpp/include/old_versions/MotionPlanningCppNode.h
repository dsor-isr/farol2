#ifndef MOTION_PLANNING_CPP_NODE_H
#define MOTION_PLANNING_CPP_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <farol_msgs/mState.h>  // Import vehicle state message
#include <tf/tf.h>
#include "BezierOptimization.h"
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <ros/master.h>
#include <regex>
#include "Hungarian.h" // or implement your own



class MotionPlanningCppNode {
public:
    MotionPlanningCppNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);
    ~MotionPlanningCppNode();

private:
    ros::NodeHandle nh_, nh_private_;
    ros::Subscriber sub_mred_, sub_mblack_;
    ros::Publisher pub_mred_, pub_mblack_;

    std::vector<std::string> vehicle_names_;
    std::map<std::string, ros::Subscriber> subscribers_;
    std::map<std::string, ros::Publisher> publishers_;
    std::map<std::string, State> vehicle_states_;
    std::map<std::string, double> vehicle_velocities_;
    std::map<std::string, bool> received_flags_;
    std::string reference_vehicle_;

    double p_node_frequency_;

    State mred_state_;
    State mblack_state_;
    double mred_v0_;
    double mblack_v0_;
    bool mred_received_, mblack_received_;

    void loadParams();
    void initializeSubscribers();
    void initializePublishers();

    void mredCallback(const farol_msgs::mState::ConstPtr &msg);
    void mblackCallback(const farol_msgs::mState::ConstPtr &msg);
    void vehicleCallback(const farol_msgs::mState::ConstPtr &msg, const std::string& vehicle_name);
    std::string formatMissionString(const Eigen::Tensor<double, 3> &controlPoints, int vehicleIndex, double Tf_opt);
    void processAndPublish();
};

#endif
