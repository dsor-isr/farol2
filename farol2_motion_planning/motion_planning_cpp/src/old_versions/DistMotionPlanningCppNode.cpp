#include "DistMotionPlanningCppNode.h"

// @.@ Constructor
DistMotionPlanningCppNode::DistMotionPlanningCppNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private)
    : nh_(*nodehandle), nh_private_(*nodehandle_private), planning_started_(false) {

    loadParams();
    initializeSubscribers();
    initializePublishers();

    future_state_pub_ = nh_.advertise<your_msgs::FutureState>("future_state", 10);
    start_sub_ = nh_.subscribe("start_planning", 1, &MotionPlanningCppNode::startCallback, this);

    // Optional: initialize timer but don't start yet
    compute_timer_ = nh_.createTimer(ros::Duration(0.1), &MotionPlanningCppNode::computationLoop, this, /*oneshot=*/false, /*autostart=*/false);
}

// @.@ Destructor
DistMotionPlanningCppNode::~DistMotionPlanningCppNode() {
    nh_.shutdown();
    nh_private_.shutdown();
    compute_timer_.stop();
}

// @.@ Load parameters
void DistMotionPlanningCppNode::loadParams() {
    p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 5);
    nh_private_.param<int>("expected_vehicles", expected_vehicles_, 2);
}

// @.@ Initialize subscribers
void DistMotionPlanningCppNode::initializeSubscribers() {
    nh_private_.param<std::string>("robot_name", self_name_, "mred0");
    std::string state_topic = "/" + self_name_ + "/State";

    state_sub_ = nh_.subscribe<farol_msgs::mState>(state_topic, 1,
                    boost::bind(&MotionPlanningCppNode::vehicleCallback, this, _1));

    // Subscribe to future state broadcasts from others
    for (int i = 0; i < expected_vehicles_; ++i) {
        std::string other = "mred" + std::to_string(i); // customize based on your naming
        if (other != self_name_) {
            std::string topic = "/" + other + "/future_state";
            future_state_subs_.push_back(
                nh_.subscribe<your_msgs::FutureState>(topic, 10,
                    boost::bind(&MotionPlanningCppNode::futureStateCallback, this, _1)));
        }
    }
}

// @.@ Initialize publishers
void DistMotionPlanningCppNode::initializePublishers() {
    std::string topic = "/" + self_name_ + "/addons/Mission_String";
    publishers_[self_name_] = nh_.advertise<std_msgs::String>(topic, 10);
    ROS_INFO_STREAM("Publisher created for: " << topic);

    future_state_pub_ = nh_.advertise<your_msgs::FutureState>("/" + self_name_ + "/future_state", 10);
}

// @.@ Initialize subscribers
void DistMotionPlanningCppNode::vehicleCallback(const farol_msgs::mState::ConstPtr &msg) {
    current_state_ = {msg->Y, msg->X, msg->Yaw * FarolGimmicks::PI / 180};
    current_velocity_ = msg->u;
    state_received_ = true;

    if (planning_started_ && state_received_ && !initial_trajectory_computed_) {
        computeInitialTrajectory();
        initial_trajectory_computed_ = true;
    }
}

void DistMotionPlanningCppNode::startCallback(const std_msgs::Empty::ConstPtr& msg) {
    if (!planning_started_) {
        ROS_INFO("Start signal received. Starting initial trajectory planning...");
        planning_started_ = true;

        // Begin computation loop timer (e.g. check 10x per second)
        computation_timer_ = nh_.createTimer(ros::Duration(0.1), &DistMotionPlanningCppNode::computationLoop, this);
    }
}

void DistMotionPlanningCppNode::computeInitialTrajectory() {
    if (!state_received_) {
        ROS_WARN("Own state not yet received. Cannot compute trajectory.");
        return;
    }

    ROS_INFO("Computing initial trajectory...");

    // Plan initial trajectory using BezierOptimization
    Eigen::Tensor<double, 3> optimal_control_points;
    double Tf_opt = 10.0;  // 10 seconds horizon

    // TODO: Call your optimization function here
    // Example: planTrajectory(current_state_, ..., optimal_control_points, Tf_opt);

    // Extract state at t=10s (END of the trajectory)
    State predicted_future_state;
    predicted_future_state.x = optimal_control_points(0, 0, BezierDegree_);  // Assuming this is how you access end point
    predicted_future_state.y = optimal_control_points(1, 0, BezierDegree_);
    predicted_future_state.theta = 0; // Compute from orientation vector if needed

    // Save locally
    future_states_[self_name_] = predicted_future_state;

    // Publish mission (trajectory)
    std_msgs::String msg;
    msg.data = formatMissionString(optimal_control_points, Tf_opt);
    mission_pub_.publish(msg);

    // Publish predicted future state at t = now + 10s
    your_msgs::FutureState fs_msg;
    fs_msg.header.stamp = ros::Time::now();
    fs_msg.name = self_name_;
    fs_msg.x = predicted_future_state.x;
    fs_msg.y = predicted_future_state.y;
    fs_msg.yaw = predicted_future_state.theta;
    future_state_pub_.publish(fs_msg);

    initial_trajectory_computed_ = true;
    ROS_INFO("Initial trajectory computed and future state published.");
}

void DistMotionPlanningCppNode::futureStateCallback(const your_msgs::FutureState::ConstPtr& msg) {
    future_states_[msg->name] = {msg->y, msg->x, msg->yaw};

    ROS_INFO_STREAM("Received future state from: " << msg->name);

    // Check if we have future states from all vehicles
    if (future_states_.size() >= expected_vehicles_) {
        ROS_INFO("All future states received. Computing next trajectory...");
        computeNextTrajectory();
    }
}

void DistMotionPlanningCppNode::computationLoop(const ros::TimerEvent&) {
    if (!planning_started_) return;

    if (!initial_trajectory_computed_) {
        computeInitialTrajectory();
    }
    
    // Otherwise, just wait — next trajectory computation is triggered
    // automatically when future states are collected in futureStateCallback().
}

void DistMotionPlanningCppNode::computeNextTrajectory() {
    ROS_INFO("Computing next trajectory from projected t=10s position...");

    State start = future_states_[self_name_];  // Your future position

    // Plan next 10s trajectory from `start`
    Eigen::Tensor<double, 3> optimal_control_points;
    double Tf_opt = 10.0;

    // TODO: planTrajectory(start, future_states_, optimal_control_points, Tf_opt);

    // Extract position at t = 10s from this next trajectory
    State predicted_future_state;
    predicted_future_state.x = optimal_control_points(0, 0, BezierDegree_);
    predicted_future_state.y = optimal_control_points(1, 0, BezierDegree_);
    predicted_future_state.theta = 0; // Update if needed

    // Update local future state
    future_states_[self_name_] = predicted_future_state;

    // Publish mission and future state
    std_msgs::String msg;
    msg.data = formatMissionString(optimal_control_points, Tf_opt);
    mission_pub_.publish(msg);

    your_msgs::FutureState fs_msg;
    fs_msg.header.stamp = ros::Time::now();
    fs_msg.name = self_name_;
    fs_msg.x = predicted_future_state.x;
    fs_msg.y = predicted_future_state.y;
    fs_msg.yaw = predicted_future_state.theta;
    future_state_pub_.publish(fs_msg);

    // Reset only others’ states to wait for their next prediction
    for (auto it = future_states_.begin(); it != future_states_.end(); ++it) {
        if (it->first != self_name_) {
            it->second = {}; // reset to empty struct
        }
    }

    ROS_INFO("Next trajectory computed and future state updated.");
}

// @.@ Format the mission string for a vehicle
std::string DistMotionPlanningCppNode::formatMissionString(const Eigen::Tensor<double, 3> &controlPoints, int vehicleIndex, double Tf_opt) {
    std::ostringstream oss;
    State ref_state = vehicle_states_[reference_vehicle_];
    oss << std::fixed << std::setprecision(3);

    // oss << "3\n491883.797 4290843.566 29S\nBEZIER " << controlPoints.dimension(2) << " ";
    oss << "3\n" << std::fixed << std::setprecision(3)  << ref_state.x << " " << ref_state.y << " 29S\nBEZIER " << controlPoints.dimension(2) << " ";

    for (int j = 0; j < controlPoints.dimension(2); ++j) {
        oss << controlPoints(vehicleIndex, 0, j) << " ";  // X values
    }
    for (int j = 0; j < controlPoints.dimension(2); ++j) {
        oss << controlPoints(vehicleIndex, 1, j) << " ";  // Y values
    }

    oss << Tf_opt; // Example time value

    return oss.str();
}

// @.@ Main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "dist_motion_planning_cpp_node");  // Name this node appropriately
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    DistMotionPlanningCppNode dist_motion_planning_cpp(&nh, &nh_private);
    ros::spin();

    return 0;
}
