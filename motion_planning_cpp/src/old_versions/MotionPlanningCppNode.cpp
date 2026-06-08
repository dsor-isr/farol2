#include "MotionPlanningCppNode.h"

// @.@ Constructor
MotionPlanningCppNode::MotionPlanningCppNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private)
    : nh_(*nodehandle), nh_private_(*nodehandle_private), mred_received_(false), mblack_received_(false) {

    loadParams();
    initializeSubscribers();
    initializePublishers();
}

// @.@ Destructor
MotionPlanningCppNode::~MotionPlanningCppNode() {
    nh_.shutdown();
    nh_private_.shutdown();
}

// @.@ Load parameters
void MotionPlanningCppNode::loadParams() {
    p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 5);
}

// @.@ Initialize subscribers
void MotionPlanningCppNode::initializeSubscribers() {
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    std::regex pattern("^/(m(red|black|yellow|vector)[0-9]+)/State$");

    for (const auto& topic : master_topics) {
        std::smatch match;
        if (std::regex_match(topic.name, match, pattern)) {
            std::string vehicle_name = match[1];

            if (std::find(vehicle_names_.begin(), vehicle_names_.end(), vehicle_name) == vehicle_names_.end()) {
                vehicle_names_.push_back(vehicle_name);
                received_flags_[vehicle_name] = false;

                subscribers_[vehicle_name] = nh_.subscribe<farol_msgs::mState>(
                    "/" + vehicle_name + "/State", 1,
                    boost::bind(&MotionPlanningCppNode::vehicleCallback, this, _1, vehicle_name)
                );
            }
        }
    }

    if (vehicle_names_.empty()) {
        ROS_WARN("No vehicles found on topics.");
    } else {
        reference_vehicle_ = vehicle_names_.front(); // First found is reference
        ROS_INFO_STREAM("Reference vehicle: " << reference_vehicle_);
    }
}

void MotionPlanningCppNode::initializePublishers() {
    for (const auto& vehicle_name : vehicle_names_) {
        std::string topic = "/" + vehicle_name + "/addons/Mission_String";
        publishers_[vehicle_name] = nh_.advertise<std_msgs::String>(topic, 10);
        ROS_INFO_STREAM("Publisher created for: " << topic);
    }
}

void MotionPlanningCppNode::vehicleCallback(const farol_msgs::mState::ConstPtr &msg, const std::string& vehicle_name) {
    State state = {msg->Y, msg->X, msg->Yaw * FarolGimmicks::PI / 180};
    vehicle_states_[vehicle_name] = state;
    vehicle_velocities_[vehicle_name] = msg->u;
    received_flags_[vehicle_name] = true;

    // Check if all flags are true
    bool all_received = std::all_of(received_flags_.begin(), received_flags_.end(),
                                     [](const auto& pair) { return pair.second; });

    if (all_received) {
        processAndPublish();
    }
}


void MotionPlanningCppNode::processAndPublish() {
    ROS_INFO("All vehicle states received. Running Bezier Optimization...");

    State ref_state = vehicle_states_[reference_vehicle_];

    std::vector<State> current_states;
    std::vector<State> goal_states;
    std::vector<double> v0s;
    std::vector<double> vTs;
    std::vector<ros::Publisher> publishers;

    enum class GoalStrategy
    {
        ForwardHeading,
        LineFormation,
        OptimalAssignment
    };

    GoalStrategy strategy = GoalStrategy::LineFormation; // or .ForwardHeading or .OptimalAssignment

    State line_start = { 4290880.0 - ref_state.x, 491945.0 - ref_state.y, FarolGimmicks::PI};

    int num_vehicles = vehicle_names_.size();

    std::vector<State> all_goal_candidates;
    for (int i = 0; i < num_vehicles; ++i)
    {
        all_goal_candidates.push_back({line_start.x,
                                       line_start.y + 40.0 * i,
                                       FarolGimmicks::PI});
    }

    // Pre-store current states
    std::vector<State> rel_states;
    std::vector<std::string> names;

    for (const auto &name : vehicle_names_)
    {
        State abs = vehicle_states_[name];
        State rel = {abs.x - ref_state.x, abs.y - ref_state.y, abs.theta};
        current_states.push_back(rel);
        rel_states.push_back(rel);
        names.push_back(name);
    }

    // Prepare goal_states to be filled
    goal_states.clear();

    if (strategy == GoalStrategy::OptimalAssignment)
    {
        // Use cost matrix and Hungarian algorithm
        Eigen::MatrixXd cost(num_vehicles, num_vehicles);
        for (int i = 0; i < num_vehicles; ++i)
        {
            for (int j = 0; j < num_vehicles; ++j)
            {
                double dx = rel_states[i].x - all_goal_candidates[j].x;
                double dy = rel_states[i].y - all_goal_candidates[j].y;
                cost(i, j) = std::sqrt(dx * dx + dy * dy);
            }
        }

        HungarianAlgorithm hungarian;
        std::vector<int> assignment;
        hungarian.Solve(cost, assignment);

        for (int i = 0; i < num_vehicles; ++i)
        {
            const auto &name = vehicle_names_[i];
            const State &abs = vehicle_states_[name];
            const State &rel = rel_states[i];
            const State &goal = all_goal_candidates[assignment[i]];

            std::cout << "=== Vehicle: " << name << " ===\n";
            std::cout << "Ref State:     x=" << ref_state.x << ", y=" << ref_state.y << ", theta=" << ref_state.theta << "\n";
            std::cout << "Current (abs): x=" << abs.x << ", y=" << abs.y << ", theta=" << abs.theta << "\n";
            std::cout << "Current (rel): x=" << rel.x << ", y=" << rel.y << ", theta=" << rel.theta << "\n";
            std::cout << "Goal (rel):    x=" << goal.x << ", y=" << goal.y << ", theta=" << goal.theta << "\n";
            std::cout << "Distance:      " << std::hypot(rel.x - goal.x, rel.y - goal.y) << "\n";
            std::cout << "---------------------------\n";

            goal_states.push_back(goal);
        }
    }
    else
    {
        int robot_index = 0;
        for (const auto &rel : rel_states)
        {
            State goal;

            if (strategy == GoalStrategy::ForwardHeading)
            {
                goal = {
                    rel.x + 20 * cos(rel.theta),
                    rel.y + 20 * sin(rel.theta),
                    rel.theta};
            }
            else if (strategy == GoalStrategy::LineFormation)
            {
                State temp = {
                    line_start.x - 0.0 * robot_index,
                    line_start.y - 15.0 * robot_index,
                    -0*FarolGimmicks::PI/2};
                goal = temp;
            }

            goal_states.push_back(goal);
            robot_index++;
        }
    }


    /* Eigen::MatrixXd circ_obs(3, 5);  

    circ_obs << 
        20.0,    48.0,    40.0,    20.0,    52.0,
        60.0,     40.0,   20.0,    32.0,    64.0,
        6.5,    11.5,     6.5,     4.0,     7.0;
         

    Eigen::MatrixXd line_obs(3, 4);

    line_obs << 
         1.0,     1.0,     0.0,     0.0,
        0.0,     0.0,     1.0,     1.0,
        4.0,   -84.0,   -84.0,    4.0;*/

    Eigen::MatrixXd circ_obs(3, 0);
    Eigen::MatrixXd line_obs(3, 0);

    
    // Remaining common setup
    for (const auto &name : vehicle_names_)
    {
        v0s.push_back(std::max(vehicle_velocities_[name], 0.02));
        vTs.push_back(0.21);
        publishers.push_back(nh_.advertise<std_msgs::String>("/" + name + "/addons/Mission_String", 10));
    }

    bool first_iter = true;
    int initialDegree = 8;  
    int finalDegree = 10;
    std::vector<int> nSplit = {1, 1, 1};

    Eigen::Tensor<double, 3> cont_P_guess(vehicle_names_.size(), 2, 2);
    cont_P_guess.setZero();

    BezierOptimization bezierOptimization(vehicle_names_.size(), initialDegree, nSplit, current_states, goal_states, v0s, vTs, circ_obs, line_obs);
    bezierOptimization.runOptimization(first_iter, cont_P_guess, 0, initialDegree);

    first_iter = false;
    bezierOptimization.runOptimization(first_iter, bezierOptimization.getControlPoints(), bezierOptimization.getTf(), finalDegree);

    auto controlPoints = bezierOptimization.getControlPoints();
    double Tf = bezierOptimization.getTf();

    int i = 0;
    for (const auto& name : vehicle_names_) {
        std_msgs::String msg;
        msg.data = formatMissionString(controlPoints, i, Tf);
        publishers_[name].publish(msg);
        ROS_INFO_STREAM("Mission " << name << ": " << msg.data);
        i++;
    }
/*
    // ==== SKIP OPTIMIZATION: USE PREDEFINED TRAJECTORIES ====
    std::map<std::string, std::string> precomputed_missions;

    precomputed_missions["mblack0"] =
        "3\n4290809.863 491879.919 29S\nBEZIER 11 "
        "0.380 1.440 2.660 2.560 95.365 -43.014 72.189 37.196 79.094 79.806 80.337 "
        "40.210 40.210 40.208 85.558 -44.867 47.094 81.357 65.375 40.682 40.681 40.681 "
        "530.241";

    precomputed_missions["mred0"] =
        "3\n4290809.863 491879.919 29S\nBEZIER 11 "
        "0.000 1.060 2.088 73.408 -11.048 10.176 60.375 52.885 77.103 79.806 80.337 "
        "0.000 -0.000 0.002 -2.994 32.186 53.741 79.756 16.437 80.646 80.681 80.681 "
        "530.241";

    precomputed_missions["mred1"] =
        "3\n4290809.863 491879.919 29S\nBEZIER 11 "
        "0.293 1.353 2.502 13.748 120.279 -77.695 96.999 27.029 78.088 79.806 80.337 "
        "80.621 80.621 80.616 66.843 94.213 2.894 16.886 45.374 0.685 0.681 0.681 "
        "530.241";
    for (const auto& name : vehicle_names_) {
        std_msgs::String msg;

        if (precomputed_missions.count(name)) {
            msg.data = precomputed_missions[name];
        } else {
            ROS_WARN_STREAM("No precomputed mission for vehicle: " << name);
            continue;
        }

        publishers_[name].publish(msg);
        ROS_INFO_STREAM("Published precomputed mission for " << name);
    }

    */
    // Reset flags
    for (auto& f : received_flags_) f.second = false;

    ros::shutdown();
}

// @.@ Format the mission string for a vehicle
std::string MotionPlanningCppNode::formatMissionString(const Eigen::Tensor<double, 3> &controlPoints, int vehicleIndex, double Tf_opt) {
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
    ros::init(argc, argv, "motion_planning_cpp_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    MotionPlanningCppNode motion_planning_cpp(&nh, &nh_private);
    ros::spin();

    return 0;
}
