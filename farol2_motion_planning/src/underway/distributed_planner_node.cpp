#include "DistributedMotionPlanningNode.hpp"


// @.@ Constructor
DistributedMotionPlanningNode::DistributedMotionPlanningNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private)
    : nh_(*nodehandle), nh_private_(*nodehandle_private)
{
    ROS_INFO("Initializing Interactive Motion Planning Node...");

    loadParams();
    // scanForVehicles();
    planning_alive_pub_ = nh_.advertise<std_msgs::Bool>("/motion_planning_cpp/node_alive", 1);
    mission_log_pub_ = nh_.advertise<std_msgs::String>("/mission_log", 10);

    alive_timer_ = nh_.createTimer(
    ros::Duration(1.0), [this](const ros::TimerEvent&) {std_msgs::Bool msg; msg.data = true; planning_alive_pub_.publish(msg);});

    // vehicle_discovery_srv_   = nh_.advertiseService("/motion_planning_cpp/discover_vehicles", &DistributedMotionPlanningNode::discoverVehicles, this);
    set_goal_srv_            = nh_.advertiseService("/motion_planning_cpp/set_goal", &DistributedMotionPlanningNode::setGoalService, this);
    sample_state_srv_        = nh_.advertiseService("/motion_planning_cpp/sample_states", &DistributedMotionPlanningNode::sampleStatesService, this);
    set_bounds_srv_          = nh_.advertiseService("/motion_planning_cpp/set_bounds", &DistributedMotionPlanningNode::setBoundsService, this);
    set_bezier_params_srv_   = nh_.advertiseService("/motion_planning_cpp/set_bezier_params", &DistributedMotionPlanningNode::setBezierParamsService, this);
    set_obstacles_srv_       = nh_.advertiseService("/motion_planning_cpp/set_obstacles", &DistributedMotionPlanningNode::setObstaclesService, this);
    get_planner_config_srv_  = nh_.advertiseService("/motion_planning_cpp/get_planner_config", &DistributedMotionPlanningNode::getPlannerConfigService, this);
    run_optimization_srv_    = nh_.advertiseService("/motion_planning_cpp/run_optimization", &DistributedMotionPlanningNode::runOptimizationService, this);
    deploy_mission_srv_      = nh_.advertiseService("/motion_planning_cpp/deploy_mission", &DistributedMotionPlanningNode::deployMissionService, this);
    cancel_optimization_srv_ = nh_.advertiseService("/motion_planning_cpp/cancel_optimization", &DistributedMotionPlanningNode::cancelOptimizationService, this);



    bezier_degree_ = 9;
    guess_degree_ =7;
    nSplit_ = {1, 1, 1};
    constr_flags_ = {1, 0, 0, 0};
    obstacles_updated_ = false;
    number_sample_Pts_ = 100;

    ROS_INFO("Interactive Motion Planning Node initialized successfully.");
}

// @.@ Destructor
DistributedMotionPlanningNode::~DistributedMotionPlanningNode() {
    nh_.shutdown();
    nh_private_.shutdown();
}

void DistributedMotionPlanningNode::publishLog(const std::string& text)
{
    std_msgs::String msg;
    msg.data = text;
    mission_log_pub_.publish(msg);
}

// @.@ Load parameters
void DistributedMotionPlanningNode::loadParams() {
    //p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 5);
}

// @.@ Initialize subscribers
void DistributedMotionPlanningNode::scanForVehiclesold() {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    std::regex pattern("^/(m(red|black|yellow|vector)[0-9]+)/State$");

    for (const auto& topic : master_topics) {
        std::smatch match;
        if (std::regex_match(topic.name, match, pattern)) {
            std::string vehicle_name = match[1];

             if (vehicle_names_set_.insert(vehicle_name).second) {

                ROS_INFO_STREAM("New vehicle discovered: " << vehicle_name);
                vehicle_names_.push_back(vehicle_name);

                std::string mission_topic =
                    "/" + vehicle_name + "/addons/Mission_String";

                publishers_[vehicle_name] =
                    nh_.advertise<std_msgs::String>(mission_topic, 10);
            }
        }
    }

    if (!vehicle_names_.empty() && reference_vehicle_.empty()) {
        reference_vehicle_ = vehicle_names_.front();
        ROS_INFO_STREAM("Reference vehicle set to: " << reference_vehicle_);
    }

    for (const auto& name : vehicle_names_) {
        traj_pubs_[name] = nh_.advertise<nav_msgs::Path>("/" + name + "/planned_path", 1, true);
    }
}

void DistributedMotionPlanningNode::scanForVehicles(const std::vector<std::string>& discovered_vehicles) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (const auto& vehicle_name : discovered_vehicles) {
        if (vehicle_names_set_.insert(vehicle_name).second) {
            ROS_INFO_STREAM("New vehicle discovered: " << vehicle_name);
            vehicle_names_.push_back(vehicle_name);

            std::string mission_topic = "/" + vehicle_name + "/addons/Mission_String";
            publishers_[vehicle_name] = nh_.advertise<std_msgs::String>(mission_topic, 10);
            traj_pubs_[vehicle_name] = nh_.advertise<nav_msgs::Path>("/" + vehicle_name + "/planned_path", 1, true);
        }
    }

    if (!vehicle_names_.empty() && reference_vehicle_.empty()) {
        reference_vehicle_ = vehicle_names_.front();
        ROS_INFO_STREAM("Reference vehicle set to: " << reference_vehicle_);
    }
}

/*bool DistributedMotionPlanningNode::discoverVehicles(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    scanForVehicles();
    if(vehicle_names_.empty()) {
        res.success = false;
        res.message = "No vehicles found.";
    } else {
        res.success = true;
        std::ostringstream oss;
        for(const auto &name : vehicle_names_) oss << name << " ";
        res.message = "Active vehicles: " + oss.str();
    }
    
    return true;
}
*/

void DistributedMotionPlanningNode::processState(const vehicle_State msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> lock(state_mutex_);  // thread-safe

    // Check if this vehicle is already known
    if (std::find(vehicle_names_.begin(), vehicle_names_.end(), vehicle_name) == vehicle_names_.end()) {
        // New vehicle → register it
        vehicle_names_.push_back(vehicle_name);
        ROS_INFO_STREAM("New vehicle registered: " << vehicle_name);

        // Create its trajectory publisher
        traj_pubs_[vehicle_name] = nh_.advertise<nav_msgs::Path>("/" + vehicle_name + "/planned_path", 1, true);

        // Also create mission publisher
        std::string mission_topic = "/" + vehicle_name + "/planning_console/Mission_String";
        publishers_[vehicle_name] = nh_.advertise<std_msgs::String>(mission_topic, 10);
    }

    // Store/update its state
    vehicle_State rel_state = {
        msg.y,                // Northing
        msg.x,                // Easting
        msg.theta,            // Yaw
        std::max(msg.v, 0.02) // Velocity
    };

    vehicle_states_[vehicle_name] = rel_state;

    ROS_INFO_STREAM("Received state for " << vehicle_name
                    << ": X(E)=" << msg.x
                    << ", Y(N)=" << msg.y
                    << ", Yaw=" << msg.theta
                    << ", u=" << msg.v);
}

bool DistributedMotionPlanningNode::sampleStatesService(motion_planning_cpp::SetState::Request &req, motion_planning_cpp::SetState::Response &res)
{

    // Build a temporary farol_msgs::mState message
    vehicle_State msg;
    msg.x   = req.E;     // Easting
    msg.y   = req.N;     // Northing
    msg.theta = req.theta;
    msg.v   = req.v;

    // Store it in vehicle_states_ map
    processState(msg, req.vehicle_name);

    res.success = true;
    res.message = "Vehicle state stored for " + req.vehicle_name;
    publishLog(res.message);

    return true;
}
/*bool DistributedMotionPlanningNode::sampleStatesServiceold(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    scanForVehiclesold();
    vehicle_states_.clear();

    for (const auto& vehicle_name : vehicle_names_) {

        auto msg = ros::topic::waitForMessage<farol_msgs::mState>(
            "/" + vehicle_name + "/State",
            nh_,
            ros::Duration(2.0));

        if (!msg) {
            res.success = false;
            res.message = "Timeout waiting for " + vehicle_name;
            return true;
        }

        processStateold(msg, vehicle_name);
    }

    res.success = true;
    res.message = "Vehicle states sampled successfully";
    publishLog("Vehicle states sampled successfully");
    return true;
}*/


bool DistributedMotionPlanningNode::setGoalService(motion_planning_cpp::SetState::Request &req, motion_planning_cpp::SetState::Response &res)
{
    if(std::find(vehicle_names_.begin(), vehicle_names_.end(), req.vehicle_name) == vehicle_names_.end()) {
        res.success = false;
        res.message = "Vehicle not found!";
        return true;
    }
    if(req.v <= 0) {
        res.success = false;
        res.message = "Invalid velocity!";
        return true;
    }

    vehicle_State abs_goal;
    abs_goal.x     = req.N; // Northing
    abs_goal.y     = req.E; // Easting
    abs_goal.theta = req.theta;
    abs_goal.v     = req.v;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        goal_states_map_[req.vehicle_name] = abs_goal;
    }
    ROS_INFO_STREAM("Received state for " << req.vehicle_name
                    << ": X(E)=" << abs_goal.x
                    << ", Y(N)=" << abs_goal.y
                    << ", Yaw=" << abs_goal.theta
                    << ", u=" << abs_goal.v);

    res.success = true;
    res.message = "Goal set for " + req.vehicle_name;
    publishLog("Goal set for " + req.vehicle_name);
    return true;
}

bool DistributedMotionPlanningNode::setBoundsService(motion_planning_cpp::SetBoundsandGains::Request &req, motion_planning_cpp::SetBoundsandGains::Response &res) {
    bounds_.vel_min     = req.vel_min;
    bounds_.vel_max     = req.vel_max;
    bounds_.acc_min     = req.acc_min;
    bounds_.acc_max     = req.acc_max;
    bounds_.ang_vel_min = req.ang_vel_min;
    bounds_.ang_vel_max = req.ang_vel_max;
    bounds_.ang_acc_min = req.ang_acc_min;
    bounds_.ang_acc_max = req.ang_acc_max;
    bounds_.obs_min     = req.obs_min;
    if (req.obs_max < 10000) {
        bounds_.obs_max = req.obs_max;
    }
    else {
        bounds_.obs_max = std::numeric_limits<double>::infinity();
    }
    bounds_.radius      = req.radius;
    bounds_.alpha       = req.alpha;
    bounds_.beta        = req.beta;
    bounds_.gamma       = req.gamma;

    bounds_.is_set = true;

    res.success = true;
    res.message = "Bounds and gains set.";
    publishLog(res.message);
    ROS_INFO_STREAM(res.message);
    return true;
}

bool DistributedMotionPlanningNode::setBezierParamsService(motion_planning_cpp::SetBezierParams::Request &req, motion_planning_cpp::SetBezierParams::Response &res)
{
    if (req.constr_flags.size() != 4) {
        res.success = false;
        res.message = "constr_flags must have size 4";
        ROS_ERROR_STREAM(res.message);
        return true;
    }

    if (req.nSplit.empty()) {
        res.success = false;
        res.message = "nSplit cannot be empty";
        ROS_ERROR_STREAM(res.message);
        return true;
    }

    if (req.bezier_degree < 4) {
        res.success = false;
        res.message = "bezier_degree must be at least 4";
        ROS_ERROR_STREAM(res.message);
        return true;
    }
    if (req.number_sample_Pts < 20) {
        res.success = false;
        res.message = "Path must have at least 20 sample points";
        ROS_ERROR_STREAM(res.message);
        return true;
    }

    bezier_degree_ = req.bezier_degree;
    guess_degree_ = req.guess_degree;
    nSplit_        = req.nSplit;
    constr_flags_  = req.constr_flags;
    number_sample_Pts_ = req.number_sample_Pts;

    ROS_INFO_STREAM("Bezier params updated:");
    ROS_INFO_STREAM("  degree: " << bezier_degree_);

    std::ostringstream ns;
    ns << "  nSplit: ";
    for (auto v : nSplit_) ns << v << " ";
    ROS_INFO_STREAM(ns.str());

    std::ostringstream cf;
    cf << "  constr_flags: ";
    for (auto v : constr_flags_) cf << (v ? "true " : "false ");
    ROS_INFO_STREAM(cf.str());

    res.success = true;
    res.message = "Bezier parameters updated successfully";
    publishLog(res.message);
    return true;
}

bool DistributedMotionPlanningNode::getPlannerConfigService(motion_planning_cpp::GetPlannerConfig::Request &req, motion_planning_cpp::GetPlannerConfig::Response &res)
{
    // Bounds
    res.vel_min = bounds_.vel_min;
    res.vel_max = bounds_.vel_max;
    res.acc_min = bounds_.acc_min;
    res.acc_max = bounds_.acc_max;
    res.ang_vel_min = bounds_.ang_vel_min;
    res.ang_vel_max = bounds_.ang_vel_max;
    res.ang_acc_min = bounds_.ang_acc_min;
    res.ang_acc_max = bounds_.ang_acc_max;
    res.obs_min = bounds_.obs_min;
    res.obs_max = bounds_.obs_max;
    res.radius  = bounds_.radius;

    res.alpha = bounds_.alpha;
    res.beta  = bounds_.beta;
    res.gamma = bounds_.gamma;

    // Bezier
    res.bezier_degree = bezier_degree_;
    res.guess_degree  = guess_degree_;
    res.nSplit        = nSplit_;
    res.constr_flags  = constr_flags_;
    res.number_sample_Pts = number_sample_Pts_;

    return true;
}

bool DistributedMotionPlanningNode::setObstaclesService(motion_planning_cpp::SetObstacles::Request &req, motion_planning_cpp::SetObstacles::Response &res)
{
    // --- Circular obstacles ---
    size_t n_circ = req.circ_obs.size() / 3;
    
    if (n_circ) {
        circ_obs_.resize(3, n_circ); // circ_obs  # [E1, N1, r1, E2, N2, r2, ...]

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            for (size_t i = 0; i < n_circ; ++i) {
                circ_obs_(0, i) = req.circ_obs[i*3 + 1]; // Northing
                circ_obs_(1, i) = req.circ_obs[i*3 + 0]; // Easting
                circ_obs_(2, i) = req.circ_obs[i*3 + 2];

            }
        }

        obstacles_updated_ = true;
    }
    else {
        circ_obs_.resize(3, 0); // No circular obstacles
    }
    
    

    // --- Line obstacles --- (TODO... - just a placeholder)
    size_t n_line = req.line_obs.size() / 3;
    if (n_line) {
        line_obs_.resize(3, n_line);
        for (size_t i = 0; i < n_line; ++i) {
            line_obs_(1, i) = req.line_obs[i*3 + 0]; // x1
            line_obs_(0, i) = req.line_obs[i*3 + 1]; // y1
            line_obs_(2, i) = req.line_obs[i*3 + 2]; // x2 or other line info
        }
    }
    else {
        line_obs_.resize(3, 0); // No line obstacles
    }

    if (n_circ == 0 && n_line == 0) {
        obstacles_updated_ = false; // No obstacles provided
    }
    
    // --- Build response + logs ---
    std::ostringstream msg;
    msg << "Obstacles updated: ";

    if (n_circ > 0) {
        msg << n_circ << " circular obstacle(s)";
    } else {
        msg << "No circular obstacles";
    }

    msg << " | ";

    if (n_line > 0) {
        msg << n_line << " line obstacle(s)";
    } else {
        msg << "No line obstacles";
    }

    res.success = true;
    res.message = msg.str();

    // --- ROS output ---
    ROS_INFO_STREAM(res.message);
    publishLog(res.message);

    return true;
}

bool DistributedMotionPlanningNode::runOptimizationService(motion_planning_cpp::RunOptimization::Request &req, motion_planning_cpp::RunOptimization::Response &res) {
        // ================= DEBUG PRINT =================
    ROS_INFO_STREAM("========== DEBUG STATE ==========");

    // Vehicle names
    std::ostringstream names_ss;
    names_ss << "vehicle_names_: ";
    for (const auto& name : vehicle_names_) {
        names_ss << "[" << name << "] ";
    }
    ROS_INFO_STREAM(names_ss.str());

    // Vehicle states
    std::ostringstream states_ss;
    states_ss << "vehicle_states_: ";
    for (const auto& [name, state] : vehicle_states_) {
        states_ss << "\n  [" << name << "] "
                << "E=" << state.y
                << ", N=" << state.x
                << ", yaw=" << state.theta
                << ", v=" << state.v;
    }
    ROS_INFO_STREAM(states_ss.str());

    // Goal states
    std::ostringstream goals_ss;
    goals_ss << "goal_states_map_: ";
    for (const auto& [name, goal] : goal_states_map_) {
        goals_ss << "\n  [" << name << "] "
                << "E=" << goal.y
                << ", N=" << goal.x
                << ", yaw=" << goal.theta
                << ", v=" << goal.v;
    }
    ROS_INFO_STREAM(goals_ss.str());

    ROS_INFO_STREAM("=================================");
    // =================================================
        
    // Basic checks
    if(vehicle_names_.empty()) {
        res.success = false;
        res.message = "No vehicles discovered!";
        publishLog(res.message);
        return true;
    }

    if(vehicle_states_.empty()) {
        res.success = false;
        res.message = "Cannot run optimization: no vehicle states sampled yet.";
        publishLog(res.message);
        return true;
    }

    // Check goals
    for(const auto &name : vehicle_names_) {
        if(goal_states_map_.count(name) == 0) {
            res.success = false;
            res.message = "Cannot run optimization: goal not defined for vehicle " + name;
            ROS_WARN_STREAM(res.message);
            publishLog(res.message);
            return true;
        }
    }
    publishLog("Setting up optimization problem");
    std::vector<vehicle_State> current_states;
    std::vector<vehicle_State> goal_states;
    std::vector<std::string> vehicles;
    vehicles.clear();
    current_states.clear();
    goal_states.clear();

    int NVehicles;
    
    // Copy shared data
    // If request empty → default to all vehicles
    if (req.vehicle_names.empty()) {
        vehicles = vehicle_names_;
    } else {
        for (const auto& name : req.vehicle_names) {

            // Check if vehicle exists
            if (std::find(vehicle_names_.begin(),
                        vehicle_names_.end(),
                        name) == vehicle_names_.end()) {

                res.success = false;
                res.message = "Vehicle not found: " + name;
                return true;
            }

            vehicles.push_back(name);
        }
    }
    selected_vehicle_names_ = vehicles;
    NVehicles = vehicles.size();
    std::string local_reference = vehicles.front();
    vehicle_State ref_state = vehicle_states_[local_reference];

    for(const auto &name : vehicles) {
        vehicle_State current_state;
        current_state.x = vehicle_states_[name].x-ref_state.x; // Northing
        current_state.y = vehicle_states_[name].y-ref_state.y; // Easting
        current_state.theta = vehicle_states_[name].theta;
        current_state.v = vehicle_states_[name].v;
        current_states.push_back(current_state);

        vehicle_State goal_state;
        goal_state.x = goal_states_map_[name].x- ref_state.x; // Northing
        goal_state.y = goal_states_map_[name].y- ref_state.y; // Easting
        goal_state.theta = goal_states_map_[name].theta;
        goal_state.v = goal_states_map_[name].v;
        
        goal_states.push_back(goal_state);

        ROS_INFO_STREAM("Vehicle: " << name << "\n | Current State: Easting = " << current_state.y << ", Northing = " << current_state.x << ", Yaw = " << current_state.theta << ", u = " << current_state.v << " |"
                                            << "\n | Goal State:    Easting = " << goal_state.y << ", Northing = " << goal_state.x << ", Yaw = " << goal_state.theta << ", u = " << goal_state.v << " |");
    }

    // Initialize solver
    // Initialize solver
    MultipleVehicleMotionPlan solver_first(guess_degree_, NVehicles, {1,1,1}, constr_flags_);
    MultipleVehicleMotionPlan solver(bezier_degree_, NVehicles, nSplit_, constr_flags_);

    if(bounds_.is_set) {
        ROS_INFO("Bounds are no longer Default.");
        solver.setBoundsAndGains(
            bounds_.vel_min, bounds_.vel_max,
            bounds_.acc_min, bounds_.acc_max,
            bounds_.ang_vel_min, bounds_.ang_vel_max,
            bounds_.ang_acc_min, bounds_.ang_acc_max,
            bounds_.obs_min, bounds_.obs_max,
            bounds_.radius,
            bounds_.alpha, bounds_.beta, bounds_.gamma
        );
    }
    solver.setupSymbolic();
    solver.computeCostFunction();
    solver_first.setupSymbolic();
    solver_first.computeCostFunction();

    if(obstacles_updated_) {
        Eigen::MatrixXd adjusted_circ_obs(3, circ_obs_.cols());
        for (int i = 0; i < circ_obs_.cols(); ++i) {
            adjusted_circ_obs(0, i) = circ_obs_(0, i) + ref_state.x;  // Northing
            adjusted_circ_obs(1, i) = circ_obs_(1, i) + ref_state.y;  // Easting
            adjusted_circ_obs(2, i) = circ_obs_(2, i);                // radius stays the same
        }
        solver_first.setOptimizationProblem(current_states, goal_states, adjusted_circ_obs, line_obs_);
    } else {
        ROS_INFO("No obstacle updates detected.");
        solver_first.setOptimizationProblem(current_states, goal_states);
    }


    solver_first.createConstraintVector();
    solver_first.createDecisionVector();
    
    cancel_flag_ = false; // reset before starting
    opt_running_ = true;
    opt_thread_ = std::thread([this, res, vehicles, ref_state, current_states, goal_states, solver = std::move(solver), solver_first = std::move(solver_first)]() mutable {
        try {

            publishLog("Starting Geuess calculation");
            solver_first.solveOptimizationProblem(&cancel_flag_);
            // solver.solveOptimizationProblem();

            Eigen::Tensor<double, 3> control_points_guess = BezierUtils::elevateTensorDegree(solver_first.getControlPoints(), bezier_degree_-guess_degree_);
            double Tf_guess = solver_first.getTf();
            // assume control_points_guess is Eigen::Tensor<double, 3>
            int dim0 = control_points_guess.dimension(0);
            int dim1 = control_points_guess.dimension(1);
            int dim2 = control_points_guess.dimension(2);

            for (int i = 0; i < dim0; ++i) {
                std::cout << "Tensor slice " << i << ":\n";
                for (int j = 0; j < dim1; ++j) {
                    std::cout << "  [";
                    for (int k = 0; k < dim2; ++k) {
                        std::cout << control_points_guess(i,j,k);
                        if (k != dim2-1) std::cout << ", ";
                    }
                    std::cout << "]\n";
                }
                std::cout << "\n";
            }

            if(obstacles_updated_) {
                for (int i = 0; i < circ_obs_.cols(); ++i) {
                    ROS_INFO_STREAM("| Obstacle " << i << ":    Easting =  " << circ_obs_(1, i) << ", Northing = " << circ_obs_(0, i) << ", r = " << circ_obs_(2, i) << " |");
                }
                Eigen::MatrixXd adjusted_circ_obs(3, circ_obs_.cols());
                for (int i = 0; i < circ_obs_.cols(); ++i) {
                    adjusted_circ_obs(0, i) = circ_obs_(0, i) + ref_state.x;  // Northing
                    adjusted_circ_obs(1, i) = circ_obs_(1, i) + ref_state.y;  // Easting
                    adjusted_circ_obs(2, i) = circ_obs_(2, i);                // radius stays the same
                }
                solver.setOptimizationProblem(current_states, goal_states, adjusted_circ_obs, line_obs_, control_points_guess, Tf_guess, false);
            } else {
                ROS_INFO("No obstacle updates detected.");
                Eigen::Matrix<double, 3, Eigen::Dynamic> empty_obs(3, 0);
                solver.setOptimizationProblem(current_states, goal_states, empty_obs, empty_obs, control_points_guess, Tf_guess, false);
            }

            solver.createConstraintVector();
            solver.createDecisionVector();
            publishLog("Starting Optimization");
            solver.solveOptimizationProblem(&cancel_flag_);
            // solver.solveOptimizationProblem();

            last_control_points_ = solver.getControlPoints();
            last_Tf_ = solver.getTf();

            ROS_INFO_STREAM("=== Optimization Results ===");
            ROS_INFO_STREAM("Final time Tf = " << last_Tf_);

            int num_vehicles = last_control_points_.dimension(0);
            int num_points   = last_control_points_.dimension(2);

            for(int v = 0; v < num_vehicles; ++v) {
                std::ostringstream oss;
                oss << "Vehicle " << vehicle_names_[v] << " control points:\n";

                for(int d = 0; d < 2; ++d) { // 0 = X, 1 = Y
                    oss << (d == 0 ? "  X: " : "  Y: ");
                    for(int p = 0; p < num_points; ++p) {
                        oss << last_control_points_(v, d, p) << " ";
                    }
                    oss << "\n";
                }

                ROS_INFO_STREAM(oss.str());
            }
            res.success = true;
            res.message = solver.getOptimizationstatus();
            publishLog(res.message);
            // --- Publish planned paths ---
            Eigen::Tensor<double, 3> trajs = solver.getOptimalTrajectories(number_sample_Pts_);
            for (size_t i = 0; i < vehicles.size(); ++i) {

                nav_msgs::Path path;
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "map";  // or "local", just be consistent

                for (size_t j = 0; j < trajs.dimension(2); ++j) {
                    geometry_msgs::PoseStamped p;
                    p.header = path.header;

                    p.pose.position.x = trajs(i, 0, j) + ref_state.x; // Northing
                    p.pose.position.y = trajs(i, 1, j) + ref_state.y; // Easting
                    p.pose.position.z = 0.0;

                    // Optional: orientation
                    p.pose.orientation.w = 1.0;

                    path.poses.push_back(p);
                }

                traj_pubs_[vehicles[i]].publish(path);
            }
            publishMissions();
            opt_running_ = false;
        } catch (const std::exception& e) {
            ROS_WARN_STREAM("Optimization thread stopped: " << e.what());
            publishLog(std::string("Optimization thread stopped: ") + e.what());
            opt_running_ = false;
        }
    });
    opt_thread_.detach();

    return true;
}

bool DistributedMotionPlanningNode::cancelOptimizationService(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res)
{
    if (!opt_running_) {
        res.success = false;
        res.message = "No optimization running";
        return true;
    }

    cancel_flag_ = true;  // thread will ignore results
    res.success = true;
    res.message = "Optimization abort requested";

    ROS_WARN("Optimization abort requested by user");
    return true;
}

void DistributedMotionPlanningNode::publishMissions()
{
    if(last_control_points_.size() == 0) {
        ROS_WARN("No optimized trajectory available!");
        return;
    }

    int i = 0;
    for(const auto &name : selected_vehicle_names_) {

        std_msgs::String msg;

        msg.data = formatMissionString(last_control_points_, i, last_Tf_);
        publishers_[name].publish(msg);

        ROS_INFO_STREAM("Mission prepared for " << name);
        i++;
    }
}

bool DistributedMotionPlanningNode::deployMissionService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if(last_control_points_.size() == 0) {
        res.success = false;
        res.message = "No optimized trajectory available!";
        return true;
    }

    int i = 0;
    for(const auto &name : selected_vehicle_names_) {
        std_msgs::String msg;
        msg.data = formatMissionString(last_control_points_, i, last_Tf_);
        publishers_[name].publish(msg);
        ROS_INFO_STREAM("Mission deployed for " << name << ": " << msg.data);
        i++;
    }

    res.success = true;
    res.message = "Missions deployed to all vehicles.";
    return true;
}

// @.@ Format the mission string for a vehicle
std::string DistributedMotionPlanningNode::formatMissionString(const Eigen::Tensor<double, 3> &controlPoints, int vehicleIndex, double Tf_opt) {
    std::ostringstream oss;
    std::string local_reference = selected_vehicle_names_.front();
    vehicle_State ref_state = vehicle_states_[local_reference];
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
int main(int argc, char** argv)
{
    ros::init(argc, argv, "interactive_motion_planning_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Instantiate node
    DistributedMotionPlanningNode node(&nh, &nh_private);

    ROS_INFO("Interactive Motion Planning Node started. Waiting for service calls...");

    ros::spin(); 

    return 0;
}