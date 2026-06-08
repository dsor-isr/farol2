#include "ros/interactive_planner_node.hpp"
#include <iomanip>

InteractivePlannerNode::InteractivePlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("interactive_motion_planning_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Interactive Motion Planning Node...");

    LoadParameters();

    initialisePublishers();
    initialiseServices();

    // Create timer for alive status (1 second interval)
    alive_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&InteractivePlannerNode::publishAliveStatus, this));


    RCLCPP_INFO(this->get_logger(), "Interactive Motion Planning Node initialized successfully.");
}

InteractivePlannerNode::~InteractivePlannerNode() {
    if (opt_thread_.joinable()) {
        opt_thread_.join();
    }
}

void InteractivePlannerNode::LoadParameters() {
    // Load Bezier parameters
    bezier_degree_ = this->declare_parameter<int>("bezier_degree", 9);
    guess_degree_ = this->declare_parameter<int>("guess_degree", 7);
    number_sample_Pts_ = this->declare_parameter<int>("number_sample_points", 100);
    
    // Load constraint flags
    constr_flags_ = this->declare_parameter<std::vector<bool>>("constraint_flags", std::vector<bool>{true, false, false, false});
    
    // Load nSplit
    nSplit_ = this->declare_parameter<std::vector<int64_t>>("n_split", std::vector<int64_t>{1, 1, 1});
    
    // Load velocity bounds
    bounds_.vel_min = this->declare_parameter<double>("vel_min", 0.001);
    bounds_.vel_max = this->declare_parameter<double>("vel_max", 0.3);
    
    // Load acceleration bounds
    bounds_.acc_min = this->declare_parameter<double>("acc_min", -0.03);
    bounds_.acc_max = this->declare_parameter<double>("acc_max", 0.03);
    
    // Load angular velocity bounds
    bounds_.ang_vel_min = this->declare_parameter<double>("ang_vel_min", -0.15);
    bounds_.ang_vel_max = this->declare_parameter<double>("ang_vel_max", 0.15);
    
    // Load angular acceleration bounds
    bounds_.ang_acc_min = this->declare_parameter<double>("ang_acc_min", -0.005);
    bounds_.ang_acc_max = this->declare_parameter<double>("ang_acc_max", 0.005);
    
    // Load obstacle bounds
    bounds_.obs_min = this->declare_parameter<double>("obs_min", 2.0);
    double obs_max_param = this->declare_parameter<double>("obs_max", 1000.0);
    bounds_.obs_max = (obs_max_param > 999) ? std::numeric_limits<double>::infinity() : obs_max_param;
    
    // Load multi-vehicle parameters
    bounds_.radius = this->declare_parameter<double>("radius", 1.5);
    RADIUS = bounds_.radius;
    
    // Load cost function weights
    bounds_.alpha = this->declare_parameter<double>("alpha", 1.0);
    bounds_.beta = this->declare_parameter<double>("beta", 5.0);
    bounds_.gamma = this->declare_parameter<double>("gamma", 1.0);
    
    bounds_.is_set = true;
    
    // Log loaded parameters
    RCLCPP_INFO(this->get_logger(), "=== Motion Planning Parameters Loaded ===");
    RCLCPP_INFO_STREAM(this->get_logger(), "Bezier degree: " << bezier_degree_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Guess degree: " << guess_degree_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Sample points: " << number_sample_Pts_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Velocity bounds: [" << bounds_.vel_min << ", " << bounds_.vel_max << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "Radius: " << bounds_.radius);
    RCLCPP_INFO_STREAM(this->get_logger(), "Cost weights - alpha: " << bounds_.alpha 
                    << ", beta: " << bounds_.beta << ", gamma: " << bounds_.gamma);
}

void InteractivePlannerNode::initialisePublishers() {
    // This function can be used to initialize any additional publishers if needed
    // Create publishers
    planning_alive_pub_ = this->create_publisher<std_msgs::msg::Bool>(TOPIC_PUB_ALIVE, rclcpp::QoS(1).transient_local());
    mission_log_pub_ = this->create_publisher<std_msgs::msg::String>(TOPIC_PUB_LOG, rclcpp::QoS(10));
}

void InteractivePlannerNode::initialiseServices() {
    // This function can be used to initialize any subscribers if needed
    set_goal_srv_ = this->create_service<farol2_motion_planning::srv::SetState>(
        TOPIC_SRV_SET_GOAL,
        std::bind(&InteractivePlannerNode::setGoalService, this, std::placeholders::_1, std::placeholders::_2));

    set_states_srv_ = this->create_service<farol2_motion_planning::srv::SetState>(
        TOPIC_SRV_SET_STATES,
        std::bind(&InteractivePlannerNode::setStatesService, this, std::placeholders::_1, std::placeholders::_2));

    set_bounds_srv_ = this->create_service<farol2_motion_planning::srv::SetBoundsAndGains>(
        TOPIC_SRV_SET_BOUNDS,
        std::bind(&InteractivePlannerNode::setBoundsService, this, std::placeholders::_1, std::placeholders::_2));

    set_bezier_params_srv_ = this->create_service<farol2_motion_planning::srv::SetBezierParams>(
        TOPIC_SRV_SET_BEZIER_PARAMS,
        std::bind(&InteractivePlannerNode::setBezierParamsService, this, std::placeholders::_1, std::placeholders::_2));

    set_obstacles_srv_ = this->create_service<farol2_motion_planning::srv::SetObstacles>(
        TOPIC_SRV_SET_OBSTACLES,
        std::bind(&InteractivePlannerNode::setObstaclesService, this, std::placeholders::_1, std::placeholders::_2));

    get_planner_config_srv_ = this->create_service<farol2_motion_planning::srv::GetPlannerConfig>(
        TOPIC_SRV_GET_PLANNER_CONFIG,
        std::bind(&InteractivePlannerNode::getPlannerConfigService, this, std::placeholders::_1, std::placeholders::_2));

    run_optimization_srv_ = this->create_service<farol2_motion_planning::srv::RunOptimization>(
        TOPIC_SRV_RUN_OPTIMIZATION,
        std::bind(&InteractivePlannerNode::runOptimizationService, this, std::placeholders::_1, std::placeholders::_2));

    cancel_optimization_srv_ = this->create_service<std_srvs::srv::Trigger>(
        TOPIC_SRV_CANCEL_OPTIMIZATION,
        std::bind(&InteractivePlannerNode::cancelOptimizationService, this, std::placeholders::_1, std::placeholders::_2));
}

void InteractivePlannerNode::publishAliveStatus() {
    auto msg = std::make_shared<std_msgs::msg::Bool>();
    msg->data = true;
    planning_alive_pub_->publish(*msg);
}

void InteractivePlannerNode::publishLog(const std::string& text) {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = text;
    mission_log_pub_->publish(*msg);
}

void InteractivePlannerNode::processState(const vehicle_state  msg, const std::string& vehicle_name){
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Check if this vehicle is already known
    if (std::find(vehicle_names_.begin(), vehicle_names_.end(), vehicle_name) == vehicle_names_.end()) {
        // New vehicle → register it
        vehicle_names_.push_back(vehicle_name);
        RCLCPP_INFO_STREAM(this->get_logger(), "New vehicle registered: " << vehicle_name);

        // Create its trajectory publisher
        traj_pubs_[vehicle_name] = this->create_publisher<nav_msgs::msg::Path>(
            "/" + vehicle_name + "/planned_path", rclcpp::QoS(1).transient_local());

        // Also create mission publisher
        std::string mission_topic = "/" + vehicle_name + "/planning_console/Mission_String";
        // Note: Publishers map not used in ROS2 version, but keeping for compatibility

        // Control points topic
        std::string cp_topic = "/" + vehicle_name + "/Stamped_controlPoints";
        control_points_pubs_[vehicle_name] = this->create_publisher<farol2_motion_planning::msg::StampedMatrix>(
            cp_topic, rclcpp::QoS(1).transient_local());
    }

    // Store/update its state
    vehicle_state rel_state = {
        msg.y,                // Northing
        msg.x,                // Easting
        msg.theta,            // Yaw
        std::max(msg.v, 0.02) // Velocity
    };

    vehicle_states_[vehicle_name] = rel_state;

    RCLCPP_INFO_STREAM(this->get_logger(), "Received state for " << vehicle_name
                    << ": X(E)=" << msg.x
                    << ", Y(N)=" << msg.y
                    << ", Yaw=" << msg.theta
                    << ", u=" << msg.v);
}

void InteractivePlannerNode::setStatesService(const std::shared_ptr<farol2_motion_planning::srv::SetState::Request> req,
                                               std::shared_ptr<farol2_motion_planning::srv::SetState::Response> res)
{
    // Build a temporary vehicle_state message
    vehicle_state msg;
    msg.x   = req->e;     // Easting
    msg.y   = req->n;     // Northing
    msg.theta = req->theta;
    msg.v   = req->v;

    // Store it in vehicle_states_ map
    processState(msg, req->vehicle_name);

    res->success = true;
    res->message = "Vehicle state stored for " + req->vehicle_name;
    publishLog(res->message);
}

void InteractivePlannerNode::setGoalService(const std::shared_ptr<farol2_motion_planning::srv::SetState::Request> req,
                                             std::shared_ptr<farol2_motion_planning::srv::SetState::Response> res)
{
    if(std::find(vehicle_names_.begin(), vehicle_names_.end(), req->vehicle_name) == vehicle_names_.end()) {
        res->success = false;
        res->message = "Vehicle not found!";
        return;
    }
    if(req->v <= 0) {
        res->success = false;
        res->message = "Invalid velocity!";
        return;
    }

    vehicle_state abs_goal;
    abs_goal.x     = req->n; // Northing
    abs_goal.y     = req->e; // Easting
    abs_goal.theta = req->theta;
    abs_goal.v     = req->v;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        goal_states_map_[req->vehicle_name] = abs_goal;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal for " << req->vehicle_name
                    << ": X(E)=" << abs_goal.x
                    << ", Y(N)=" << abs_goal.y
                    << ", Yaw=" << abs_goal.theta
                    << ", u=" << abs_goal.v);

    res->success = true;
    res->message = "Goal set for " + req->vehicle_name;
    publishLog("Goal set for " + req->vehicle_name);
}

void InteractivePlannerNode::setBoundsService(const std::shared_ptr<farol2_motion_planning::srv::SetBoundsAndGains::Request> req,
                                               std::shared_ptr<farol2_motion_planning::srv::SetBoundsAndGains::Response> res)
{
    bounds_.vel_min     = req->vel_min;
    bounds_.vel_max     = req->vel_max;
    bounds_.acc_min     = req->acc_min;
    bounds_.acc_max     = req->acc_max;
    bounds_.ang_vel_min = req->ang_vel_min;
    bounds_.ang_vel_max = req->ang_vel_max;
    bounds_.ang_acc_min = req->ang_acc_min;
    bounds_.ang_acc_max = req->ang_acc_max;
    bounds_.obs_min     = req->obs_min;
    if (req->obs_max < 10000) {
        bounds_.obs_max = req->obs_max;
    }
    else {
        bounds_.obs_max = std::numeric_limits<double>::infinity();
    }
    bounds_.radius      = req->radius;
    bounds_.alpha       = req->alpha;
    bounds_.beta        = req->beta;
    bounds_.gamma       = req->gamma;

    bounds_.is_set = true;

    RADIUS = req->radius;
    res->success = true;
    res->message = "Bounds and gains set.";
    publishLog(res->message);
    RCLCPP_INFO_STREAM(this->get_logger(), res->message);
}

void InteractivePlannerNode::setBezierParamsService(const std::shared_ptr<farol2_motion_planning::srv::SetBezierParams::Request> req,
                                                     std::shared_ptr<farol2_motion_planning::srv::SetBezierParams::Response> res)
{
    if (req->constr_flags.size() != 4) {
        res->success = false;
        res->message = "constr_flags must have size 4";
        RCLCPP_ERROR_STREAM(this->get_logger(), res->message);
        return;
    }

    if (req->n_split.empty()) {
        res->success = false;
        res->message = "nSplit cannot be empty";
        RCLCPP_ERROR_STREAM(this->get_logger(), res->message);
        return;
    }

    if (req->bezier_degree < 4) {
        res->success = false;
        res->message = "bezier_degree must be at least 4";
        RCLCPP_ERROR_STREAM(this->get_logger(), res->message);
        return;
    }
    if (req->number_sample_pts < 20) {
        res->success = false;
        res->message = "Path must have at least 20 sample points";
        RCLCPP_ERROR_STREAM(this->get_logger(), res->message);
        return;
    }

    bezier_degree_ = req->bezier_degree;
    guess_degree_ = req->guess_degree;
    nSplit_ = req->n_split;
    constr_flags_ = req->constr_flags;
    number_sample_Pts_ = req->number_sample_pts;

    RCLCPP_INFO_STREAM(this->get_logger(), "Bezier params updated:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  degree: " << bezier_degree_);

    std::ostringstream ns;
    ns << "  nSplit: ";
    for (auto v : nSplit_) ns << v << " ";
    RCLCPP_INFO_STREAM(this->get_logger(), ns.str());

    std::ostringstream cf;
    cf << "  constr_flags: ";
    for (auto v : constr_flags_) cf << (v ? "true " : "false ");
    RCLCPP_INFO_STREAM(this->get_logger(), cf.str());

    res->success = true;
    res->message = "Bezier parameters updated successfully";
    publishLog(res->message);
}

void InteractivePlannerNode::getPlannerConfigService(const std::shared_ptr<farol2_motion_planning::srv::GetPlannerConfig::Request> req,
                                                      std::shared_ptr<farol2_motion_planning::srv::GetPlannerConfig::Response> res)
{
    // Bounds
    res->vel_min = bounds_.vel_min;
    res->vel_max = bounds_.vel_max;
    res->acc_min = bounds_.acc_min;
    res->acc_max = bounds_.acc_max;
    res->ang_vel_min = bounds_.ang_vel_min;
    res->ang_vel_max = bounds_.ang_vel_max;
    res->ang_acc_min = bounds_.ang_acc_min;
    res->ang_acc_max = bounds_.ang_acc_max;
    res->obs_min = bounds_.obs_min;
    res->obs_max = bounds_.obs_max;
    res->radius  = bounds_.radius;

    res->alpha = bounds_.alpha;
    res->beta  = bounds_.beta;
    res->gamma = bounds_.gamma;

    // Bezier
    res->bezier_degree = bezier_degree_;
    res->guess_degree  = guess_degree_;
    res->n_split       = nSplit_;
    res->constr_flags  = constr_flags_;
    res->number_sample_pts = number_sample_Pts_;
}

void InteractivePlannerNode::setObstaclesService(const std::shared_ptr<farol2_motion_planning::srv::SetObstacles::Request> req,
                                                  std::shared_ptr<farol2_motion_planning::srv::SetObstacles::Response> res)
{
    // --- Circular obstacles ---
    size_t n_circ = req->circ_obs.size() / 3;
    
    if (n_circ) {
        circ_obs_.resize(3, n_circ); // circ_obs  # [E1, N1, r1, E2, N2, r2, ...]

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            for (size_t i = 0; i < n_circ; ++i) {
                circ_obs_(0, i) = req->circ_obs[i*3 + 1]; // Northing
                circ_obs_(1, i) = req->circ_obs[i*3 + 0]; // Easting
                circ_obs_(2, i) = req->circ_obs[i*3 + 2];
            }
        }

        obstacles_updated_ = true;
    }
    else {
        circ_obs_.resize(3, 0);
    }
    
    

    // --- Line obstacles --- (TODO... - just a placeholder)
    size_t n_line = req->line_obs.size() / 3;
    if (n_line) {
        line_obs_.resize(3, n_line);
        for (size_t i = 0; i < n_line; ++i) {
            line_obs_(1, i) = req->line_obs[i*3 + 0]; // x1
            line_obs_(0, i) = req->line_obs[i*3 + 1]; // y1
            line_obs_(2, i) = req->line_obs[i*3 + 2]; // x2 or other line info
        }
    }
    else {
        line_obs_.resize(3, 0);
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

    res->success = true;
    res->message = msg.str();

    RCLCPP_INFO_STREAM(this->get_logger(), res->message);
    publishLog(res->message);
}

/**
 * @brief Runs the multi-vehicle trajectory optimization service.
 */
void InteractivePlannerNode::runOptimizationService(const std::shared_ptr<farol2_motion_planning::srv::RunOptimization::Request> req,
                                                     std::shared_ptr<farol2_motion_planning::srv::RunOptimization::Response> res)
{
    // Basic checks
    if(vehicle_names_.empty()) {
        res->success = false;
        res->message = "No vehicles discovered!";
        publishLog(res->message);
        return;
    }

    if(vehicle_states_.empty()) {
        res->success = false;
        res->message = "Cannot run optimization: no vehicle states sampled yet.";
        publishLog(res->message);
        return;
    }

    // Check goals
    for(const auto &name : vehicle_names_) {
        if(goal_states_map_.count(name) == 0) {
            res->success = false;
            res->message = "Cannot run optimization: goal not defined for vehicle " + name;
            RCLCPP_WARN_STREAM(this->get_logger(), res->message);
            publishLog(res->message);
            return;
        }
    }

    publishLog("Setting up optimization problem");
    std::vector<vehicle_state> current_states;
    std::vector<vehicle_state> goal_states;
    std::vector<std::string> vehicles;
    vehicles.clear();
    current_states.clear();
    goal_states.clear();

    int NVehicles;
    
    // Copy shared data
    // If request empty → default to all vehicles
    if (req->vehicle_names.empty()) {
        vehicles = vehicle_names_;
    } else {
        for (const auto& name : req->vehicle_names) {
            if (std::find(vehicle_names_.begin(),
                        vehicle_names_.end(),
                        name) == vehicle_names_.end()) {
                res->success = false;
                res->message = "Vehicle not found: " + name;
                return;
            }
            vehicles.push_back(name);
        }
    }
    selected_vehicle_names_ = vehicles;
    NVehicles = vehicles.size();
    std::string local_reference = vehicles.front();
    vehicle_state ref_state = vehicle_states_[local_reference];

    for(const auto &name : vehicles) {
        vehicle_state current_state;
        current_state.x = vehicle_states_[name].x-ref_state.x; // Northing
        current_state.y = vehicle_states_[name].y-ref_state.y; // Easting
        current_state.theta = vehicle_states_[name].theta;
        current_state.v = vehicle_states_[name].v;
        current_states.push_back(current_state);

        vehicle_state goal_state;
        goal_state.x = goal_states_map_[name].x- ref_state.x; // Northing
        goal_state.y = goal_states_map_[name].y- ref_state.y; // Easting
        goal_state.theta = goal_states_map_[name].theta;
        goal_state.v = goal_states_map_[name].v;
        
        goal_states.push_back(goal_state);
    }

    // Convert constraint flags from bool to uint8_t for MultipleVehiclePlanner
    std::vector<uint8_t> constr_flags_uint8;
    for (bool flag : constr_flags_) {
        constr_flags_uint8.push_back(static_cast<uint8_t>(flag ? 1 : 0));
    }

    // Initialize solver
    MultipleVehiclePlanner solver_first(guess_degree_, 1, {1,1,1}, constr_flags_uint8);

    if(bounds_.is_set) {
        RCLCPP_INFO(this->get_logger(), "Bounds are no longer Default.");
        solver_first.setBoundsAndGains(
            bounds_.vel_min, bounds_.vel_max,
            bounds_.acc_min, bounds_.acc_max,
            bounds_.ang_vel_min, bounds_.ang_vel_max,
            bounds_.ang_acc_min, bounds_.ang_acc_max,
            bounds_.obs_min, bounds_.obs_max,
            bounds_.radius,
            bounds_.alpha, bounds_.beta, bounds_.gamma
        );
    }

    double Tf_guess = -std::numeric_limits<double>::infinity();
    Eigen::Tensor<double, 3> all_control_points;
    std::vector<double> Tf_values(NVehicles, 0.0);
    cancel_flag_ = false;

    for (int i = 0; i < NVehicles; ++i) {
        solver_first.setupSymbolic();
        solver_first.computeCostFunction();
        std::vector<vehicle_state> current_state_vec;
        std::vector<vehicle_state> goal_state_vec;

        current_state_vec.push_back(current_states[i]);
        goal_state_vec.push_back(goal_states[i]);

        if(obstacles_updated_) {
            Eigen::MatrixXd adjusted_circ_obs(3, circ_obs_.cols());
            for (int k = 0; k < circ_obs_.cols(); ++k) {
                adjusted_circ_obs(0, k) = circ_obs_(0, k) - ref_state.x;  // Northing
                adjusted_circ_obs(1, k) = circ_obs_(1, k) - ref_state.y;  // Easting
                adjusted_circ_obs(2, k) = circ_obs_(2, k);                // radius stays the same
                std::cout << "Obs " << k 
                    << " | X: " << circ_obs_(0, k)
                    << " Y: " << circ_obs_(1, k)
                    << " R: " << circ_obs_(2, k)
                    << "\n";
            }
            solver_first.setOptimizationProblem(current_state_vec, goal_state_vec, adjusted_circ_obs, line_obs_);
        } else {
            RCLCPP_INFO(this->get_logger(), "No obstacle updates detected.");
            solver_first.setOptimizationProblem(current_state_vec, goal_state_vec);
        }

        solver_first.createConstraintVector();
        solver_first.createDecisionVector();
        publishLog("Starting Guess calculation");
        solver_first.solveOptimizationProblem(&cancel_flag_);

        Eigen::Tensor<double, 3> cp_raw = solver_first.getControlPoints();

        int dim1 = cp_raw.dimension(1); // should be 2
        int dim2 = cp_raw.dimension(2); // N control points

        if (i == 0) {
            all_control_points = Eigen::Tensor<double, 3>(NVehicles, dim1, dim2);
        }

        std::cout << "Vehicle " << i << " Optimal control points:\n";

        for (int d = 0; d < dim1; ++d) {
            for (int n = 0; n < dim2; ++n) {
                all_control_points(i, d, n) = cp_raw(0, d, n);
                std::cout << all_control_points(i, d, n) << " ";
            }
            std::cout << "\n";
        }

        Tf_values[i] = solver_first.getTf();
        Tf_guess = std::max(Tf_guess, Tf_values[i]);
        std::cout << "Obtained Tf for vehicle " << i << ":\n" << solver_first.getTf() << "\n";
    }

    Eigen::Tensor<double, 3> control_points_guess = BezierUtils::elevateTensorDegree(all_control_points, bezier_degree_-guess_degree_);

    publishControlPoints(all_control_points, Tf_values, true);
    std::vector<std::pair<int,int>> collisions;
    bool collision_detected = false;

    for (int i = 0; i < NVehicles; ++i) {
        for (int j = i + 1; j < NVehicles; ++j) {
            Eigen::MatrixXd P1 = BezierUtils::tensor_2_matrix(control_points_guess, i);
            Eigen::MatrixXd P2 = BezierUtils::tensor_2_matrix(control_points_guess, j);

            if (!BezierUtils::check_vehicle_distances(P1, P2, RADIUS, nSplit_[2])) {
                collisions.emplace_back(i, j);
                collision_detected = true;

                RCLCPP_WARN_STREAM(this->get_logger(), "Collision detected between Vehicle "
                                << i << " and Vehicle " << j << "!");
            }
        }
    }

    if (collision_detected) {
        std::vector<std::pair<int,int>> sorted_collisions = collisions;

        std::sort(sorted_collisions.begin(), sorted_collisions.end(),
            [&](auto& a, auto& b) {
                double scoreA = Tf_values[a.first] + Tf_values[a.second];
                double scoreB = Tf_values[b.first] + Tf_values[b.second];
                return scoreA > scoreB; // prioritize high Tf conflicts
            });

        std::vector<int> to_remove = selectTrajectoriesToRemove(NVehicles, sorted_collisions, Tf_values);
        
        double Tf_min = 0.0;
        for (int i = 0; i < NVehicles; ++i) {
            if (std::find(to_remove.begin(), to_remove.end(), i) == to_remove.end()) {
                Tf_min = std::max(Tf_min, Tf_values[i]);
            } 
        }

        RCLCPP_WARN_STREAM(this->get_logger(), "Trajectories to remove:");
        for (int idx : to_remove) {
            RCLCPP_WARN_STREAM(this->get_logger(), " - Vehicle " << idx
                            << " (Tf = " << Tf_values[idx] << ")");
        }
        RCLCPP_WARN_STREAM(this->get_logger(), "Minimum Tf among kept trajectories: " << Tf_min);

        // Convert constraint flags from bool to uint8_t for MultipleVehiclePlanner
        std::vector<uint8_t> constr_flags_uint8;
        for (bool flag : constr_flags_) {
            constr_flags_uint8.push_back(static_cast<uint8_t>(flag ? 1 : 0));
        }

        // Convert nSplit_ from int64_t to int
        std::vector<int> nSplit_int(nSplit_.begin(), nSplit_.end());


        MultipleVehiclePlanner solver(bezier_degree_, NVehicles, nSplit_int, constr_flags_uint8);
        if(bounds_.is_set) {
            RCLCPP_INFO(this->get_logger(), "Bounds are no longer Default.");
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

        cancel_flag_ = false;
        opt_running_ = true;
        
        // Lambda capturing necessary data for the optimization thread
        auto optimization_task = [this, req, vehicles, ref_state, current_states, goal_states, 
                                   Tf_guess, control_points_guess, to_remove, Tf_min, 
                                   solver = std::move(solver)]() mutable {
            try {
                printControlPoints(control_points_guess);

                if(obstacles_updated_) {
                    Eigen::MatrixXd adjusted_circ_obs(3, circ_obs_.cols());
                    for (int i = 0; i < circ_obs_.cols(); ++i) {
                        adjusted_circ_obs(0, i) = circ_obs_(0, i) - ref_state.x;
                        adjusted_circ_obs(1, i) = circ_obs_(1, i) - ref_state.y;
                        adjusted_circ_obs(2, i) = circ_obs_(2, i);
                        RCLCPP_INFO_STREAM(this->get_logger(), "| Obstacle " << i << ":    Easting =  " 
                            << adjusted_circ_obs(1, i) << ", Northing = " << adjusted_circ_obs(0, i) 
                            << ", r = " << adjusted_circ_obs(2, i) << " |");
                    }
                    solver.setOptimizationProblem(current_states, goal_states, adjusted_circ_obs, line_obs_, control_points_guess, Tf_guess, false);
                } else {
                    RCLCPP_INFO(this->get_logger(), "No obstacle updates detected.");
                    Eigen::Matrix<double, 3, Eigen::Dynamic> empty_obs(3, 0);
                    solver.setOptimizationProblem(current_states, goal_states, empty_obs, empty_obs, control_points_guess, Tf_guess, false);
                }

                solver.createConstraintVector(to_remove);
                solver.createDecisionVector(to_remove, Tf_min);
                publishLog("Starting Optimization");
                solver.solveOptimizationProblem(&cancel_flag_);

                last_control_points_ = solver.getControlPoints();
                last_Tf_ = solver.getTf();
                std::vector<double> Tf_equal(vehicle_names_.size(), last_Tf_);

                publishControlPoints(last_control_points_, Tf_equal, false);

                RCLCPP_INFO_STREAM(this->get_logger(), "=== Optimization Results ===");
                RCLCPP_INFO_STREAM(this->get_logger(), "Final time Tf = " << last_Tf_);

                int num_vehicles = last_control_points_.dimension(0);
                int num_points   = last_control_points_.dimension(2);

                for(int v = 0; v < num_vehicles; ++v) {
                    std::ostringstream oss;
                    oss << "Vehicle " << vehicle_names_[v] << " control points:\n";

                    for(int d = 0; d < 2; ++d) {
                        oss << (d == 0 ? "  X: " : "  Y: ");
                        for(int p = 0; p < num_points; ++p) {
                            oss << last_control_points_(v, d, p) << " ";
                        }
                        oss << "\n";
                    }

                    RCLCPP_INFO_STREAM(this->get_logger(), oss.str());
                }

                publishLog(solver.getOptimizationstatus());

                // Publish planned paths
                Eigen::Tensor<double, 3> trajs = solver.getOptimalTrajectories(number_sample_Pts_);
                for (size_t i = 0; i < vehicles.size(); ++i) {
                    auto path = std::make_shared<nav_msgs::msg::Path>();
                    path->header.stamp = this->now();
                    path->header.frame_id = "map";

                    for (size_t j = 0; j < trajs.dimension(2); ++j) {
                        geometry_msgs::msg::PoseStamped p;
                        p.header = path->header;

                        p.pose.position.x = trajs(i, 0, j) + ref_state.x;
                        p.pose.position.y = trajs(i, 1, j) + ref_state.y;
                        p.pose.position.z = 0.0;

                        path->poses.push_back(p);
                    }

                    traj_pubs_[vehicles[i]]->publish(*path);
                }

                publishMissions();
                opt_running_ = false;

            } catch (const std::exception& e) {
                RCLCPP_WARN_STREAM(this->get_logger(), "Optimization thread stopped: " << e.what());
                publishLog(std::string("Optimization thread stopped: ") + e.what());
                opt_running_ = false;
            }
        };

        // Detach thread for async optimization
        if (opt_thread_.joinable()) {
            opt_thread_.join();
        }
        opt_thread_ = std::thread(optimization_task);
        opt_thread_.detach();

    } else {
        // No collisions - publish paths directly
        int numPoints = 100;
        Eigen::Tensor<double, 3> trajs(control_points_guess.dimension(0), control_points_guess.dimension(1), numPoints);
        const auto& CP = control_points_guess;

        for(int i = 0; i < control_points_guess.dimension(0); ++i){
            Eigen::MatrixXd pts(2, control_points_guess.dimension(2));
            for (int k = 0; k <= control_points_guess.dimension(2) - 1; ++k) {
                pts(0, k) = CP(i, 0, k);
                pts(1, k) = CP(i, 1, k);
            }
            for (int j = 0; j < numPoints; ++j){
                double u = static_cast<double>(j) / (numPoints - 1);
                Eigen::VectorXd point = BezierUtils::deCasteljau(u, pts);
                trajs(i, 0, j) = point(0);
                trajs(i, 1, j) = point(1);
            }
        }

        for (size_t i = 0; i < vehicles.size(); ++i) {
            auto path = std::make_shared<nav_msgs::msg::Path>();
            path->header.stamp = this->now();
            path->header.frame_id = "map";

            for (size_t j = 0; j < trajs.dimension(2); ++j) {
                geometry_msgs::msg::PoseStamped p;
                p.header = path->header;

                p.pose.position.x = trajs(i, 0, j) + ref_state.x;
                p.pose.position.y = trajs(i, 1, j) + ref_state.y;
                p.pose.position.z = 0.0;

                path->poses.push_back(p);
            }

            traj_pubs_[vehicles[i]]->publish(*path);
        }

        last_control_points_ = control_points_guess;
        last_Tf_ = Tf_guess;
        std::vector<double> Tf_equal(vehicle_names_.size(), last_Tf_);

        publishControlPoints(last_control_points_, Tf_equal, false);
        publishMissions();
        opt_running_ = false;
    }

    res->success = true;
    res->message = "Optimization started";
}

void InteractivePlannerNode::cancelOptimizationService(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (!opt_running_) {
        res->success = false;
        res->message = "No optimization running";
        return;
    }

    cancel_flag_ = true;
    res->success = true;
    res->message = "Optimization abort requested";

    RCLCPP_WARN(this->get_logger(), "Optimization abort requested by user");
}

void InteractivePlannerNode::publishMissions()
{
    if(last_control_points_.size() == 0) {
        RCLCPP_WARN(this->get_logger(), "No optimized trajectory available!");
        return;
    }

    int i = 0;
    for(const auto &name : selected_vehicle_names_) {
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = formatMissionString(last_control_points_, i, last_Tf_);
        // Note: Publishing via control_points_pubs_ would be better
        // For now, missions are available through the control points
        RCLCPP_INFO_STREAM(this->get_logger(), "Mission prepared for " << name);
        i++;
    }
}

std::string InteractivePlannerNode::formatMissionString(const Eigen::Tensor<double, 3> &controlPoints, int vehicleIndex, double Tf_opt) {
    std::ostringstream oss;
    std::string local_reference = selected_vehicle_names_.front();
    vehicle_state ref_state = vehicle_states_[local_reference];
    oss << std::fixed << std::setprecision(3);

    oss << "3\n" << std::fixed << std::setprecision(3)  << ref_state.x << " " << ref_state.y << " 29S\nBEZIER " << controlPoints.dimension(2) << " ";

    for (int j = 0; j < controlPoints.dimension(2); ++j) {
        oss << controlPoints(vehicleIndex, 0, j) << " ";  // X values
    }
    for (int j = 0; j < controlPoints.dimension(2); ++j) {
        oss << controlPoints(vehicleIndex, 1, j) << " ";  // Y values
    }

    oss << Tf_opt;

    return oss.str();
}

std::vector<int> InteractivePlannerNode::selectTrajectoriesToRemove(int NVehicles, const std::vector<std::pair<int,int>>& collisions, const std::vector<double>& Tf_values)
{
    std::vector<bool> removed(NVehicles, false);
    std::vector<bool> bestRemoved(NVehicles, false);

    int bestCount = INT_MAX;
    double bestTfSum = -1.0;

    std::function<void()> dfs = [&]() {
        int currentCount = 0;
        double currentTfSum = 0.0;

        for (int i = 0; i < NVehicles; ++i) {
            if (removed[i]) {
                currentCount++;
                currentTfSum += Tf_values[i];
            }
        }

        if (currentCount > bestCount) return;

        for (const auto& [u, v] : collisions) {
            if (!removed[u] && !removed[v]) {
                removed[u] = true;
                dfs();
                removed[u] = false;

                removed[v] = true;
                dfs();
                removed[v] = false;

                return;
            }
        }

        if (currentCount < bestCount ||
           (currentCount == bestCount && currentTfSum > bestTfSum))
        {
            bestCount = currentCount;
            bestTfSum = currentTfSum;
            bestRemoved = removed;
        }
    };

    dfs();

    std::vector<int> result;
    for (int i = 0; i < NVehicles; ++i) {
        if (bestRemoved[i]) result.push_back(i);
    }

    return result;
}

void InteractivePlannerNode::publishControlPoints(const Eigen::Tensor<double, 3>& tensor, const std::vector<double>& tf_values, bool is_guess)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    int n_vehicles = tensor.dimension(0);
    int dim1 = tensor.dimension(1);
    int dim2 = tensor.dimension(2);

    for (int i = 0; i < n_vehicles; ++i) {
        if (i >= vehicle_names_.size()) continue;

        const std::string& vehicle_name = vehicle_names_[i];

        auto it = control_points_pubs_.find(vehicle_name);
        if (it == control_points_pubs_.end()) continue;

        auto msg = std::make_shared<farol2_motion_planning::msg::StampedMatrix>();

        msg->header.stamp = this->now();
        msg->header.frame_id = "map";

        msg->n_vehicles = 1;
        msg->dim1 = dim1;
        msg->dim2 = dim2;

        msg->data.reserve(dim1 * dim2);

        for (int d = 0; d < dim1; ++d) {
            for (int n = 0; n < dim2; ++n) {
                msg->data.push_back(tensor(i, d, n));
            }
        }

        msg->tf_values = { tf_values[i] };
        msg->is_guess = is_guess;

        it->second->publish(*msg);
    }
}

void printControlPoints(const Eigen::Tensor<double, 3>& cp)
{
    const int num_vehicles = cp.dimension(0);
    const int dim2 = cp.dimension(1);
    const int num_points = cp.dimension(2);

    for (int v = 0; v < num_vehicles; ++v)
    {
        std::cout << "Vehicle " << v << " control points:\n";

        for (int d = 0; d < dim2; ++d)
        {
            if (d == 0) std::cout << "  X: ";
            else if (d == 1) std::cout << "  Y: ";
            else std::cout << "  dim" << d << ": ";

            for (int p = 0; p < num_points; ++p)
            {
                std::cout << cp(v, d, p) << " ";
            }
            std::cout << "\n";
        }

        std::cout << std::endl;
    }
}

// Main function for ROS2
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InteractivePlannerNode>();

    RCLCPP_INFO(node->get_logger(), "Interactive Motion Planning Node started. Waiting for service calls...");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}