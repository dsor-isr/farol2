#include "underway/distributed_planner_node.hpp"
#include <iomanip>

DistributedPlannerNode::DistributedPlannerNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("distributed_motion_planning_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Distributed Motion Planning Node...");

    LoadParameters();

    initialisePublishers();
    initialiseServices();
    initialiseSubscribers();

    // Create timer for alive status (1 second interval)
    alive_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DistributedPlannerNode::publishAliveStatus, this));

    RCLCPP_INFO(this->get_logger(), "Distributed Motion Planning Node initialized successfully.");
}

DistributedPlannerNode::~DistributedPlannerNode()
{
    if (opt_thread_.joinable())
    {
        opt_thread_.join();
    }
}

void DistributedPlannerNode::LoadParameters()
{
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

    use_goal_straight_line_ = this->declare_parameter<bool>("use_goal_straight_line", false);
    use_goal_closer_point_ = this->declare_parameter<bool>("use_goal_closer_point", true);
    cell_margin_ = this->declare_parameter<double>("cell_margin", 2.0);

    // Log loaded parameters
    RCLCPP_INFO(this->get_logger(), "=== Motion Planning Parameters Loaded ===");
    RCLCPP_INFO_STREAM(this->get_logger(), "Bezier degree: " << bezier_degree_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Guess degree: " << guess_degree_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Sample points: " << number_sample_Pts_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Velocity bounds: [" << bounds_.vel_min << ", " << bounds_.vel_max << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "Radius: " << bounds_.radius);
    RCLCPP_INFO_STREAM(this->get_logger(), "Cost weights - alpha: " << bounds_.alpha << ", beta: " << bounds_.beta << ", gamma: " << bounds_.gamma);
}

void DistributedPlannerNode::initialisePublishers()
{
    // This function can be used to initialize any additional publishers if needed
    // Create publishers
    planning_alive_pub_ = this->create_publisher<std_msgs::msg::Bool>(TOPIC_PUB_ALIVE, rclcpp::QoS(1).transient_local());
    mission_log_pub_ = this->create_publisher<std_msgs::msg::String>(TOPIC_PUB_LOG, rclcpp::QoS(10));
    state_pub_ = this->create_publisher<farol2_motion_planning::msg::NeighborPosition>(TOPIC_PUB_STATE, rclcpp::QoS(10));
    traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("/" + vehicle_name_ + "/planned_path", rclcpp::QoS(1).transient_local());
    mission_pub_ = this->create_publisher<std_msgs::msg::String>("/" + vehicle_name_ + "/planning_console/Mission_String", rclcpp::QoS(1).transient_local());
    control_points_pub_= this->create_publisher<farol2_motion_planning::msg::StampedMatrix>("/" + vehicle_name_ + "/Stamped_controlPoints", rclcpp::QoS(1).transient_local());

}

void DistributedPlannerNode::initialiseServices()
{
    // This function can be used to initialize any subscribers if needed
    set_goal_srv_ = this->create_service<farol2_motion_planning::srv::SetState>(
        TOPIC_SRV_SET_GOAL,
        std::bind(&DistributedPlannerNode::setGoalService, this, std::placeholders::_1, std::placeholders::_2));

    set_states_srv_ = this->create_service<farol2_motion_planning::srv::SetState>(
        TOPIC_SRV_SET_STATES,
        std::bind(&DistributedPlannerNode::setStatesService, this, std::placeholders::_1, std::placeholders::_2));

    set_bounds_srv_ = this->create_service<farol2_motion_planning::srv::SetBoundsAndGains>(
        TOPIC_SRV_SET_BOUNDS,
        std::bind(&DistributedPlannerNode::setBoundsService, this, std::placeholders::_1, std::placeholders::_2));

    set_bezier_params_srv_ = this->create_service<farol2_motion_planning::srv::SetBezierParams>(
        TOPIC_SRV_SET_BEZIER_PARAMS,
        std::bind(&DistributedPlannerNode::setBezierParamsService, this, std::placeholders::_1, std::placeholders::_2));

    set_obstacles_srv_ = this->create_service<farol2_motion_planning::srv::SetObstacles>(
        TOPIC_SRV_SET_OBSTACLES,
        std::bind(&DistributedPlannerNode::setObstaclesService, this, std::placeholders::_1, std::placeholders::_2));

    get_planner_config_srv_ = this->create_service<farol2_motion_planning::srv::GetPlannerConfig>(
        TOPIC_SRV_GET_PLANNER_CONFIG,
        std::bind(&DistributedPlannerNode::getPlannerConfigService, this, std::placeholders::_1, std::placeholders::_2));
}

void DistributedPlannerNode::initialiseSubscribers()
{
    neighbor_sub_ = create_subscription<farol2_motion_planning::msg::NeighborPosition>(
        TOPIC_SUB_STATE,
        rclcpp::QoS(10),
        std::bind(&DistributedPlannerNode::neighborCallback, this, std::placeholders::_1)
    );
}

void DistributedPlannerNode::neighborCallback(const farol2_motion_planning::msg::NeighborPosition::ConstSharedPtr msg)
{
    if (msg->vehicle_name == vehicle_name_)
    {
        return;
    }

    if (msg->position.size() != 2)
    {
        RCLCPP_WARN(get_logger(), "Ignoring message from %s: position has %zu elements instead of 2", msg->vehicle_name.c_str(), msg->position.size());
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!distributed_scheme_running_)
    {
        return;
    }

    if (std::find(known_vehicle_names_.begin(), known_vehicle_names_.end(), msg->vehicle_name) == known_vehicle_names_.end())
    {
        known_vehicle_names_.push_back(msg->vehicle_name);
    }

    if (static_cast<int>(msg->curr_iter) != current_iteration_)
    {
        return;
    }

    neighbors_history_[msg->curr_iter][msg->vehicle_name] = Eigen::Vector2d(msg->position[0], msg->position[1]);
}

void DistributedPlannerNode::publishAliveStatus()
{
    auto msg = std::make_shared<std_msgs::msg::Bool>();
    msg->data = true;
    planning_alive_pub_->publish(*msg);
}

void DistributedPlannerNode::publishLog(const std::string &text)
{
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = text;
    mission_log_pub_->publish(*msg);
}

void DistributedPlannerNode::processState(const vehicle_state msg, const std::string &vehicle_name)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Check if this is my vehicle
    if (vehicle_name == vehicle_name_)
    {
        // Store/update its state
        initial_state_ = {
            msg.y,                // Northing
            msg.x,                // Easting
            msg.theta,            // Yaw
            std::max(msg.v, 0.02) // Velocity
        };
        RCLCPP_INFO_STREAM(this->get_logger(), "Received state for " << vehicle_name << ": X(E)=" << msg.x << ", Y(N)=" << msg.y << ", Yaw=" << msg.theta << ", u=" << msg.v);

    }
}

void DistributedPlannerNode::setStatesService(const std::shared_ptr<farol2_motion_planning::srv::SetState::Request> req,
                                              std::shared_ptr<farol2_motion_planning::srv::SetState::Response> res)
{
    // Build a temporary vehicle_state message
    vehicle_state msg;
    msg.x = req->e; // Easting
    msg.y = req->n; // Northing
    msg.theta = req->theta;
    msg.v = req->v;

    // Store it in vehicle_states_ map
    processState(msg, req->vehicle_name);

    res->success = true;
    res->message = "Vehicle state stored for " + req->vehicle_name;
    // publishLog(res->message);
}

void DistributedPlannerNode::setGoalService(const std::shared_ptr<farol2_motion_planning::srv::SetState::Request> req,
                                            std::shared_ptr<farol2_motion_planning::srv::SetState::Response> res)
{
    if (req->vehicle_name == vehicle_name_)
    {
        goal_state_.x = req->n; // Northing
        goal_state_.y = req->e; // Easting
        goal_state_.theta = req->theta;
        goal_state_.v = std::max(req->v, 0.02); // Velocity
    }

    res->success = true;
    res->message = "Goal set for " + req->vehicle_name;
}

void DistributedPlannerNode::setBoundsService(const std::shared_ptr<farol2_motion_planning::srv::SetBoundsAndGains::Request> req,
                                              std::shared_ptr<farol2_motion_planning::srv::SetBoundsAndGains::Response> res)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received bounds and gains:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  vel_min: " << req->vel_min << ", vel_max: " << req->vel_max);
    RCLCPP_INFO_STREAM(this->get_logger(), "  acc_min: " << req->acc_min << ", acc_max: " << req->acc_max);
    RCLCPP_INFO_STREAM(this->get_logger(), "  ang_vel_min: " << req->ang_vel_min << ", ang_vel_max: " << req->ang_vel_max);
    RCLCPP_INFO_STREAM(this->get_logger(), "  ang_acc_min: " << req->ang_acc_min << ", ang_acc_max: " << req->ang_acc_max);
    RCLCPP_INFO_STREAM(this->get_logger(), "  obs_min: " << req->obs_min << ", obs_max: " << req->obs_max);
    RCLCPP_INFO_STREAM(this->get_logger(), "  radius: " << req->radius);
    RCLCPP_INFO_STREAM(this->get_logger(), "  alpha: " << req->alpha << ", beta: " << req->beta << ", gamma: " << req->gamma);
    bounds_.vel_min = req->vel_min;
    bounds_.vel_max = req->vel_max;
    bounds_.acc_min = req->acc_min;
    bounds_.acc_max = req->acc_max;
    bounds_.ang_vel_min = req->ang_vel_min;
    bounds_.ang_vel_max = req->ang_vel_max;
    bounds_.ang_acc_min = req->ang_acc_min;
    bounds_.ang_acc_max = req->ang_acc_max;
    bounds_.obs_min = req->obs_min;
    if (req->obs_max < 10000)
    {
        bounds_.obs_max = req->obs_max;
    }
    else
    {
        bounds_.obs_max = std::numeric_limits<double>::infinity();
    }
    bounds_.radius = req->radius;
    bounds_.alpha = req->alpha;
    bounds_.beta = req->beta;
    bounds_.gamma = req->gamma;

    bounds_.is_set = true;

    RADIUS = req->radius;
    res->success = true;
    res->message = "Bounds and gains set.";
    publishLog(res->message);
    RCLCPP_INFO_STREAM(this->get_logger(), res->message);
}

void DistributedPlannerNode::setBezierParamsService(const std::shared_ptr<farol2_motion_planning::srv::SetBezierParams::Request> req,
                                                    std::shared_ptr<farol2_motion_planning::srv::SetBezierParams::Response> res)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received bezier params:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  degree: " << req->bezier_degree);
    RCLCPP_INFO_STREAM(this->get_logger(), "  guess_degree: " << req->guess_degree);
    RCLCPP_INFO_STREAM(this->get_logger(), "  n_split: ");
    for (const auto &v : req->n_split)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "    " << v);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "  constr_flags: ");
    for (const auto &v : req->constr_flags)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "    " << (v ? "true" : "false"));
    }
    if (req->constr_flags.size() != 4)
    {
        res->success = false;
        res->message = "constr_flags must have size 4";
        RCLCPP_ERROR_STREAM(this->get_logger(), res->message);
        return;
    }

    if (req->n_split.empty())
    {
        res->success = false;
        res->message = "nSplit cannot be empty";
        RCLCPP_ERROR_STREAM(this->get_logger(), res->message);
        return;
    }

    if (req->bezier_degree < 4)
    {
        res->success = false;
        res->message = "bezier_degree must be at least 4";
        RCLCPP_ERROR_STREAM(this->get_logger(), res->message);
        return;
    }

    bezier_degree_ = req->bezier_degree;
    guess_degree_ = req->guess_degree;
    nSplit_ = req->n_split;
    constr_flags_ = req->constr_flags;

    RCLCPP_INFO_STREAM(this->get_logger(), "Bezier params updated:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  degree: " << bezier_degree_);

    std::ostringstream ns;
    ns << "  nSplit: ";
    for (auto v : nSplit_)
        ns << v << " ";
    RCLCPP_INFO_STREAM(this->get_logger(), ns.str());

    std::ostringstream cf;
    cf << "  constr_flags: ";
    for (auto v : constr_flags_)
        cf << (v ? "true " : "false ");
    RCLCPP_INFO_STREAM(this->get_logger(), cf.str());

    res->success = true;
    res->message = "Bezier parameters updated successfully";
    publishLog(res->message);
}

void DistributedPlannerNode::getPlannerConfigService(const std::shared_ptr<farol2_motion_planning::srv::GetPlannerConfig::Request> req,
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
    res->radius = bounds_.radius;

    res->alpha = bounds_.alpha;
    res->beta = bounds_.beta;
    res->gamma = bounds_.gamma;

    // Bezier
    res->bezier_degree = bezier_degree_;
    res->guess_degree = guess_degree_;
    res->n_split = nSplit_;
    res->constr_flags = constr_flags_;
    res->number_sample_pts = number_sample_Pts_;
}

void DistributedPlannerNode::setObstaclesService(const std::shared_ptr<farol2_motion_planning::srv::SetObstacles::Request> req,
                                                 std::shared_ptr<farol2_motion_planning::srv::SetObstacles::Response> res)
{
    // --- Circular obstacles ---
    size_t n_circ = req->circ_obs.size() / 3;

    if (n_circ)
    {
        circ_obs_.resize(3, n_circ); // circ_obs  # [E1, N1, r1, E2, N2, r2, ...]

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            for (size_t i = 0; i < n_circ; ++i)
            {
                circ_obs_(0, i) = req->circ_obs[i * 3 + 1]; // Northing
                circ_obs_(1, i) = req->circ_obs[i * 3 + 0]; // Easting
                circ_obs_(2, i) = req->circ_obs[i * 3 + 2];
            }
        }

        obstacles_updated_ = true;
    }
    else
    {
        circ_obs_.resize(3, 0);
    }

    // --- Eliptical obstacles ---
    size_t n_ellip = req->elip_obs.size() / 5;

    if (n_ellip)
    {
        elip_obs_.resize(5, n_ellip); // elip_obs  # [E1, N1, a1, b1, phi1, E2, N2, a2, b2, phi2, ...]

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            for (size_t i = 0; i < n_ellip; ++i)
            {
                elip_obs_(0, i) = req->elip_obs[i * 5 + 1]; // Northing
                elip_obs_(1, i) = req->elip_obs[i * 5 + 0]; // Easting
                elip_obs_(2, i) = req->elip_obs[i * 5 + 2]; // a
                elip_obs_(3, i) = req->elip_obs[i * 5 + 3]; // b
                elip_obs_(4, i) = req->elip_obs[i * 5 + 4]; // phi
            }
        }

        obstacles_updated_ = true;
    }
    else
    {
        elip_obs_.resize(5, 0);
    }

    // --- Line obstacles --- (TODO... - just a placeholder)
    size_t n_line = req->line_obs.size() / 3;
    if (n_line)
    {
        line_obs_.resize(3, n_line);
        for (size_t i = 0; i < n_line; ++i)
        {
            line_obs_(1, i) = req->line_obs[i * 3 + 0]; // x1
            line_obs_(0, i) = req->line_obs[i * 3 + 1]; // y1
            line_obs_(2, i) = req->line_obs[i * 3 + 2]; // x2 or other line info
        }
    }
    else
    {
        line_obs_.resize(3, 0);
    }

    if (n_circ == 0 && n_line == 0 && n_ellip == 0)
    {
        obstacles_updated_ = false; // No obstacles provided
    }

    // --- Build response + logs ---
    std::ostringstream msg;
    msg << "Obstacles updated: ";

    if (n_circ > 0)
    {
        msg << n_circ << " circular obstacle(s)";
    }
    else
    {
        msg << "No circular obstacles";
    }

    msg << " | ";

    if (n_ellip > 0)
    {
        msg << n_ellip << " elliptical obstacle(s)";
    }
    else
    {
        msg << "No elliptical obstacles";
    }
    msg << " | ";

    if (n_line > 0)
    {
        msg << n_line << " line obstacle(s)";
    }
    else
    {
        msg << "No line obstacles";
    }

    res->success = true;
    res->message = msg.str();

    RCLCPP_INFO_STREAM(this->get_logger(), res->message);
    // publishLog(res->message);
}

void DistributedPlannerNode::runDistributedSchemeIteration()
{
    
    if(distributed_scheme_running_)
    {
        // check if vehicle has reached goal 
        if(!isClose(goal_state_, states_history_[current_iteration_], pos_tol_, theta_tol_, v_tol_))
        {
            
            vehicle_state current_state = states_history_[current_iteration_];
            vehicle_state current_goal;
            
            Eigen::Vector2d eigen_state(current_state.x,current_state.y);

            Eigen::Vector2d eigen_goal(goal_state_.x,goal_state_.y);
            std::vector<Eigen::Vector2d> eigen_neighbors;
            
            {
                std::lock_guard<std::mutex> lock(state_mutex_);

                const auto it = neighbors_history_.find(current_iteration_);

                if (it != neighbors_history_.end())
                {
                    for (const auto& [name, position] : it->second)
                    {
                        eigen_neighbors.push_back(position);
                    }
                }
            }

            VoronoiCell current_cell = DistributedUtils::getVoronoiNeighbors(eigen_state, eigen_neighbors, cell_margin_);

            // Planner initialization
            MultipleVehiclePlanner solver(bezier_degree_, 1, {1,1,1},{true, false, false, false});
            solver.setBoundsAndGains(0.01, 1.0,
                                -5, 5,
                                -5, 5,
                                -5, 5,
                                1.5, 5000.0,
                                1.5, 1.0, 0.0, 0.0);

            if(DistributedUtils::isInsideVoronoiCell(eigen_goal, eigen_state, current_cell))
            {
                current_goal = goal_state_;
                solver.setFinalVelHeadMode(true, true);
            }
            else if(use_goal_straight_line_ && !use_goal_closer_point_)
            {
                Eigen::Vector2d current_goal_intersec = DistributedUtils::getCurrentGoalVoronoiIntersection(eigen_state, eigen_goal, current_cell);

                current_goal.x = current_goal_intersec(0);
                current_goal.y = current_goal_intersec(1);
                // These two I suspect are not used but maybe as guess they could so ...
                current_goal.theta = std::atan2((eigen_goal(1)-current_goal_intersec(1)),(eigen_goal(0)-current_goal_intersec(0)));
                current_goal.v = 0.01;
                solver.setFinalVelHeadMode(false, false);
            }
            else if(!use_goal_straight_line_ && use_goal_closer_point_)
            {
                Eigen::Vector2d current_goal_closest = DistributedUtils::getClosestPointInVoronoiCell(eigen_state, eigen_goal, current_cell);
                current_goal.x = current_goal_closest(0);
                current_goal.y = current_goal_closest(1);
                // These two I suspect are not used but maybe as guess they could so ...
                current_goal.theta = std::atan2((eigen_goal(1)-current_goal_closest(1)),(eigen_goal(0)-current_goal_closest(0)));
                current_goal.v = 0.01;
                solver.setFinalVelHeadMode(false, false);
            }
            else if(use_goal_straight_line_ && use_goal_closer_point_)
            {
                Eigen::Vector2d current_goal_closest = DistributedUtils::getClosestPointInVoronoiCell(eigen_state, eigen_goal, current_cell);
                Eigen::Vector2d current_goal_intersec = DistributedUtils::getCurrentGoalVoronoiIntersection(eigen_state, eigen_goal, current_cell);

                current_goal.x = (current_goal_closest(0) + current_goal_intersec(0))/2;
                current_goal.y = (current_goal_closest(1) + current_goal_intersec(1))/2;
                // These two I suspect are not used but maybe as guess they could so ...
                current_goal.theta = std::atan2((eigen_goal(1)-current_goal.y),(eigen_goal(0)-current_goal.x));
                current_goal.v = 0.02;
                solver.setFinalVelHeadMode(false, false);
            }
            else
            {
                std::string mess = "Neither select closest goal nor intersection are true!!!!\n";
                RCLCPP_ERROR_STREAM(this->get_logger(), mess);
                return;
            }

            std::vector<vehicle_state> current_states{current_state};
            std::vector<vehicle_state> goal_states{current_goal};

            solver.setupSymbolic();
            solver.computeCostFunction();
            solver.setOptimizationProblem(current_states, goal_states);
            solver.createConstraintVector();
            solver.createDecisionVector();

            std::atomic<bool> cancel_flag{false};

            
            solver.solveOptimizationProblem(&cancel_flag);

            std::string status_str = solver.getOptimizationstatus();
            std::string s = status_str;
            std::transform(s.begin(), s.end(), s.begin(),
                            [](unsigned char c)
                            { return std::tolower(c); });
            bool success = s.find("succe") != std::string::npos;
            double tf = solver.getTf();
            Eigen::Tensor<double, 3> ControlPts = solver.getControlPoints();
            Eigen::Tensor<double, 3> opt_traj = solver.getOptimalTrajectories(2000);


            cell_history_.push_back(current_cell);
            current_iteration_++;
        }
        else
        {
            distributed_scheme_running_ = false;
        }
    }
}

void DistributedPlannerNode::publishMissions()
{
    if (last_control_points_.size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "No optimized trajectory available!");
        return;
    }

    
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = formatMissionString(last_control_points_, last_Tf_);
    // Note: Publishing via control_points_pubs_ would be better

    mission_pub_->publish(*msg);

}

std::string DistributedPlannerNode::formatMissionString(const Eigen::Tensor<double, 3> &controlPoints, double Tf_opt)
{
    std::ostringstream oss;
    std::string local_reference = selected_vehicle_names_.front();
    vehicle_state ref_state = vehicle_states_[local_reference];
    oss << std::fixed << std::setprecision(3);

    oss << "3\n"
        << std::fixed << std::setprecision(3) << ref_state.x << " " << ref_state.y << " 29S\nBEZIER " << controlPoints.dimension(2) << " ";

    for (int j = 0; j < controlPoints.dimension(2); ++j)
    {
        oss << controlPoints(0, 0, j) << " "; // X values
    }
    for (int j = 0; j < controlPoints.dimension(2); ++j)
    {
        oss << controlPoints(0, 1, j) << " "; // Y values
    }

    oss << Tf_opt;

    return oss.str();
}

void DistributedPlannerNode::publishControlPoints(const Eigen::Tensor<double, 3> &tensor, const std::vector<double> &tf_values, bool is_guess)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    int n_vehicles = tensor.dimension(0);
    int dim1 = tensor.dimension(1);
    int dim2 = tensor.dimension(2);


    auto msg = std::make_shared<farol2_motion_planning::msg::StampedMatrix>();

    msg->header.stamp = this->now();
    msg->header.frame_id = "map";

    msg->n_vehicles = 1;
    msg->dim1 = dim1;
    msg->dim2 = dim2;

    msg->data.reserve(dim1 * dim2);

    for (int d = 0; d < dim1; ++d)
    {
        for (int n = 0; n < dim2; ++n)
        {
            msg->data.push_back(tensor(0, d, n));
        }
    }

    msg->tf_values = {tf_values[0]};
    msg->is_guess = is_guess;

    control_points_pub_->publish(*msg);
}

void printControlPoints(const Eigen::Tensor<double, 3> &cp)
{
    const int num_vehicles = cp.dimension(0);
    const int dim2 = cp.dimension(1);
    const int num_points = cp.dimension(2);

    for (int v = 0; v < num_vehicles; ++v)
    {
        std::cout << "Vehicle " << v << " control points:\n";

        for (int d = 0; d < dim2; ++d)
        {
            if (d == 0)
                std::cout << "  X: ";
            else if (d == 1)
                std::cout << "  Y: ";
            else
                std::cout << "  dim" << d << ": ";

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
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistributedPlannerNode>();

    RCLCPP_INFO(node->get_logger(), "Distributed Motion Planning Node started. Waiting for service calls...");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}