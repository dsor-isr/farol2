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

    prepareSolver();

    // Create timer for alive status (1 second interval)
    alive_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DistributedPlannerNode::publishAliveStatus, this));


    scheme_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&DistributedPlannerNode::runDistributedSchemeIteration, this));

    RCLCPP_INFO(this->get_logger(), "Distributed Motion Planning Node initialized successfully.");
}

DistributedPlannerNode::~DistributedPlannerNode()
{
    if (opt_thread_.joinable())
    {
        opt_thread_.join();
    }
}

// ------------------------------------------------------------------------------------------------------------------//
// -------------------------------------------------Initialization---------------------------------------------------//
// ------------------------------------------------------------------------------------------------------------------//
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

    // Load cost function weights
    bounds_.alpha = this->declare_parameter<double>("alpha", 1.0);
    bounds_.beta = this->declare_parameter<double>("beta", 5.0);
    bounds_.gamma = this->declare_parameter<double>("gamma", 1.0);

    bounds_.is_set = true;

    use_goal_straight_line_ = this->declare_parameter<bool>("use_goal_straight_line", false);
    use_goal_closer_point_ = this->declare_parameter<bool>("use_goal_closer_point", true);
    cell_margin_ = this->declare_parameter<double>("cell_margin", 2.0);
    sampling_period_ = this->declare_parameter<double>("sampling_period", 2.0);

    discovery_mode_ = this->declare_parameter<bool>("discovery_mode", true);

    vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "myellow0");

    // Synchronizing stuff
    const auto third_period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(sampling_period_ / 3.0));

    if (sampling_period_ <= 0.0 || third_period <= std::chrono::nanoseconds::zero())
    {
        throw std::runtime_error("sampling_period must be positive and long enough to schedule timers");
    }

    broadcast_timer_ = this->create_wall_timer(third_period,std::bind(&DistributedPlannerNode::publishNeighborPosition, this));

    // Poll frequently enough to notice the deadline without blocking the executor.
    solve_watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(5),std::bind(&DistributedPlannerNode::checkDistributedSolveDeadline, this));

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
    traj_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/" + vehicle_name_ + "/planned_path", rclcpp::QoS(1).transient_local());
    mission_pub_ = this->create_publisher<std_msgs::msg::String>("/" + vehicle_name_ + "/planning_console/Mission_String", rclcpp::QoS(1).transient_local());
    control_points_pub_= this->create_publisher<farol2_motion_planning::msg::StampedMatrix>("/" + vehicle_name_ + "/Stamped_controlPoints", rclcpp::QoS(1).transient_local());

}

void DistributedPlannerNode::initialiseServices()
{
    // This function can be used to initialize any subscribers if needed
    set_goal_srv_ = this->create_service<farol2_motion_planning::srv::SetState>(
        "/" + vehicle_name_ +"/"+ TOPIC_SRV_SET_GOAL,
        std::bind(&DistributedPlannerNode::setGoalService, this, std::placeholders::_1, std::placeholders::_2));

    set_states_srv_ = this->create_service<farol2_motion_planning::srv::SetState>(
        "/" + vehicle_name_ +"/"+ TOPIC_SRV_SET_STATES,
        std::bind(&DistributedPlannerNode::setStatesService, this, std::placeholders::_1, std::placeholders::_2));

    set_bounds_srv_ = this->create_service<farol2_motion_planning::srv::SetBoundsAndGains>(
        "/" + vehicle_name_ +"/"+ TOPIC_SRV_SET_BOUNDS,
        std::bind(&DistributedPlannerNode::setBoundsService, this, std::placeholders::_1, std::placeholders::_2));

    set_bezier_params_srv_ = this->create_service<farol2_motion_planning::srv::SetBezierParams>(
        "/" + vehicle_name_ +"/"+ TOPIC_SRV_SET_BEZIER_PARAMS,
        std::bind(&DistributedPlannerNode::setBezierParamsService, this, std::placeholders::_1, std::placeholders::_2));

    set_obstacles_srv_ = this->create_service<farol2_motion_planning::srv::SetObstacles>(
        "/" + vehicle_name_ +"/"+ TOPIC_SRV_SET_OBSTACLES,
        std::bind(&DistributedPlannerNode::setObstaclesService, this, std::placeholders::_1, std::placeholders::_2));

    get_planner_config_srv_ = this->create_service<farol2_motion_planning::srv::GetPlannerConfig>(
        "/" + vehicle_name_ +"/"+ TOPIC_SRV_GET_PLANNER_CONFIG,
        std::bind(&DistributedPlannerNode::getPlannerConfigService, this, std::placeholders::_1, std::placeholders::_2));
}

void DistributedPlannerNode::initialiseSubscribers()
{
    neighbor_sub_ = create_subscription<farol2_motion_planning::msg::NeighborPosition>(
        TOPIC_SUB_STATE,
        rclcpp::QoS(10),
        std::bind(&DistributedPlannerNode::neighborCallback, this, std::placeholders::_1)
    );
    distributed_start_stop_sub_ = create_subscription<farol2_motion_planning::msg::DistributedCommand>(
        TOPIC_SUB_START_STOP, 10,
        std::bind(&DistributedPlannerNode::distributedStartStopCallback, this, std::placeholders::_1));
}

// ------------------------------------------------------------------------------------------------------------------//
// -------------------------------------------------Sub Callbacks----------------------------------------------------//
// ------------------------------------------------------------------------------------------------------------------//
void DistributedPlannerNode::neighborCallback(const farol2_motion_planning::msg::NeighborPosition::ConstSharedPtr msg)
{
    if (msg->vehicle_name.empty() || msg->vehicle_name == vehicle_name_)
    {
        return;
    }

    if (msg->position.size() != 2)
    {
        RCLCPP_WARN(get_logger(), "Ignoring message from %s: position has %zu elements instead of 2", msg->vehicle_name.c_str(), msg->position.size());
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);

    // Discovery records vehicle identities only; it does not store positions.
    if (!distributed_scheme_running_ && get_parameter("discovery_mode").as_bool())
    {
        if (std::find(known_vehicle_names_.begin(), known_vehicle_names_.end(), msg->vehicle_name) == known_vehicle_names_.end())
        {
            known_vehicle_names_.push_back(msg->vehicle_name);
            RCLCPP_INFO(get_logger(), "Discovered vehicle: %s", msg->vehicle_name.c_str());
        }
        return;
    }
    
    // Ignore messages from vehicles not in the configured/discovered list.
    if (std::find(known_vehicle_names_.begin(), known_vehicle_names_.end(), msg->vehicle_name) == known_vehicle_names_.end())
    {
        return;
    }

    if (current_iteration_ < 0 || msg->curr_iter < static_cast<uint32_t>(current_iteration_))
    {
    return;  // stale
    }
    const int iter = static_cast<int>(msg->curr_iter);
    auto &iteration_neighbors = neighbors_history_[iter];
    const Eigen::Vector2d position(msg->position[0], msg->position[1]);

    if (!distributed_scheme_running_)
    {
        // Idle: retain the latest position as the starting value.
        iteration_neighbors.insert_or_assign(msg->vehicle_name, position);
        return;
    }

    // Running: keep only the first message received from each vehicle
    // for this iteration. Idle values already in the map count as present.
    if (iteration_neighbors.find(msg->vehicle_name) != iteration_neighbors.end())
    {
        return;
    }

    iteration_neighbors.emplace(msg->vehicle_name, position);
}

void DistributedPlannerNode::distributedStartStopCallback(const farol2_motion_planning::msg::DistributedCommand::SharedPtr msg)
{
  const auto & names = msg->vehicle_names;
  if (std::find(names.begin(), names.end(), vehicle_name_) == names.end())
    return;

  if (msg->start)
  {
    if (distributed_scheme_running_ || solve_running_)
    {
      RCLCPP_WARN(get_logger(), "Scheme is already running.");
      return;
    }

    if (!initial_state_received_ || !goal_state_received_)
    {
      RCLCPP_WARN(get_logger(), "Set this vehicle's initial state and goal before starting.");
      return;
    }

    states_history_.clear();
    control_points_history_.clear();
    tf_history_.clear();
    trajectories_history_.clear();
    cell_history_.clear();

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      neighbors_history_.clear();
      current_iteration_ = 0;
      states_history_.push_back(initial_state_);
    }

    first_run_ = true;
    iteration_collection_started_.reset();
    distributed_scheme_running_ = true;

    RCLCPP_INFO(get_logger(), "Distributed scheme started for %s", vehicle_name_.c_str());
  }
  else
  {
    if (!distributed_scheme_running_)
    {
      RCLCPP_WARN(get_logger(), "Scheme is not running.");
      return;
    }

    distributed_scheme_running_ = false;

    RCLCPP_INFO(get_logger(), "Distributed scheme stopped for %s", vehicle_name_.c_str());
  }
}

// ------------------------------------------------------------------------------------------------------------------//
// -------------------------------------------------Pub logic functions----------------------------------------------//
// ------------------------------------------------------------------------------------------------------------------//
void DistributedPlannerNode::publishNeighborPosition()
{
    farol2_motion_planning::msg::NeighborPosition msg;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        vehicle_state state;

        if (distributed_scheme_running_)
        {
            if (current_iteration_ < 0 || static_cast<size_t>(current_iteration_) >= states_history_.size())
            {
                return;
            }

            state = states_history_[current_iteration_];
            msg.curr_iter = static_cast<uint32_t>(current_iteration_);
        }
        else
        {
            // Idle/discovery messages use iteration 0.
            if (!get_parameter("discovery_mode").as_bool() && known_vehicle_names_.empty())
            {
                return;
            }

            state = initial_state_;
            msg.curr_iter = 0;
        }

        msg.vehicle_name = vehicle_name_;
        msg.position = {state.x, state.y};
    }

    msg.header.stamp = now();
    state_pub_->publish(msg);
}

void DistributedPlannerNode::publishMissions()
{
    // if (last_control_points_.size() == 0)
    // {
    //     RCLCPP_WARN(this->get_logger(), "No optimized trajectory available!");
    //     return;
    // }

    
    // auto msg = std::make_shared<std_msgs::msg::String>();
    // msg->data = formatMissionString(last_control_points_, last_Tf_);
    // // Note: Publishing via control_points_pubs_ would be better

    // mission_pub_->publish(*msg);

}

void DistributedPlannerNode::publishLog(const std::string &text)
{
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = text;
    mission_log_pub_->publish(*msg);
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

void DistributedPlannerNode::publishAliveStatus()
{
    auto msg = std::make_shared<std_msgs::msg::Bool>();
    msg->data = true;
    planning_alive_pub_->publish(*msg);
}

void DistributedPlannerNode::publishTrajectorySamples(const Eigen::MatrixXd &trajectory, int first_sample, int last_sample)
{
    if (trajectory.rows() < 2 || first_sample < 0 || last_sample < first_sample || last_sample >= trajectory.cols())
    {
        RCLCPP_ERROR(get_logger(), "Invalid trajectory sample range to publish.");
        return;
    }

    geometry_msgs::msg::PoseArray positions;
    positions.header.stamp = now();
    positions.header.frame_id = "map";
    positions.poses.reserve(static_cast<std::size_t>(last_sample - first_sample + 1));

    for (int i = first_sample; i <= last_sample; ++i)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = trajectory(0, i);
        pose.position.y = trajectory(1, i);
        pose.orientation.w = 1.0;
        positions.poses.push_back(std::move(pose));
    }

    traj_pub_->publish(positions);
}

// ------------------------------------------------------------------------------------------------------------------//
// -------------------------------------------------Services---------------------------------------------------------//
// ------------------------------------------------------------------------------------------------------------------//
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
        goal_state_received_ = true;
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

    res->success = true;
    res->message = "Bounds and gains set.";
    publishLog(res->message);
    RCLCPP_INFO_STREAM(this->get_logger(), res->message);

    prepareSolver();
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

    prepareSolver();
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

// ------------------------------------------------------------------------------------------------------------------//
// -------------------------------------------------Syncronization and Distributed-----------------------------------//
// ------------------------------------------------------------------------------------------------------------------//
void DistributedPlannerNode::startDistributedSolve(std::chrono::nanoseconds budget)
{
    // Do not start another solve on solver_ while the previous worker may still use it.
    if (solve_running_ || (opt_thread_.joinable() && !solve_done_.load()))
    {
        RCLCPP_WARN(get_logger(), "Previous distributed solve is still running; skipping this solve.");
        return;
    }

    if (opt_thread_.joinable())
    {
        opt_thread_.join();
    }

    solve_cancel_flag_.store(false);
    solve_done_.store(false);
    solve_result_ready_ = false;
    solve_succeeded_ = false;
    solve_running_ = true;
    solve_deadline_ = std::chrono::steady_clock::now() + budget;

    opt_thread_ = std::thread([this]()
    {
        try
        {
            solver_.solveOptimizationProblem(&solve_cancel_flag_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Distributed solver threw an exception: %s", e.what());
        }

        solve_done_.store(true);
    });
}

void DistributedPlannerNode::checkDistributedSolveDeadline()
{
    if (!solve_running_)
    {
        return;
    }

    if (!solve_done_.load() &&
        std::chrono::steady_clock::now() >= solve_deadline_)
    {
        // The solver must periodically check this flag to stop promptly.
        solve_cancel_flag_.store(true);
    }

    if (solve_done_.load())
    {
        finishDistributedSolve();
    }
}

void DistributedPlannerNode::finishDistributedSolve()
{
    if (!solve_running_ || !solve_done_.load())
    {
        return;
    }

    if (opt_thread_.joinable())
    {
        opt_thread_.join();
    }

    solve_running_ = false;

    std::string status = solver_.getOptimizationstatus();
    std::transform(status.begin(), status.end(), status.begin(), [](unsigned char c) { return std::tolower(c); });

    // A cancelled solve must not be treated as successful just because it returned.
    solve_succeeded_ = !solve_cancel_flag_.load() && status.find("succe") != std::string::npos;

    
    solve_result_ready_ = true;
    commitDistributedIteration();

    if (solve_succeeded_)
    {
        RCLCPP_INFO(get_logger(), "Distributed solve completed successfully.");
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Distributed solve failed or exceeded its time budget.");
    }

}

void DistributedPlannerNode::commitDistributedIteration()
{
    if (!pending_iteration_)
    {
        RCLCPP_ERROR(get_logger(), "Cannot commit: no pending distributed iteration.");
        return;
    }

    const auto pending = *pending_iteration_;

    if (pending.index != current_iteration_)
    {
        RCLCPP_ERROR(get_logger(), "Cannot commit iteration %d: current iteration is %d.", pending.index, current_iteration_);
        pending_iteration_.reset();
        return;
    }

    Eigen::MatrixXd curve;
    double curve_duration = 0.0;
    vehicle_state next_state = pending.start_state;
    Eigen::MatrixXd trajectory;
    int publish_last_sample = -1;
    bool have_next_state = false;

    if (solve_succeeded_)
    {
        curve = BezierUtils::tensor_2_matrix(solver_.getControlPoints(), 0);
        curve_duration = solver_.getTf();

        const bool bad_curve_shape = curve.rows() < 2;
        const bool bad_duration = !std::isfinite(curve_duration) || curve_duration <= 0.0;
        const bool period_exceeds_duration = std::isfinite(curve_duration) &&
                                             sampling_period_ > curve_duration;

        if (bad_curve_shape || bad_duration || period_exceeds_duration)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Unusable solver trajectory: curve rows=%ld cols=%ld, "
                "Tf=%.9f (finite=%s, positive=%s), sampling_period=%.9f, "
                "period_exceeds_Tf=%s, curve_all_finite=%s",
                static_cast<long>(curve.rows()),
                static_cast<long>(curve.cols()),
                curve_duration,
                std::isfinite(curve_duration) ? "true" : "false",
                curve_duration > 0.0 ? "true" : "false",
                sampling_period_,
                period_exceeds_duration ? "true" : "false",
                curve.allFinite() ? "true" : "false");

            solve_succeeded_ = false;
        }
        else
        {
            const auto sampled = solver_.getOptimalTrajectories(number_sample_Pts_);

            if (number_sample_Pts_ < 2 || sampled.dimension(0) < 1 || sampled.dimension(1) < 2 || sampled.dimension(2) != number_sample_Pts_)
            {
                RCLCPP_WARN(get_logger(), "Solver returned an invalid sampled trajectory.");
            }
            else
            {
                trajectory.resize(2, number_sample_Pts_);
                for (int i = 0; i < number_sample_Pts_; ++i)
                {
                    trajectory(0, i) = sampled(0, 0, i);
                    trajectory(1, i) = sampled(0, 1, i);
                }

                // Uniform samples cover [0, curve_duration].
                publish_last_sample = static_cast<int>(std::round((sampling_period_ / curve_duration) * (trajectory.cols() - 1)));

                if (publish_last_sample <= 0 || publish_last_sample >= trajectory.cols())
                {
                    RCLCPP_WARN(get_logger(), "Cannot select the first sampling-period window.");
                }
            }
            const double t = sampling_period_ / curve_duration;
            const Eigen::VectorXd position = BezierUtils::deCasteljau(t, curve);

            auto res = BezierUtils::divide_bezier(curve, t);
            const Eigen::MatrixXd& P = res.second;
            const Eigen::MatrixXd& der = BezierUtils::bezier_derivative(P)/(curve_duration-sampling_period_);

            

            next_state.x = position(0);
            next_state.y = position(1);
           // Heading from the derivative
            next_state.theta = std::atan2(der(1, 0), der(0, 0));

            // Speed: magnitude of the derivative vector
            next_state.v = std::sqrt(
                std::pow(der(0, 0), 2) +
                std::pow(der(1, 0), 2)
            );
            have_next_state = true;
        }
    } 
    
    if (!solve_succeeded_)
    {
        // The first iteration has no earlier trajectory to reuse.
        if (pending.index == 0)
        {
            RCLCPP_ERROR(get_logger(),"First optimization failed; no previous trajectory exists for fallback. Stopping the scheme.");
            distributed_scheme_running_ = false;
            pending_iteration_.reset();
            return;
        }

        const auto previous_index = static_cast<std::size_t>(pending.index - 1);

        if (control_points_history_.size() <= previous_index || tf_history_.size() <= previous_index)
        {
            RCLCPP_ERROR(get_logger(), "Fallback unavailable: previous trajectory history is incomplete. Stopping the scheme");
            distributed_scheme_running_ = false;
            pending_iteration_.reset();
            return;
        }

        const double previous_duration = tf_history_[previous_index];

        if (!std::isfinite(previous_duration) || previous_duration <= 2*sampling_period_)
        {
            RCLCPP_ERROR(get_logger(), "Fallback trajectory has no remaining duration. Stopping the scheme.");
            distributed_scheme_running_ = false;
            pending_iteration_.reset();
            return;
        }

        const Eigen::MatrixXd &previous_trajectory = trajectories_history_[previous_index];
        const int sample_count = static_cast<int>(previous_trajectory.cols());

        // Convert times T and 2T to indices in the uniformly sampled trajectory.
        const int first_sample = static_cast<int>(std::round(
            (sampling_period_ / previous_duration) * (sample_count - 1)));
        const int second_sample = static_cast<int>(std::round(
            ((2.0 * sampling_period_) / previous_duration) * (sample_count - 1)));

        if (first_sample < 0 || second_sample <= first_sample || second_sample >= sample_count)
        {
            RCLCPP_ERROR(get_logger(), "Previous sampled trajectory has no [T, 2T] window.");
        }

        // Store the remaining trajectory starting at T.
        // Its first sampling-period window is the previous trajectory's [T, 2T].
        trajectory = previous_trajectory.rightCols(sample_count - first_sample);
        publish_last_sample = second_sample - first_sample;

        const double split = sampling_period_ / previous_duration;
        auto divided_curve = BezierUtils::divide_bezier(control_points_history_[previous_index], split);

        curve = std::move(divided_curve.second);
        curve_duration = previous_duration - sampling_period_;

        if (curve.rows() < 2 || curve_duration <= 0.0)
        {
            RCLCPP_ERROR(get_logger(), "Fallback produced an invalid remaining trajectory. Stopping the scheme.");
            distributed_scheme_running_ = false;
            pending_iteration_.reset();
            return;
        }

        const double t = sampling_period_ / curve_duration;
        const Eigen::VectorXd position = BezierUtils::deCasteljau(t, curve);

        auto res = BezierUtils::divide_bezier(curve, t);
        const Eigen::MatrixXd& P = res.second;
        const Eigen::MatrixXd& der = BezierUtils::bezier_derivative(P)/(curve_duration-sampling_period_);

        

        next_state.x = position(0);
        next_state.y = position(1);
        // Heading from the derivative
        next_state.theta = std::atan2(der(1, 0), der(0, 0));

        // Speed: magnitude of the derivative vector
        next_state.v = std::sqrt(
            std::pow(der(0, 0), 2) +
            std::pow(der(1, 0), 2)
        );
        have_next_state = true;
    }

    if (!have_next_state)
    {
        RCLCPP_ERROR(get_logger(), "Could not compute the next state. Stopping the scheme.");
        distributed_scheme_running_ = false;
        pending_iteration_.reset();
        return;
    }

    control_points_history_.push_back(std::move(curve));
    tf_history_.push_back(curve_duration);
    trajectories_history_.push_back(std::move(trajectory));
    states_history_.push_back(next_state);
    cell_history_.push_back(pending.cell);

    ++current_iteration_;
    pending_iteration_.reset();
    iteration_collection_started_.reset();
    publishTrajectorySamples(trajectories_history_.back(), 0, publish_last_sample);

}

void DistributedPlannerNode::runDistributedSchemeIteration()
{
    
    if (!distributed_scheme_running_ || pending_iteration_.has_value() || solve_running_)
    {
        return;
    }
    // check if vehicle has reached goal 
    if(!isClose(goal_state_, states_history_[current_iteration_], pos_tol_, theta_tol_, v_tol_))
    {
        const auto now_steady = std::chrono::steady_clock::now();

        if (!iteration_collection_started_)
        {
            iteration_collection_started_ = now_steady;
        }

        bool all_neighbors_received = true;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            const auto it = neighbors_history_.find(current_iteration_);

            
            for (const auto &name : known_vehicle_names_)
            {
                if (name == vehicle_name_)
                {
                    continue;
                }

                if (it == neighbors_history_.end() || it->second.find(name) == it->second.end())
                {
                    all_neighbors_received = false;
                    break;
                }
            }
        }

        const auto collection_budget = std::chrono::duration<double>(sampling_period_ / 3.0);

        const bool collection_timed_out = now_steady - *iteration_collection_started_ >= collection_budget;

        if (!all_neighbors_received && !collection_timed_out)
        {
            return; // Keep broadcasting/listening; retry on the next timer tick.
        }
        
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
            RCLCPP_INFO(get_logger(),"############################################ ######################################### \n ############################################ ######################################### \n ############################################ #########################################");
            RCLCPP_INFO(get_logger(), "[%s][iter %d] neighbors: %zu received / %zu known",
                    vehicle_name_.c_str(), current_iteration_,
                    eigen_neighbors.size(), known_vehicle_names_.size());
            if (it != neighbors_history_.end())
            {
            for (const auto& [name, position] : it->second)
                RCLCPP_INFO(get_logger(), "  %s: (%.3f, %.3f)", name.c_str(), position.x(), position.y());
            }

            
        }

        VoronoiCell current_cell = DistributedUtils::getVoronoiNeighbors(eigen_state, eigen_neighbors, cell_margin_);

        for (const Eigen::Vector3d& edge : current_cell.edges)
        {
            RCLCPP_INFO(get_logger(), "  edge: goal_value=%.3f current_value=%.3f",
                        edge.x() * eigen_goal.x() + edge.y() * eigen_goal.y() + edge.z(),
                        edge.x() * eigen_state.x() + edge.y() * eigen_state.y() + edge.z());
        }
            
                        
        if(DistributedUtils::isInsideVoronoiCell(eigen_goal, eigen_state, current_cell))
        {
            RCLCPP_INFO(
                get_logger(),
                "Goal chosen is actual goal(%.6f, %.6f)",
                goal_state_.x, goal_state_.y);
            current_goal = goal_state_;
            solver_.setFinalVelHeadMode(true, true);
            solver_.setFinalPosition(true);
        }
        else if(use_goal_straight_line_ && !use_goal_closer_point_)
        {
            Eigen::Vector2d current_goal_intersec = DistributedUtils::getCurrentGoalVoronoiIntersection(eigen_state, eigen_goal, current_cell);

            current_goal.x = current_goal_intersec(0);
            current_goal.y = current_goal_intersec(1);
            // These two I suspect are not used but maybe as guess they could so ...
            current_goal.theta = std::atan2((eigen_goal(1)-current_goal_intersec(1)),(eigen_goal(0)-current_goal_intersec(0)));
            current_goal.v = 0.01;
            solver_.setFinalVelHeadMode(false, false);
            solver_.setFinalPosition(false);
        }
        else if(!use_goal_straight_line_ && use_goal_closer_point_)
        {
            Eigen::Vector2d current_goal_closest = DistributedUtils::getClosestPointInVoronoiCell(eigen_state, eigen_goal, current_cell);
            current_goal.x = current_goal_closest(0);
            current_goal.y = current_goal_closest(1);
            // These two I suspect are not used but maybe as guess they could so ...
            current_goal.theta = std::atan2((eigen_goal(1)-current_goal_closest(1)),(eigen_goal(0)-current_goal_closest(0)));
            current_goal.v = 0.01;
            solver_.setFinalVelHeadMode(false, false);
            solver_.setFinalPosition(false);
        }
        else if (use_goal_straight_line_ && use_goal_closer_point_)
        {
            Eigen::Vector2d current_goal_closest =
                DistributedUtils::getClosestPointInVoronoiCell(eigen_state, eigen_goal, current_cell);
            Eigen::Vector2d current_goal_intersec =
                DistributedUtils::getCurrentGoalVoronoiIntersection(eigen_state, eigen_goal, current_cell);

            RCLCPP_INFO(
                get_logger(),
                "Voronoi goal points: closest=(%.6f, %.6f), intersection=(%.6f, %.6f)",
                current_goal_closest(0), current_goal_closest(1),
                current_goal_intersec(0), current_goal_intersec(1));

            current_goal.x = (current_goal_closest(0) + current_goal_intersec(0)) / 2.0;
            current_goal.y = (current_goal_closest(1) + current_goal_intersec(1)) / 2.0;
            current_goal.theta = std::atan2(
                eigen_goal(1) - current_goal.y,
                eigen_goal(0) - current_goal.x);
            current_goal.v = 0.02;
            solver_.setFinalVelHeadMode(false, false);
            solver_.setFinalPosition(false);
        }
        else
        {
            std::string mess = "Neither select closest goal nor intersection are true!!!!\n";
            RCLCPP_ERROR_STREAM(this->get_logger(), mess);
            return;
        }

        pending_iteration_ = PendingIteration{
            current_iteration_,
            current_state,
            current_cell
        };

        solver_.setC3Continuity(!first_run_);

        std::vector<vehicle_state> current_states{current_state};
        std::vector<vehicle_state> goal_states{current_goal};

        Eigen::Matrix<double, 3, Eigen::Dynamic> all_line_obs(3, line_obs_.cols() + current_cell.edges.size());
        Eigen::Matrix<double, 3, Eigen::Dynamic> empty_obs(3, 0);
        Eigen::Matrix<double, 5, Eigen::Dynamic> empty_elip_obs(5, 0);
        all_line_obs << line_obs_, cellToLineObs(current_cell);
        if(first_run_)
        {
            solver_.setOptimizationProblem(current_states, goal_states, empty_obs, empty_obs, empty_elip_obs);
        }
        else
        {
            const double tf_prev = tf_history_[current_iteration_-1];
            const double tf_guess = tf_prev - sampling_period_;

            auto res = BezierUtils::divide_bezier(control_points_history_[current_iteration_-1],
                                                sampling_period_ / tf_prev);
            const Eigen::MatrixXd& P = res.second;

            Eigen::Tensor<double, 3> prev_control_points = BezierUtils::matrix_2_tensor(P);

            solver_.setOptimizationProblem(current_states, goal_states, empty_obs, empty_obs, empty_elip_obs, prev_control_points, tf_guess, false);

            RCLCPP_INFO(get_logger(), "[%s][iter %d] state=(%.3f, %.3f, th=%.3f, v=%.3f) tf_prev=%.3f tf_guess=%.3f tf_min=%.3f",
                        vehicle_name_.c_str(), current_iteration_, current_state.x, current_state.y,
                        current_state.theta, current_state.v, tf_prev, tf_guess, tf_min_);
        }
        solver_.createConstraintVectorDistributed(sampling_period_);
        solver_.createDecisionVectorDistributed(tf_min_);

        const auto solve_budget = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(sampling_period_ / 3.0));

        startDistributedSolve(solve_budget);
        first_run_ = false;

        return; // Completion handler will commit this iteration.

    }
    else
    {
        distributed_scheme_running_ = false;
    }
}

// ------------------------------------------------------------------------------------------------------------------//
// -------------------------------------------------Helpers----------------------------------------------------------//
// ------------------------------------------------------------------------------------------------------------------//
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

void DistributedPlannerNode::prepareSolver()
{
    if (!bounds_.is_set) {
        throw std::runtime_error("Bounds and gains must be loaded before preparing the solver");
    }
    solver_.setOptimizationParameters(bezier_degree_, 1,nSplitInt(), constraintFlagsUint8());
    solver_.setBoundsAndGains(bounds_.vel_min, bounds_.vel_max,
                bounds_.acc_min, bounds_.acc_max,
                bounds_.ang_vel_min, bounds_.ang_vel_max,
                bounds_.ang_acc_min, bounds_.ang_acc_max,
                bounds_.obs_min, bounds_.obs_max,
                bounds_.radius,
                bounds_.alpha, bounds_.beta, bounds_.gamma);
    solver_.setupSymbolic();
    solver_.computeCostFunctionDistributed(sampling_period_);

    RCLCPP_INFO(this->get_logger(), "prepared solver for new optimization.");

    first_run_ = true;
    
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
        initial_state_received_ = true;

    }
}

// definition
Eigen::Matrix<double, 3, Eigen::Dynamic> DistributedPlannerNode::cellToLineObs(const VoronoiCell& cell)
{
    Eigen::Matrix<double, 3, Eigen::Dynamic> out(3, static_cast<Eigen::Index>(cell.edges.size()));
    for (Eigen::Index k = 0; k < out.cols(); ++k)
        out.col(k) = cell.edges[k];
    return out;
}

std::vector<uint8_t> DistributedPlannerNode::constraintFlagsUint8() const
{
    std::vector<uint8_t> flags;
    flags.reserve(constr_flags_.size());
    for (bool flag : constr_flags_)
        flags.push_back(flag ? 1 : 0);
    return flags;
}

std::vector<int> DistributedPlannerNode::nSplitInt() const
{
    return std::vector<int>(nSplit_.begin(), nSplit_.end());
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