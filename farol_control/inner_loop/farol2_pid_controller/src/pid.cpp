#include "pid.hpp"

/* Constructor */
PID::PID() : Node("pid", 
                  rclcpp::NodeOptions()
                    .allow_undeclared_parameters(false)
                    .automatically_declare_parameters_from_overrides(true)) {

  clock_ = this->get_clock();
  last_update_time_ = clock_->now();

  auto now = clock_->now();

  controller_last_reference_ = {
    {"surge", now},
    {"sway", now},
    {"heave", now},
    {"yaw", now},
    {"pitch", now},
    {"roll", now},
    {"yaw_rate", now},
    {"pitch_rate", now},
    {"roll_rate", now}
  };

  controller_has_reference_ = {
    {"surge", false},
    {"sway", false},
    {"heave", false},
    {"yaw", false},
    {"pitch", false},
    {"roll", false},
    {"yaw_rate", false},
    {"pitch_rate", false},
    {"roll_rate", false}
  };
  
  /* Initialise body wrench request forces and torques as 0 */
  resetBodyWrenchRequest();
  
  loadParams();

  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
  
  createControllers();

  // Build the dispatch table after controllers are instantiated.
  initializeControllerConfigs();

}

/* Destructor */
PID::~PID() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void PID::loadParams() {
  static std::map<std::string, rclcpp::Parameter> raw_controllers_configuration;

  configured_controllers_.clear();
  controller_names_.clear();
  controller_parameters_.clear();
  controller_debug_.clear();

  const bool has_controller_list = this->get_parameter("controllers", configured_controllers_) &&
                                   !configured_controllers_.empty();

  // Optional allow-list: if absent, all known channels are considered and
  // later filtered by each channel's `enabled` flag.
  if (has_controller_list) {
    for (const auto &name : configured_controllers_) {
      if (controller_map_.count(name) == 0) {
        RCLCPP_WARN(get_logger(), "Ignoring unknown controller '%s' listed in parameter 'controllers'.", name.c_str());
        continue;
      }
      controller_names_.insert(name);
    }
  } else {
    for (const auto &[name, _] : controller_map_) {
      controller_names_.insert(name);
    }
  }

  /* Get raw flatten list of configured controllers */
  /* Example of the map's keys: */
  /* - node_frequency */
  /* - pitch.enabled */
  /* - pitch.kd */
  /* - pitch.ki */
  /* - pitch.kp */
  /* - topics.publishers.body_wrench_request */
  /* - topics.subscribers.nav_state */
  if (!get_node_parameters_interface()->get_parameters_by_prefix(
        "", raw_controllers_configuration)) {
    /* If no controller configurations were found */
    RCLCPP_ERROR(get_logger(), "No controllers found in control config file.");
    rclcpp::shutdown();
  }

  /* Populate data structure with parameters for each controller */
  for (const auto & [key, param] : raw_controllers_configuration) {
    /* Get string from key up to first ".", which is potentially a controller name */
    static std::string name;
    name = key.substr(0, key.find('.'));

    /* If key corresponds to a parameter from a controller */
    if (controller_names_.find(name) != controller_names_.end()) {
      /* If no parameter has been added yet for controller in name string */
      if (controller_parameters_.find(name) == controller_parameters_.end()) {
        /* Create entry for controller in key string */
        controller_parameters_.insert({name, {}});
      }
      
      /* Get name of the parameter */
      static std::string param_name;
      param_name = key.substr(name.length() + 1, -1);

      /* Add parameter for this controller. */
      /* Here it is assumed that the parameters for each controller are always */
      /* double (except for the "enabled" parameter) otherwise the parameter is skipped */
      try {
        if (param_name == "enabled") {
          controller_parameters_[name].insert({param_name, param.as_bool() ? 1.0 : 0.0});
          RCLCPP_DEBUG(get_logger(), "BOOL %s: %d", key.c_str(), param.as_bool());
        } else if (param_name == "debug") {
          controller_debug_[name] = param.as_bool();
          RCLCPP_DEBUG(get_logger(), "BOOL %s: %d", key.c_str(), param.as_bool());
        } else if (param_name == "use_ref_lpf") {
          controller_parameters_[name].insert({param_name, param.as_bool() ? 1.0 : 0.0});
          RCLCPP_DEBUG(get_logger(), "BOOL %s: %d", key.c_str(), param.as_bool());
        } else if (param_name == "use_lpf") {
          // Backward compatibility for legacy configs.
          controller_parameters_[name].insert({"use_ref_lpf", param.as_bool() ? 1.0 : 0.0});
          RCLCPP_DEBUG(get_logger(), "BOOL %s: %d", key.c_str(), param.as_bool());
        } else if (param_name == "delta_implementation") {
          controller_parameters_[name].insert({param_name, param.as_bool() ? 1.0 : 0.0});
          RCLCPP_DEBUG(get_logger(), "BOOL %s: %d", key.c_str(), param.as_bool());
        } else if (param_name == "use_state_lpf") {
          controller_parameters_[name].insert({param_name, param.as_bool() ? 1.0 : 0.0});
          RCLCPP_DEBUG(get_logger(), "BOOL %s: %d", key.c_str(), param.as_bool());
        } else if (param_name == "use_state_lpf_for_state_rate") {
          controller_parameters_[name].insert({param_name, param.as_bool() ? 1.0 : 0.0});
          RCLCPP_DEBUG(get_logger(), "BOOL %s: %d", key.c_str(), param.as_bool());
        } else if (param_name == "use_filtered_state_for_control") {
          controller_parameters_[name].insert({param_name, param.as_bool() ? 1.0 : 0.0});
          RCLCPP_DEBUG(get_logger(), "BOOL %s: %d", key.c_str(), param.as_bool());
        } else if (param_name == "use_filtered_ref_for_control") {
          controller_parameters_[name].insert({param_name, param.as_bool() ? 1.0 : 0.0});
          RCLCPP_DEBUG(get_logger(), "BOOL %s: %d", key.c_str(), param.as_bool());
        } else if (param_name == "use_rate_limiter") {
          controller_parameters_[name].insert({param_name, param.as_bool() ? 1.0 : 0.0});
          RCLCPP_DEBUG(get_logger(), "BOOL %s: %d", key.c_str(), param.as_bool());
        } else {
          controller_parameters_[name].insert({param_name, param.as_double()});
          RCLCPP_DEBUG(get_logger(), "DOUBLE %s: %f", key.c_str(), param.as_double());
        }

      } catch(...) {
        RCLCPP_INFO(get_logger(), "Unexpected parameter (not boolean or double) found for controller %s: %s", name.c_str(), param_name.c_str());
        continue;
      }
    }
  }

  std::set<std::string> enabled_controllers;
  for (const auto &name : controller_names_) {
    const auto controller_it = controller_parameters_.find(name);
    if (controller_it == controller_parameters_.end()) {
      continue;
    }

    const auto enabled_it = controller_it->second.find("enabled");
    if (enabled_it != controller_it->second.end() && enabled_it->second != 0.0) {
      enabled_controllers.insert(name);
    }
  }
  // Keep only channels explicitly enabled in configuration.
  controller_names_ = enabled_controllers;

  if (controller_names_.empty()) {
    RCLCPP_WARN(get_logger(), "No enabled controllers found. Node will run without applying control effort.");
  }

  course_control_ = this->get_parameter("course_control").as_bool();
  if (!this->get_parameter("use_heading_rate_as_yaw_rate", use_heading_rate_as_yaw_rate_)) {
    use_heading_rate_as_yaw_rate_ = false;
  }
  lpf_order_ = this->get_parameter("lpf_order").as_int();
  lpf_method_ = this->get_parameter("lpf_method").as_string();
  lpf_design_ = this->get_parameter("lpf_design").as_string();
}

/**
 * @brief Initialise Subscribers
 */
void PID::initialiseSubscribers() {
  nav_state_sub_ = create_subscription<farol2_interfaces::msg::NavigationState>(
                    get_parameter("topics.subscribers.nav_state").as_string(), 
                    1, std::bind(&PID::navStateCallback, this, std::placeholders::_1));

  for (const auto &name : controller_names_) {
    if (controller_map_.count(name) == 0) {
      continue;
    }

    reference_subscribers_[name] = create_subscription<std_msgs::msg::Float32>(
      get_parameter("topics.subscribers." + name + "_ref").as_string(),
      1,
      [this, name](const std_msgs::msg::Float32 &msg) {
        this->referenceCallback(name, msg.data);
      });
  }
}

/**
 * @brief Initialise Publishers
 */
void PID::initialisePublishers() {
  thrust_x_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("topics.publishers.thrust_x").as_string(), 1);

  thrust_y_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("topics.publishers.thrust_y").as_string(), 1);

  thrust_z_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("topics.publishers.thrust_z").as_string(), 1);

  torque_x_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("topics.publishers.torque_x").as_string(), 1);

  torque_y_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("topics.publishers.torque_y").as_string(), 1);

  torque_z_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("topics.publishers.torque_z").as_string(), 1);
  /*
  body_wrench_request_pub_ = create_publisher<control_allocation::msg::BodyWrenchRequest>(
                    get_parameter("topics.publishers.body_wrench_request").as_string(), 1);
  
  */
  for (const auto& [name, dbg] : controller_debug_) {
    if (!dbg) continue;
    const auto topic =
        get_parameter("topics.publishers.debug." + name).as_string();
    debug_publishers_[name] = create_publisher<farol2_pid_controller::msg::PidDebug>(topic, 1);
  }          
}

/**
 * @brief Initialise Services
 */
void PID::initialiseServices() {
  /* Service servers */
  /* Service to change controllers' parameters */
  change_params_srv_ = create_service<farol2_pid_controller::srv::ChangeParams>(
                        get_parameter("topics.services.change_params").as_string(),
                        std::bind(&PID::changeParamsCallback, this, std::placeholders::_1, std::placeholders::_2));

  /* Service to set course control flag */
  course_control_srv_ = create_service<std_srvs::srv::SetBool>(
                        get_parameter("topics.services.course_control").as_string(),
                        std::bind(&PID::courseControlCallback, this, std::placeholders::_1, std::placeholders::_2));

  /* service clients */
  /*... */

  return;
}

/**
 * @brief Initialise Timers
 */
void PID::initialiseTimers() {


  /* Get node frequency from parameters */
  node_frequency_ = get_parameter("node_frequency").as_double();

  /* Create timer */
  timer_ = create_timer
    (std::chrono::milliseconds(int(1.0/node_frequency_*1000)), 
    std::bind(&PID::timerCallback, this));
}

void PID::navStateCallback(const farol2_interfaces::msg::NavigationState &msg) {
  nav_state_ = msg;
  has_nav_state_ = true;
}

void PID::referenceCallback(const std::string &controller_name, double raw_value) {
  double ref_value = raw_value;

  // Linear channels use incoming values directly; angular channels keep
  // internal references in radians.
  if (controller_name == "surge") surge_ref_ = ref_value;
  else if (controller_name == "sway") sway_ref_ = ref_value;
  else if (controller_name == "heave") heave_ref_ = ref_value;
  else if (controller_name == "yaw") {
    ref_value = farol2_utils::deg2rad(raw_value);
    yaw_ref_ = ref_value;
  } else if (controller_name == "pitch") {
    ref_value = farol2_utils::deg2rad(raw_value);
    pitch_ref_ = ref_value;
  } else if (controller_name == "roll") {
    ref_value = farol2_utils::deg2rad(raw_value);
    roll_ref_ = ref_value;
  } else if (controller_name == "yaw_rate") {
    ref_value = farol2_utils::deg2rad(raw_value);
    yaw_rate_ref_ = ref_value;
  } else if (controller_name == "pitch_rate") {
    ref_value = farol2_utils::deg2rad(raw_value);
    pitch_rate_ref_ = ref_value;
  } else if (controller_name == "roll_rate") {
    ref_value = farol2_utils::deg2rad(raw_value);
    roll_rate_ref_ = ref_value;
  }

  const auto now = clock_->now();
  double dt_ref = 0.0;
  if (controller_has_reference_[controller_name]) {
    dt_ref = (now - controller_last_reference_[controller_name]).seconds();
    if (dt_ref > 20.0 / node_frequency_) {
      dt_ref = 0.0;
    }
  }

  auto rg_it = reference_generators_.find(controller_name);
  if (rg_it != reference_generators_.end() && rg_it->second) {
    reference_outputs_[controller_name] = rg_it->second->update(ref_value, dt_ref);
  }

  controller_has_reference_[controller_name] = true;
  controller_last_reference_[controller_name] = now;
}

bool PID::validateControllerParams(const std::string &controller_name,
                                   const std::vector<std::string> &required_params) {
  auto controller_it = controller_parameters_.find(controller_name);
  if (controller_it == controller_parameters_.end()) {
    return false;
  }

  for (const auto &param : required_params) {
    if (controller_it->second.count(param) == 0) {
      return false;
    }
  }

  return true;
}

void PID::createControllers() {
  const std::vector<std::string> pi_required = {"kp", "ki", "lpf_wc", "tau_min", "tau_max"};
  const std::vector<std::string> pid_required = {"kp", "ki", "kd", "lpf_wc", "tau_min", "tau_max"};
  const std::vector<std::string> yaw_required = {"kp", "ki", "kd", "kffa", "kffv_lin", "kffv_sq", "lpf_wc", "tau_min", "tau_max"};

  const auto create_pi = [this](const std::string &name, std::unique_ptr<ControllerPI> &controller) {
    const bool use_ref_lpf = controller_parameters_[name].count("use_ref_lpf") ? controller_parameters_[name]["use_ref_lpf"] != 0.0 : true;
    controller = std::make_unique<ControllerPI>();
    controller->configure(controller_parameters_[name]["kp"],
                          controller_parameters_[name]["ki"],
                          controller_parameters_[name]["tau_min"],
                          controller_parameters_[name]["tau_max"],
                          use_ref_lpf,
                          controller_parameters_[name]["lpf_wc"]);
  };

  const auto create_pid = [this](const std::string &name,
                                 std::unique_ptr<ControllerPID> &controller,
                                 double kffv_lin,
                                 double kffv_sq,
                                 double kffa) {
    const bool use_ref_lpf = controller_parameters_[name].count("use_ref_lpf") ? controller_parameters_[name]["use_ref_lpf"] != 0.0 : true;
    const bool delta_implementation =
      controller_parameters_[name].count("delta_implementation") ? controller_parameters_[name]["delta_implementation"] != 0.0 : true;
    const bool use_filtered_ref_for_control =
      controller_parameters_[name].count("use_filtered_ref_for_control") ? controller_parameters_[name]["use_filtered_ref_for_control"] != 0.0 : false;
    const bool use_rate_limiter =
      controller_parameters_[name].count("use_rate_limiter") ? controller_parameters_[name]["use_rate_limiter"] != 0.0 : false;
    const double rate_limit =
      controller_parameters_[name].count("rate_limit") ? controller_parameters_[name]["rate_limit"] : 0.0;

    reference_generators_[name] = std::make_unique<farol_control::ReferenceGenerator>();
    reference_generators_[name]->configure(true,
                                           use_rate_limiter,
                                           rate_limit,
                                           use_ref_lpf,
                                           use_filtered_ref_for_control,
                                           controller_parameters_[name]["lpf_wc"],
                                           lpf_order_,
                                           lpf_design_,
                                           lpf_method_);
    reference_outputs_[name] = reference_generators_[name]->output();

    controller = std::make_unique<ControllerPID>();
    controller->configure(controller_parameters_[name]["kp"],
                          controller_parameters_[name]["ki"],
                          controller_parameters_[name]["kd"],
                          kffv_lin,
                          kffv_sq,
                          kffa,
                          controller_parameters_[name]["tau_min"],
                          controller_parameters_[name]["tau_max"],
                          delta_implementation,
                          true);
  };

  if (controller_names_.count("surge") && !validateControllerParams("surge", pi_required)) {
    RCLCPP_ERROR(get_logger(), "Surge Controller missing parameters (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }
  if (controller_names_.count("surge")) create_pi("surge", controller_surge_);

  if (controller_names_.count("sway") && !validateControllerParams("sway", pi_required)) {
    RCLCPP_ERROR(get_logger(), "Sway Controller missing parameters (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }
  if (controller_names_.count("sway")) create_pi("sway", controller_sway_);

  if (controller_names_.count("heave") && !validateControllerParams("heave", pi_required)) {
    RCLCPP_ERROR(get_logger(), "Surge Controller parameters gains (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }
  if (controller_names_.count("heave")) create_pi("heave", controller_heave_);

  if (controller_names_.count("yaw") && !validateControllerParams("yaw", yaw_required)) {
    RCLCPP_ERROR(get_logger(), "Yaw Controller missing parameters (kp, ki, kd, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }
  if (controller_names_.count("yaw")) {
    create_pid("yaw",
               controller_yaw_,
               controller_parameters_["yaw"]["kffv_lin"],
               controller_parameters_["yaw"]["kffv_sq"],
               controller_parameters_["yaw"]["kffa"]);
  }

  if (controller_names_.count("pitch") && !validateControllerParams("pitch", pid_required)) {
    RCLCPP_ERROR(get_logger(), "Pitch Controller missing parameters (kp, ki, kd, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }
  if (controller_names_.count("pitch")) create_pid("pitch", controller_pitch_, 0.0, 0.0, 0.0);

  if (controller_names_.count("roll") && !validateControllerParams("roll", pid_required)) {
    RCLCPP_ERROR(get_logger(), "Roll Controller missing parameters (kp, ki, kd, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }
  if (controller_names_.count("roll")) create_pid("roll", controller_roll_, 0.0, 0.0, 0.0);

  if (controller_names_.count("yaw_rate") && !validateControllerParams("yaw_rate", pi_required)) {
    RCLCPP_ERROR(get_logger(), "Yaw Rate Controller missing parameters (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }
  if (controller_names_.count("yaw_rate")) create_pi("yaw_rate", controller_yaw_rate_);

  if (controller_names_.count("pitch_rate") && !validateControllerParams("pitch_rate", pi_required)) {
    RCLCPP_ERROR(get_logger(), "Pitch Rate Controller missing parameters (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }
  if (controller_names_.count("pitch_rate")) create_pi("pitch_rate", controller_pitch_rate_);

  if (controller_names_.count("roll_rate") && !validateControllerParams("roll_rate", pi_required)) {
    RCLCPP_ERROR(get_logger(), "Roll Rate Controller missing parameters (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }
  if (controller_names_.count("roll_rate")) create_pi("roll_rate", controller_roll_rate_);
}

void PID::initializeControllerConfigs() {
  controller_configs_.clear();

  const auto pi_required = std::vector<std::string>{"kp", "ki", "lpf_wc", "tau_min", "tau_max"};
  const auto pid_required = std::vector<std::string>{"kp", "ki", "kd", "lpf_wc", "tau_min", "tau_max"};
  const auto yaw_required = std::vector<std::string>{"kp", "ki", "kd", "kffa", "kffv_lin", "kffv_sq", "lpf_wc", "tau_min", "tau_max"};

  const auto make_pi_config = [this](const std::string &name,
                                     ControllerType type,
                                     const std::vector<std::string> &required_params,
                                     const std::function<double()> &get_state,
                                     const std::function<double()> &get_ref,
                                     const std::function<void(double)> &accumulate_output,
                                     const std::function<void(farol2_pid_controller::msg::PidDebug &)> &fill_debug) {
    return ControllerConfig{name, type, false, required_params, get_state, get_ref, []() { return 0.0; }, accumulate_output, fill_debug};
  };

  const auto make_pid_config = [this](const std::string &name,
                                      ControllerType type,
                                      const std::vector<std::string> &required_params,
                                      const std::function<double()> &get_state,
                                      const std::function<double()> &get_ref,
                                      const std::function<double()> &get_rate,
                                      const std::function<void(double)> &accumulate_output,
                                      const std::function<void(farol2_pid_controller::msg::PidDebug &)> &fill_debug) {
    return ControllerConfig{name, type, true, required_params, get_state, get_ref, get_rate, accumulate_output, fill_debug};
  };

  for (const auto &name : controller_names_) {
    if (controller_map_.count(name) == 0) {
      continue;
    }

    // Each case binds state/ref/rate accessors and output axis mapping for the
    // generic controller execution path.
    switch (controller_map_[name]) {
      case SURGE:
        controller_configs_[name] = make_pi_config(
          name,
          SURGE,
          pi_required,
          [this]() { return nav_state_.body_velocity_fluid.x; },
          [this]() { return surge_ref_; },
          [this](double tau) { body_wrench_request_msg_.wrench.force.x += tau; },
          [this](farol2_pid_controller::msg::PidDebug &debug_msg) {
            debug_msg.error = controller_surge_->getError();
            debug_msg.p_term = controller_surge_->getProportionalTerm();
            debug_msg.i_term = controller_surge_->getIntegralTerm();
            debug_msg.tau_d = controller_surge_->getTau_d();
            debug_msg.tau_dot = controller_surge_->getTauDot();
            debug_msg.tau_sat = controller_surge_->getTau_sat();
            debug_msg.a_term = controller_surge_->getAntiWindupTerm();
            debug_msg.tau = tau_;
          });
        break;

      case SWAY:
        controller_configs_[name] = make_pi_config(
          name,
          SWAY,
          pi_required,
          [this]() { return nav_state_.body_velocity_fluid.y; },
          [this]() { return sway_ref_; },
          [this](double tau) { body_wrench_request_msg_.wrench.force.y += tau; },
          [this](farol2_pid_controller::msg::PidDebug &debug_msg) {
            debug_msg.error = controller_sway_->getError();
            debug_msg.p_term = controller_sway_->getProportionalTerm();
            debug_msg.i_term = controller_sway_->getIntegralTerm();
            debug_msg.tau_d = controller_sway_->getTau_d();
            debug_msg.tau_dot = controller_sway_->getTauDot();
            debug_msg.tau_sat = controller_sway_->getTau_sat();
            debug_msg.a_term = controller_sway_->getAntiWindupTerm();
            debug_msg.tau = tau_;
          });
        break;

      case HEAVE:
        controller_configs_[name] = make_pi_config(
          name,
          HEAVE,
          pi_required,
          [this]() { return nav_state_.body_velocity_fluid.z; },
          [this]() { return heave_ref_; },
          [this](double tau) { body_wrench_request_msg_.wrench.force.z += tau; },
          [this](farol2_pid_controller::msg::PidDebug &debug_msg) {
            debug_msg.error = controller_heave_->getError();
            debug_msg.p_term = controller_heave_->getProportionalTerm();
            debug_msg.i_term = controller_heave_->getIntegralTerm();
            debug_msg.tau_d = controller_heave_->getTau_d();
            debug_msg.tau_dot = controller_heave_->getTauDot();
            debug_msg.tau_sat = controller_heave_->getTau_sat();
            debug_msg.a_term = controller_heave_->getAntiWindupTerm();
            debug_msg.tau = tau_;
          });
        break;

      case YAW:
        controller_configs_[name] = make_pid_config(
          name,
          YAW,
          yaw_required,
          [this]() -> double {
            if (course_control_) {
              return farol2_utils::deg2rad(static_cast<double>(nav_state_.course_angle));
            }
            return farol2_utils::deg2rad(static_cast<double>(nav_state_.orientation.z));
          },
          [this]() { return yaw_ref_; },
          [this]() -> double {
            if (use_heading_rate_as_yaw_rate_) {
              return farol2_utils::deg2rad(static_cast<double>(nav_state_.heading_rate));
            }
            return farol2_utils::deg2rad(static_cast<double>(nav_state_.orientation_rate.z));
          },
          [this](double tau) { body_wrench_request_msg_.wrench.torque.z += tau; },
          [this](farol2_pid_controller::msg::PidDebug &debug_msg) {
            const auto &ref = reference_outputs_["yaw"];
            debug_msg.error = controller_yaw_->getError();
            debug_msg.error_rate = controller_yaw_->error_dot_;
            debug_msg.error_rate_dot = controller_yaw_->error_rate_dot_;
            debug_msg.p_term = controller_yaw_->getProportionalTerm();
            debug_msg.i_term = controller_yaw_->getIntegralTerm();
            debug_msg.d_term = controller_yaw_->getDerivativeTerm();
            debug_msg.ff_term = controller_yaw_->getFFTerm();
            debug_msg.tau_d = controller_yaw_->getTau_d();
            debug_msg.tau_dot = controller_yaw_->getTauDot();
            debug_msg.tau_sat = controller_yaw_->getTau_sat();
            debug_msg.a_term = controller_yaw_->getAntiWindupTerm();
            debug_msg.tau = controller_yaw_->getOutput();
            debug_msg.state = controller_yaw_->state_;
            debug_msg.state_rate_used = controller_yaw_->state_rate_used_;
            debug_msg.ref_raw = farol2_utils::wrapTo2Pi(ref.ref_raw);
            debug_msg.ref_filt = farol2_utils::wrapTo2Pi(ref.ref_filt);
            debug_msg.dref_filt = ref.dref;
            debug_msg.ddref_filt = ref.ddref;
          });
        break;

      case PITCH:
        controller_configs_[name] = make_pid_config(
          name,
          PITCH,
          pid_required,
          [this]() { return farol2_utils::deg2rad(nav_state_.orientation.y); },
          [this]() { return pitch_ref_; },
          [this]() { return farol2_utils::deg2rad(nav_state_.orientation_rate.y); },
          [this](double tau) { body_wrench_request_msg_.wrench.torque.y += tau; },
          [this](farol2_pid_controller::msg::PidDebug &debug_msg) {
            debug_msg.error = controller_pitch_->getError();
            debug_msg.p_term = controller_pitch_->getProportionalTerm();
            debug_msg.i_term = controller_pitch_->getIntegralTerm();
            debug_msg.d_term = controller_pitch_->getDerivativeTerm();
            debug_msg.ff_term = controller_pitch_->getFFTerm();
            debug_msg.tau_d = controller_pitch_->getTau_d();
            debug_msg.tau_dot = controller_pitch_->getTauDot();
            debug_msg.tau_sat = controller_pitch_->getTau_sat();
            debug_msg.a_term = controller_pitch_->getAntiWindupTerm();
            debug_msg.tau = controller_pitch_->getOutput();
            debug_msg.state_rate_used = controller_pitch_->state_rate_used_;
          });
        break;

      case ROLL:
        controller_configs_[name] = make_pid_config(
          name,
          ROLL,
          pid_required,
          [this]() { return farol2_utils::deg2rad(nav_state_.orientation.x); },
          [this]() { return roll_ref_; },
          [this]() { return farol2_utils::deg2rad(nav_state_.orientation_rate.x); },
          [this](double tau) { body_wrench_request_msg_.wrench.torque.x += tau; },
          [this](farol2_pid_controller::msg::PidDebug &debug_msg) {
            debug_msg.error = controller_roll_->getError();
            debug_msg.p_term = controller_roll_->getProportionalTerm();
            debug_msg.i_term = controller_roll_->getIntegralTerm();
            debug_msg.d_term = controller_roll_->getDerivativeTerm();
            debug_msg.ff_term = controller_roll_->getFFTerm();
            debug_msg.tau_d = controller_roll_->getTau_d();
            debug_msg.tau_dot = controller_roll_->getTauDot();
            debug_msg.tau_sat = controller_roll_->getTau_sat();
            debug_msg.a_term = controller_roll_->getAntiWindupTerm();
            debug_msg.tau = tau_;
            debug_msg.state_rate_used = controller_roll_->state_rate_used_;
          });
        break;

      case YAW_RATE:
        controller_configs_[name] = make_pi_config(
          name,
          YAW_RATE,
          pi_required,
          [this]() { return farol2_utils::deg2rad(nav_state_.orientation_rate.z); },
          [this]() { return yaw_rate_ref_; },
          [this](double tau) { body_wrench_request_msg_.wrench.torque.z += tau; },
          [this](farol2_pid_controller::msg::PidDebug &debug_msg) {
            debug_msg.error = controller_yaw_rate_->getError();
            debug_msg.p_term = controller_yaw_rate_->getProportionalTerm();
            debug_msg.i_term = controller_yaw_rate_->getIntegralTerm();
            debug_msg.tau_d = controller_yaw_rate_->getTau_d();
            debug_msg.tau_dot = controller_yaw_rate_->getTauDot();
            debug_msg.tau_sat = controller_yaw_rate_->getTau_sat();
            debug_msg.a_term = controller_yaw_rate_->getAntiWindupTerm();
            debug_msg.tau = tau_;
          });
        break;

      case PITCH_RATE:
        controller_configs_[name] = make_pi_config(
          name,
          PITCH_RATE,
          pi_required,
          [this]() { return farol2_utils::deg2rad(nav_state_.orientation_rate.y); },
          [this]() { return pitch_rate_ref_; },
          [this](double tau) { body_wrench_request_msg_.wrench.torque.y += tau; },
          [this](farol2_pid_controller::msg::PidDebug &debug_msg) {
            debug_msg.error = controller_pitch_rate_->getError();
            debug_msg.p_term = controller_pitch_rate_->getProportionalTerm();
            debug_msg.i_term = controller_pitch_rate_->getIntegralTerm();
            debug_msg.tau_d = controller_pitch_rate_->getTau_d();
            debug_msg.tau_dot = controller_pitch_rate_->getTauDot();
            debug_msg.tau_sat = controller_pitch_rate_->getTau_sat();
            debug_msg.a_term = controller_pitch_rate_->getAntiWindupTerm();
            debug_msg.tau = tau_;
          });
        break;

      case ROLL_RATE:
        controller_configs_[name] = make_pi_config(
          name,
          ROLL_RATE,
          pi_required,
          [this]() { return farol2_utils::deg2rad(nav_state_.orientation_rate.x); },
          [this]() { return roll_rate_ref_; },
          [this](double tau) { body_wrench_request_msg_.wrench.torque.x += tau; },
          [this](farol2_pid_controller::msg::PidDebug &debug_msg) {
            debug_msg.error = controller_roll_rate_->getError();
            debug_msg.p_term = controller_roll_rate_->getProportionalTerm();
            debug_msg.i_term = controller_roll_rate_->getIntegralTerm();
            debug_msg.tau_d = controller_roll_rate_->getTau_d();
            debug_msg.tau_dot = controller_roll_rate_->getTauDot();
            debug_msg.tau_sat = controller_roll_rate_->getTau_sat();
            debug_msg.a_term = controller_roll_rate_->getAntiWindupTerm();
            debug_msg.tau = tau_;
          });
        break;
    }
  }
}


/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void PID::timerCallback() {

  auto now = clock_->now();

  double dt = (now - last_update_time_).seconds();
  //RCLCPP_INFO(get_logger(), "dt: %f", dt);

  last_update_time_ = now;

  // Safety guard: skip cycles with invalid or excessively delayed dt.
  if (dt <= 0.0 || dt > 2.0/node_frequency_) { //Check if the dt is correct or if the timer had a big delay
    RCLCPP_WARN(get_logger(), "PID Timer callback - dt value abnormal (%f). Skipping this iteration.", dt);
    return;
  }


  /* Run controllers to update body wrench request */
  callControllers(dt);

  /* Go through existing controllers */
  /* Don't publish if controller hasn't received references */
  static std::set<std::string>::iterator it;
  for (it = controller_names_.begin(); it != controller_names_.end(); it++) {
    /* If controller is not enabled or hasn't received a reference, skip it publishing */
    if (!has_nav_state_ || !controller_parameters_[*it]["enabled"] || !controller_has_reference_[*it] || !hasRecentReference(controller_last_reference_[*it], node_frequency_)) {
      continue;
    }

    /* If controller is enabled */
    /* Publish individual forces and torques messages */
    switch (controller_map_[*it]){
      case SURGE:
        float32_msg_.data = body_wrench_request_msg_.wrench.force.x;
        thrust_x_pub_->publish(float32_msg_);
        break;
      case SWAY:
        float32_msg_.data = body_wrench_request_msg_.wrench.force.y;
        thrust_y_pub_->publish(float32_msg_);
        break;
      case HEAVE:
        float32_msg_.data = body_wrench_request_msg_.wrench.force.z;
        thrust_z_pub_->publish(float32_msg_);
        break;
      case YAW:
        float32_msg_.data = body_wrench_request_msg_.wrench.torque.z;
        torque_z_pub_->publish(float32_msg_);
        break;
      case PITCH:
        float32_msg_.data = body_wrench_request_msg_.wrench.torque.y;
        torque_y_pub_->publish(float32_msg_);
        break;
      case ROLL:
        float32_msg_.data = body_wrench_request_msg_.wrench.torque.x;
        torque_x_pub_->publish(float32_msg_);
        break;
      case YAW_RATE:
        float32_msg_.data = body_wrench_request_msg_.wrench.torque.z;
        torque_z_pub_->publish(float32_msg_);
        break;
      case PITCH_RATE:
        float32_msg_.data = body_wrench_request_msg_.wrench.torque.y;
        torque_y_pub_->publish(float32_msg_);
        break;
      case ROLL_RATE:
        float32_msg_.data = body_wrench_request_msg_.wrench.torque.x;
        torque_x_pub_->publish(float32_msg_);
        break;
    }
  }

  /* Set body wrench request to 0 */
  resetBodyWrenchRequest();

  return;
}

/**
 * @brief Change controllers' parameters callback.
 */
void PID::changeParamsCallback(const std::shared_ptr<farol2_pid_controller::srv::ChangeParams::Request> request,
                               std::shared_ptr<farol2_pid_controller::srv::ChangeParams::Response> response) {
  if (!controller_yaw_) {
    response->success = false;
    response->message = "Yaw controller is not enabled.";
    return;
  }


  /* If required controller does not exist */
  if (controller_names_.find(request->controller) == controller_names_.end()) {
    response->success = false;
    response->message = "Controller " + request->controller + " does not exist - it's not (correctly?) configured in inner_loop.yaml.";
    return;
  }
  // /* If any parameter is invalid */
  // if (request->kp <= 0 || request->ki <= 0 || request->kd <= 0 || request->lpf_wc <= 0 ||
  //     request->tau_min <= 0 || request->tau_max <= 0 || request->tau_min >= request->tau_max) {
  //   response->success = false;
  //   response->message = "Parameter(s) invalid (negative gains/pole/tau, tau_min > tau_max).";
  // }
  if(request->w0 > 0 && request->xi > 0 && request->mr > 0){
    controller_yaw_->kp_ = request->mr*(request->w0*request->w0 + 20*request->xi*request->xi*request->w0*request->w0);
    controller_yaw_->ki_ = request->mr*(10*request->xi*request->w0*request->w0*request->w0);
    controller_yaw_->kd_ = request->mr*(12*request->xi*request->w0);
    response->success = true;
    response->message = "Changed " + request->controller + " controller's params based on w0 and xi. New gains are: kp: " + std::to_string(controller_yaw_->kp_) + " ki: " + std::to_string(controller_yaw_->ki_) + " kd: " + std::to_string(controller_yaw_->kd_);
    tau_ = 0.0;
  }
  else{
    controller_yaw_->kp_ = request->kp;
    controller_yaw_->ki_ = request->ki;
    controller_yaw_->kd_ = request->kd;
    response->success = true;
    response->message = "Changed " + request->controller + " controller's params to specified (kp, ki, kd). New gains are: kp=" + std::to_string(controller_yaw_->kp_) + ", ki=" + std::to_string(controller_yaw_->ki_) + ", kd=" + std::to_string(controller_yaw_->kd_);
    tau_ = 0.0;
  }
  
  /* Set additional parameters if provided */
  if (request->lpf_wc > 0) {
    response->message += "; lpf_wc ignored (ref generator owns LPF now)";
  }
  if (request->tau_min > 0) {
    controller_yaw_->tau_min_ = request->tau_min;
    response->message += "; tau_min=" + std::to_string(request->tau_min);
  }
  if (request->tau_max > 0) {
    controller_yaw_->tau_max_ = request->tau_max;
    response->message += "; tau_max=" + std::to_string(request->tau_max);
  }
  return;
}

void PID::courseControlCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  course_control_ = request->data;
  response->success = true;
  response->message = "Course control flag set to: " + std::string(request->data ? "true" : "false");
}

bool PID::hasRecentReference(const rclcpp::Time &last_reference_timestamp, const int &node_frequency) {
  /* Here it is assumed that a reference must have been received less than 2 times the node period ago */
  /* E.g. if the node is running at 10Hz, the period is 0.1s, so the last reference must have been     */
  /*      received less than 0.2s ago.  
                                                                 */

  double threshold = 20.0/(double)node_frequency;
  static int32_t secs = (int32_t)floor(threshold);
  static uint32_t nanosecs = (uint32_t)((threshold - floor(threshold))*1e9);
  
  RCLCPP_DEBUG(get_logger(), "Now: %ld, Last: %ld, Duration: %ld.", clock_->now().nanoseconds(), last_reference_timestamp.nanoseconds(), rclcpp::Duration(secs, nanosecs).nanoseconds());

  if (clock_->now() - last_reference_timestamp < rclcpp::Duration(secs, nanosecs)) {
    return true;
  }

  return false;
}

void PID::callControllers(double dt) {
  for (const auto &name : controller_names_) {
    // Compute only for enabled channels with fresh references.
    if (!has_nav_state_ || !controller_parameters_[name]["enabled"] || !controller_has_reference_[name] || !hasRecentReference(controller_last_reference_[name], node_frequency_)) {
      continue;
    }

    auto it = controller_configs_.find(name);
    if (it != controller_configs_.end()) {
      executeController(it->second, dt);
    }
  }

  return;
}

void PID::executeController(const ControllerConfig &cfg, double dt) {
  const auto ref_it = reference_outputs_.find(cfg.name);
  const bool has_ref_out = (ref_it != reference_outputs_.end());
  const double ref_used = has_ref_out ? ref_it->second.ref_used_for_control : cfg.get_ref();
  const double dref = has_ref_out ? ref_it->second.dref : 0.0;
  const double ddref = has_ref_out ? ref_it->second.ddref : 0.0;

  // Dispatch by controller type while preserving each channel's PI/PID flavor.
  switch (cfg.type) {
    case SURGE:
      if (!controller_surge_) return;
      tau_ = controller_surge_->callController(cfg.get_state(), cfg.get_ref(), dt);
      break;
    case SWAY:
      if (!controller_sway_) return;
      tau_ = controller_sway_->callController(cfg.get_state(), cfg.get_ref(), dt);
      break;
    case HEAVE:
      if (!controller_heave_) return;
      tau_ = controller_heave_->callController(cfg.get_state(), cfg.get_ref(), dt);
      break;
    case YAW:
      if (!controller_yaw_) return;
      tau_ = controller_yaw_->callController(cfg.get_state(), ref_used, cfg.get_rate(), dref, ddref, dt);
      break;
    case PITCH:
      if (!controller_pitch_) return;
      tau_ = controller_pitch_->callController(cfg.get_state(), ref_used, cfg.get_rate(), dref, ddref, dt);
      break;
    case ROLL:
      if (!controller_roll_) return;
      tau_ = controller_roll_->callController(cfg.get_state(), ref_used, cfg.get_rate(), dref, ddref, dt);
      break;
    case YAW_RATE:
      if (!controller_yaw_rate_) return;
      tau_ = controller_yaw_rate_->callController(cfg.get_state(), cfg.get_ref(), dt);
      break;
    case PITCH_RATE:
      if (!controller_pitch_rate_) return;
      tau_ = controller_pitch_rate_->callController(cfg.get_state(), cfg.get_ref(), dt);
      break;
    case ROLL_RATE:
      if (!controller_roll_rate_) return;
      tau_ = controller_roll_rate_->callController(cfg.get_state(), cfg.get_ref(), dt);
      break;
  }

  farol2_pid_controller::msg::PidDebug debug_msg;
  debug_msg.header.stamp = clock_->now();
  cfg.fill_debug(debug_msg);

  // Keep debug publication behavior per controller as configured.
  if (controller_debug_[cfg.name]) {
    auto it = debug_publishers_.find(cfg.name);
    if (it != debug_publishers_.end()) {
      it->second->publish(debug_msg);
    }
  }

  // Accumulate this channel output into the correct wrench axis.
  cfg.accumulate_output(tau_);
}

void PID::resetBodyWrenchRequest() {
  /* Set body wrench request to 0 */
  body_wrench_request_msg_.wrench.force.x = 0.0;
  body_wrench_request_msg_.wrench.force.y = 0.0;
  body_wrench_request_msg_.wrench.force.z = 0.0;
  body_wrench_request_msg_.wrench.torque.x = 0.0;
  body_wrench_request_msg_.wrench.torque.y = 0.0;
  body_wrench_request_msg_.wrench.torque.z = 0.0;
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PID>());
  rclcpp::shutdown();
  return 0;
}
