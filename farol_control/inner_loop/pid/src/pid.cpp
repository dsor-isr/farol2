#include "pid.hpp"

/* Constructor */
PID::PID() : Node("pid", 
                  rclcpp::NodeOptions()
                    .allow_undeclared_parameters(true)
                    .automatically_declare_parameters_from_overrides(true)),
             controller_surge_(0.0, 0.0, 0.0, 0.0, 0.0),
             controller_sway_(0.0, 0.0, 0.0, 0.0, 0.0),
             controller_heave_(0.0, 0.0, 0.0, 0.0, 0.0),
             controller_yaw_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true, 2, "tustin", "bessel"),
             controller_pitch_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true, 2, "tustin", "bessel"),
             controller_roll_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true, 2, "tustin", "bessel"),
             controller_yaw_rate_(0.0, 0.0, 0.0, 0.0, 0.0),
             controller_pitch_rate_(0.0, 0.0, 0.0, 0.0, 0.0),
             controller_roll_rate_(0.0, 0.0, 0.0, 0.0, 0.0) {
  
  /* Initialise body wrench request forces and torques as 0 */
  resetBodyWrenchRequest();
  
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
  
  createControllers();
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
        "control.inner_loop.pid", raw_controllers_configuration)) {
    /* If no controller configurations were found */
    RCLCPP_ERROR(get_logger(), "No controllers found in control config file.");
    rclcpp::shutdown();
  }

  /* Get list of controllers */
  for (const auto & [key, param] : raw_controllers_configuration) {
    RCLCPP_DEBUG(get_logger(), "Raw controllers: %s", key.c_str());
    /* Add controller to list if there is a flag for enabling the controller */
    if (key.find("enabled") != std::string::npos) {
      /* Get controller name */
      static std::string controller_name;
      controller_name = key.substr(0, key.find('.'));

      /* Add controller name to set */
      controller_names_.insert(controller_name);
    }
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
  course_control_ = this->get_parameter("control.inner_loop.pid.course_control").as_bool();
  lpf_order_ = this->get_parameter("control.inner_loop.pid.lpf_order").as_int();
  lpf_method_ = this->get_parameter("control.inner_loop.pid.lpf_method").as_string();
  lpf_design_ = this->get_parameter("control.inner_loop.pid.lpf_design").as_string();
}

/**
 * @brief Initialise Subscribers
 */
void PID::initialiseSubscribers() {
  nav_state_sub_ = create_subscription<farol_msgs::msg::NavigationState>(
                    get_parameter("control.inner_loop.pid.topics.subscribers.nav_state").as_string(), 
                    1, std::bind(&PID::navStateCallback, this, std::placeholders::_1));

  surge_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.subscribers.surge_ref").as_string(), 
                    1, std::bind(&PID::surgeRefCallback, this, std::placeholders::_1));

  sway_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.subscribers.sway_ref").as_string(), 
                    1, std::bind(&PID::swayRefCallback, this, std::placeholders::_1));

  heave_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.subscribers.heave_ref").as_string(), 
                    1, std::bind(&PID::heaveRefCallback, this, std::placeholders::_1));

  yaw_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.subscribers.yaw_ref").as_string(), 
                    1, std::bind(&PID::yawRefCallback, this, std::placeholders::_1));

  pitch_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.subscribers.pitch_ref").as_string(), 
                    1, std::bind(&PID::pitchRefCallback, this, std::placeholders::_1));

  roll_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.subscribers.roll_ref").as_string(), 
                    1, std::bind(&PID::rollRefCallback, this, std::placeholders::_1));

  yaw_rate_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.subscribers.yaw_rate_ref").as_string(), 
                    1, std::bind(&PID::yawRateRefCallback, this, std::placeholders::_1));

  pitch_rate_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.subscribers.pitch_rate_ref").as_string(), 
                    1, std::bind(&PID::pitchRateRefCallback, this, std::placeholders::_1));

  roll_rate_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.subscribers.roll_rate_ref").as_string(), 
                    1, std::bind(&PID::rollRateRefCallback, this, std::placeholders::_1));

  
}

/**
 * @brief Initialise Publishers
 */
void PID::initialisePublishers() {
  thrust_x_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.publishers.thrust_x").as_string(), 1);

  thrust_y_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.publishers.thrust_y").as_string(), 1);

  thrust_z_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.publishers.thrust_z").as_string(), 1);

  torque_x_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.publishers.torque_x").as_string(), 1);

  torque_y_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.publishers.torque_y").as_string(), 1);

  torque_z_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.pid.topics.publishers.torque_z").as_string(), 1);
  /*
  body_wrench_request_pub_ = create_publisher<control_allocation::msg::BodyWrenchRequest>(
                    get_parameter("control.inner_loop.pid.topics.publishers.body_wrench_request").as_string(), 1);
  
  */
  for (const auto& [name, dbg] : controller_debug_) {
    if (!dbg) continue;
    const auto topic =
        get_parameter("control.inner_loop.pid.topics.publishers.debug." + name).as_string();
    debug_publishers_[name] = create_publisher<pid::msg::PidDebug>(topic, 1);
  }          
}

/**
 * @brief Initialise Services
 */
void PID::initialiseServices() {
  /* Service servers */
  /* Service to change controllers' parameters */
  change_params_srv_ = create_service<pid::srv::ChangeParams>(
                        get_parameter("control.inner_loop.pid.topics.services.change_params").as_string(),
                        std::bind(&PID::changeParamsCallback, this, std::placeholders::_1, std::placeholders::_2));

  /* service clients */
  /*... */

  return;
}

/**
 * @brief Initialise Timers
 */
void PID::initialiseTimers() {
  /* Get node frequency from parameters */
  freq_ = get_parameter("control.inner_loop.pid.node_frequency").as_int();

  /* Create timer */
  timer_ = create_wall_timer
    (std::chrono::milliseconds(int(1.0/freq_*1000)), 
    std::bind(&PID::timerCallback, this));
}

void PID::navStateCallback(const farol_msgs::msg::NavigationState &msg) {
  nav_state_ = msg;
}

void PID::surgeRefCallback(const std_msgs::msg::Float32 &msg) {
  surge_ref_ = msg.data;
  controller_last_reference_["surge"] = clock_.now();
}

void PID::swayRefCallback(const std_msgs::msg::Float32 &msg) {
  sway_ref_ = msg.data;
  controller_last_reference_["sway"] = clock_.now();
}

void PID::heaveRefCallback(const std_msgs::msg::Float32 &msg) {
  heave_ref_ = msg.data;
  controller_last_reference_["heave"] = clock_.now();
}

void PID::yawRefCallback(const std_msgs::msg::Float32 &msg) {
  yaw_ref_ = msg.data;
  controller_last_reference_["yaw"] = clock_.now();
}

void PID::pitchRefCallback(const std_msgs::msg::Float32 &msg) {
  pitch_ref_ = msg.data;
  controller_last_reference_["pitch"] = clock_.now();
}

void PID::rollRefCallback(const std_msgs::msg::Float32 &msg) {
  roll_ref_ = msg.data;
  controller_last_reference_["roll"] = clock_.now();
}

void PID::yawRateRefCallback(const std_msgs::msg::Float32 &msg) {
  yaw_rate_ref_ = msg.data;
  controller_last_reference_["yaw_rate"] = clock_.now();
}

void PID::pitchRateRefCallback(const std_msgs::msg::Float32 &msg) {
  pitch_rate_ref_ = msg.data;
  controller_last_reference_["pitch_rate"] = clock_.now();
}

void PID::rollRateRefCallback(const std_msgs::msg::Float32 &msg) {
  roll_rate_ref_ = msg.data;
  controller_last_reference_["roll_rate"] = clock_.now();
}

void PID::createControllers() {
  /* SURGE */
  /* Check if all parameters exist */
  if (controller_parameters_["surge"].count("kp") == 0 ||
      controller_parameters_["surge"].count("ki") == 0 ||
      controller_parameters_["surge"].count("lpf_wc") == 0 ||
      controller_parameters_["surge"].count("tau_min") == 0 ||
      controller_parameters_["surge"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Surge Controller missing parameters (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  controller_surge_ = ControllerPI(controller_parameters_["surge"]["kp"],
                                   controller_parameters_["surge"]["ki"],
                                   controller_parameters_["surge"]["lpf_wc"],
                                   controller_parameters_["surge"]["tau_min"],
                                   controller_parameters_["surge"]["tau_max"]);
  
  /* SWAY */                                 
  /* Check if all parameters exist */
  if (controller_parameters_["sway"].count("kp") == 0 ||
      controller_parameters_["sway"].count("ki") == 0 ||
      controller_parameters_["sway"].count("lpf_wc") == 0 ||
      controller_parameters_["sway"].count("tau_min") == 0 ||
      controller_parameters_["sway"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Sway Controller missing parameters (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  controller_sway_ = ControllerPI(controller_parameters_["sway"]["kp"],
                                  controller_parameters_["sway"]["ki"],
                                  controller_parameters_["sway"]["lpf_wc"],
                                  controller_parameters_["sway"]["tau_min"],
                                  controller_parameters_["sway"]["tau_max"]);
  
  /* HEAVE */                                
  /* Check if all parameters exist */
  if (controller_parameters_["heave"].count("kp") == 0 ||
      controller_parameters_["heave"].count("ki") == 0 ||
      controller_parameters_["heave"].count("lpf_wc") == 0 ||
      controller_parameters_["heave"].count("tau_min") == 0 ||
      controller_parameters_["heave"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Surge Controller parameters gains (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  controller_heave_ = ControllerPI(controller_parameters_["heave"]["kp"],
                                   controller_parameters_["heave"]["ki"],
                                   controller_parameters_["heave"]["lpf_wc"],
                                   controller_parameters_["heave"]["tau_min"],
                                   controller_parameters_["heave"]["tau_max"]);

  /* YAW */                                 
  /* Check if all parameters exist */
  if (controller_parameters_["yaw"].count("kp") == 0 ||
      controller_parameters_["yaw"].count("ki") == 0 ||
      controller_parameters_["yaw"].count("kd") == 0 ||
      controller_parameters_["yaw"].count("kffa") == 0 ||
      controller_parameters_["yaw"].count("kffv_lin") == 0 ||
      controller_parameters_["yaw"].count("kffv_sq") == 0 ||
      controller_parameters_["yaw"].count("lpf_wc") == 0 ||
      controller_parameters_["yaw"].count("tau_min") == 0 ||
      controller_parameters_["yaw"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Yaw Controller missing parameters (kp, ki, kd, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  controller_yaw_ = ControllerPID(controller_parameters_["yaw"]["kp"], 
                                  controller_parameters_["yaw"]["ki"], 
                                  controller_parameters_["yaw"]["kd"],
                                  controller_parameters_["yaw"]["lpf_wc"],
                                  controller_parameters_["yaw"]["tau_min"],
                                  controller_parameters_["yaw"]["tau_max"],
                                  controller_parameters_["yaw"]["kffv_lin"],
                                  controller_parameters_["yaw"]["kffv_sq"],
                                  controller_parameters_["yaw"]["kffa"],
                                  true,
                                  lpf_order_,
                                  lpf_method_,
                                  lpf_design_);
  
  /* PITCH */                                
  /* Check if all parameters exist */
  if (controller_parameters_["pitch"].count("kp") == 0 ||
      controller_parameters_["pitch"].count("ki") == 0 ||
      controller_parameters_["pitch"].count("kd") == 0 ||
      controller_parameters_["pitch"].count("lpf_wc") == 0 ||
      controller_parameters_["pitch"].count("tau_min") == 0 ||
      controller_parameters_["pitch"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Pitch Controller missing parameters (kp, ki, kd, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  controller_pitch_ = ControllerPID(controller_parameters_["pitch"]["kp"], 
                                    controller_parameters_["pitch"]["ki"], 
                                    controller_parameters_["pitch"]["kd"],
                                    controller_parameters_["pitch"]["lpf_wc"],
                                    controller_parameters_["pitch"]["tau_min"],
                                    controller_parameters_["pitch"]["tau_max"],
                                    0.0,
                                    0.0,
                                    0.0,
                                    true,
                                    lpf_order_,
                                    lpf_method_,
                                    lpf_design_);
  
  /* ROLL */                                  
  /* Check if all parameters exist */
  if (controller_parameters_["roll"].count("kp") == 0 ||
      controller_parameters_["roll"].count("ki") == 0 ||
      controller_parameters_["roll"].count("kd") == 0 ||
      controller_parameters_["roll"].count("lpf_wc") == 0 ||
      controller_parameters_["roll"].count("tau_min") == 0 ||
      controller_parameters_["roll"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Roll Controller missing parameters (kp, ki, kd, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  controller_roll_ = ControllerPID(controller_parameters_["roll"]["kp"], 
                                  controller_parameters_["roll"]["ki"], 
                                  controller_parameters_["roll"]["kd"],
                                  controller_parameters_["roll"]["lpf_wc"],
                                  controller_parameters_["roll"]["tau_min"],
                                  controller_parameters_["roll"]["tau_max"],
                                  0.0,
                                  0.0,
                                  0.0,
                                  true,
                                  lpf_order_,
                                  lpf_method_,
                                  lpf_design_);
  
  /* YAW RATE */                                 
  /* Check if all parameters exist */
  if (controller_parameters_["yaw_rate"].count("kp") == 0 ||
      controller_parameters_["yaw_rate"].count("ki") == 0 ||
      controller_parameters_["yaw_rate"].count("lpf_wc") == 0 ||
      controller_parameters_["yaw_rate"].count("tau_min") == 0 ||
      controller_parameters_["yaw_rate"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Yaw Rate Controller missing parameters (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  controller_yaw_rate_ = ControllerPI(controller_parameters_["yaw_rate"]["kp"],
                                      controller_parameters_["yaw_rate"]["ki"],
                                      controller_parameters_["yaw_rate"]["lpf_wc"],
                                      controller_parameters_["yaw_rate"]["tau_min"],
                                      controller_parameters_["yaw_rate"]["tau_max"]);
  
  /* PITCH RATE */                                    
  /* Check if all parameters exist */
  if (controller_parameters_["pitch_rate"].count("kp") == 0 ||
      controller_parameters_["pitch_rate"].count("ki") == 0 ||
      controller_parameters_["pitch_rate"].count("lpf_wc") == 0 ||
      controller_parameters_["pitch_rate"].count("tau_min") == 0 ||
      controller_parameters_["pitch_rate"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Pitch Rate Controller missing parameters (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  controller_pitch_rate_ = ControllerPI(controller_parameters_["pitch_rate"]["kp"],
                                        controller_parameters_["pitch_rate"]["ki"],
                                        controller_parameters_["pitch_rate"]["lpf_wc"],
                                        controller_parameters_["pitch_rate"]["tau_min"],
                                        controller_parameters_["pitch_rate"]["tau_max"]);

  /* ROLL RATE */                                      
  /* Check if all parameters exist */
  if (controller_parameters_["roll_rate"].count("kp") == 0 ||
      controller_parameters_["roll_rate"].count("ki") == 0 ||
      controller_parameters_["roll_rate"].count("lpf_wc") == 0 ||
      controller_parameters_["roll_rate"].count("tau_min") == 0 ||
      controller_parameters_["roll_rate"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Roll Rate Controller missing parameters (kp, ki, lpf_wc, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  controller_roll_rate_ = ControllerPI(controller_parameters_["roll_rate"]["kp"],
                                       controller_parameters_["roll_rate"]["ki"],
                                       controller_parameters_["roll_rate"]["lpf_wc"],
                                       controller_parameters_["roll_rate"]["tau_min"],
                                       controller_parameters_["roll_rate"]["tau_max"]);
}


/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void PID::timerCallback() {
  /* Run controllers to update body wrench request */
  callControllers();

  /* Go through existing controllers */
  /* Don't publish if controller hasn't received references */
  static std::set<std::string>::iterator it;
  for (it = controller_names_.begin(); it != controller_names_.end(); it++) {
    /* If controller is not enabled or hasn't received a reference, skip it publishing */
    if (!controller_parameters_[*it]["enabled"] || !hasRecentReference(controller_last_reference_[*it], freq_)) {
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
      case ATTITUDE:
        float32_msg_.data = body_wrench_request_msg_.wrench.torque.z;
        torque_z_pub_->publish(float32_msg_);

        float32_msg_.data = body_wrench_request_msg_.wrench.torque.y;
        torque_y_pub_->publish(float32_msg_);

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
void PID::changeParamsCallback(const std::shared_ptr<pid::srv::ChangeParams::Request> request,
                               std::shared_ptr<pid::srv::ChangeParams::Response> response) {

  /* If required controller does not exist */
  if (controller_names_.find(request->controller) == controller_names_.end()) {
    response->success = false;
    response->message = "Controller " + request->controller + " does not exist - it's not (correctly?) configured in control.yaml.";
    return;
  }

  /* If any parameter is invalid */
  if (request->kp <= 0 || request->ki <= 0 || request->kd <= 0 || request->lpf_wc <= 0 ||
      request->tau_min <= 0 || request->tau_max <= 0 || request->tau_min >= request->tau_max) {
    response->success = false;
    response->message = "Parameter(s) invalid (negative gains/pole/tau, tau_min > tau_max).";
  }

}

bool PID::hasRecentReference(const rclcpp::Time &last_reference_timestamp, const int &node_frequency) {
  /* Here it is assumed that a reference must have been received less than 2 times the node period ago */
  /* E.g. if the node is running at 10Hz, the period is 0.1s, so the last reference must have been     */
  /*      received less than 0.2s ago.                                                                 */

  static double threshold = 2.0/(double)node_frequency;
  static int32_t secs = (int32_t)floor(threshold);
  static uint32_t nanosecs = (uint32_t)((threshold - floor(threshold))*1e9);
  
  RCLCPP_DEBUG(get_logger(), "Now: %ld, Last: %ld, Duration: %ld.", clock_.now().nanoseconds(), last_reference_timestamp.nanoseconds(), rclcpp::Duration(secs, nanosecs).nanoseconds());

  if (clock_.now() - last_reference_timestamp < rclcpp::Duration(secs, nanosecs)) {
    return true;
  }

  return false;
}

void PID::callControllers() {
  static std::set<std::string>::iterator it;
  
  /* Go through existing controllers */
  for (it = controller_names_.begin(); it != controller_names_.end(); it++) {
    /* If controller is not enabled or hasn't received a reference, skip it */
    if (!controller_parameters_[*it]["enabled"] || !hasRecentReference(controller_last_reference_[*it], freq_)) {
      continue;
    }

    /* If controller is enabled */
    switch (controller_map_[*it]){
      case SURGE:
        callControllerSurge();
        break;
      case SWAY:
        callControllerSway();
        break;
      case HEAVE:
        callControllerHeave();
        break;
      case YAW:
        callControllerYaw();
        break;
      case PITCH:
        callControllerPitch();
        break;
      case ROLL:
        callControllerRoll();
        break;
      case YAW_RATE:
        callControllerYawRate();
        break;
      case PITCH_RATE:
        callControllerPitchRate();
        break;
      case ROLL_RATE:
        callControllerRollRate();
        break;
      case ATTITUDE:
        callControllerAttitude();
        break;
    }
  }

  return;
}

void PID::callControllerSurge() {
  /* Call PI controller */
  tau_ = controller_surge_.callController(nav_state_.body_velocity_fluid.x, surge_ref_, 1.0/freq_);

  pid::msg::PidDebug debug_msg;
  debug_msg.header.stamp = clock_.now();
  debug_msg.error = controller_surge_.getError();
  debug_msg.p_term = controller_surge_.getProportionalTerm();
  debug_msg.i_term = controller_surge_.getIntegralTerm();
  debug_msg.tau_d = controller_surge_.getTau_d();
  debug_msg.tau_dot = controller_surge_.getTauDot();
  debug_msg.tau_sat = controller_surge_.getTau_sat();
  debug_msg.a_term = controller_surge_.getAntiWindupTerm();
  debug_msg.tau = tau_;

  if (controller_debug_["surge"]) {
  auto it = debug_publishers_.find("surge");
  if (it != debug_publishers_.end()) it->second->publish(debug_msg);
  }

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.force.x += tau_;

  return;
};

void PID::callControllerSway() {
  /* Call PI controller */
  tau_ = controller_sway_.callController(nav_state_.body_velocity_fluid.y, sway_ref_, 1.0/freq_);

  pid::msg::PidDebug debug_msg;
  debug_msg.header.stamp = clock_.now();
  debug_msg.error = controller_sway_.getError();
  debug_msg.p_term = controller_sway_.getProportionalTerm();
  debug_msg.i_term = controller_sway_.getIntegralTerm();
  debug_msg.tau_d = controller_sway_.getTau_d();
  debug_msg.tau_dot = controller_sway_.getTauDot();
  debug_msg.tau_sat = controller_sway_.getTau_sat();
  debug_msg.a_term = controller_sway_.getAntiWindupTerm();
  debug_msg.tau = tau_;

  if (controller_debug_["sway"]) {
  auto it = debug_publishers_.find("sway");
  if (it != debug_publishers_.end()) it->second->publish(debug_msg);
  }

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.force.y += tau_;
  
  return;
};

void PID::callControllerHeave() {
  /* Call PI controller */
  tau_ = controller_heave_.callController(nav_state_.body_velocity_fluid.z, heave_ref_, 1.0/freq_);

    pid::msg::PidDebug debug_msg;
  debug_msg.header.stamp = clock_.now();
  debug_msg.error = controller_heave_.getError();
  debug_msg.p_term = controller_heave_.getProportionalTerm();
  debug_msg.i_term = controller_heave_.getIntegralTerm();
  debug_msg.tau_d = controller_heave_.getTau_d();
  debug_msg.tau_dot = controller_heave_.getTauDot();
  debug_msg.tau_sat = controller_heave_.getTau_sat();
  debug_msg.a_term = controller_heave_.getAntiWindupTerm();
  debug_msg.tau = tau_;

  if (controller_debug_["heave"]) {
  auto it = debug_publishers_.find("heave");
  if (it != debug_publishers_.end()) it->second->publish(debug_msg);
  }

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.force.z += tau_;
  
  return;
};

void PID::callControllerYaw() {
  /* Call PID controller */
  if(course_control_)
    tau_ = controller_yaw_.callController(nav_state_.course_angle, yaw_ref_, nav_state_.orientation_rate.z, 1.0/freq_);
  else 
    tau_ = controller_yaw_.callController(nav_state_.orientation.z, yaw_ref_, nav_state_.orientation_rate.z, 1.0/freq_);


  pid::msg::PidDebug debug_msg;
  debug_msg.header.stamp = clock_.now();
  debug_msg.error = controller_yaw_.getError();
  debug_msg.error_rate = controller_yaw_.error_rate_;
  debug_msg.error_rate_dot = controller_yaw_.error_rate_dot_;
  debug_msg.p_term = controller_yaw_.getProportionalTerm();
  debug_msg.i_term = controller_yaw_.getIntegralTerm();
  debug_msg.d_term = controller_yaw_.getDerivativeTerm();
  debug_msg.tau_d = controller_yaw_.getTau_d();
  debug_msg.tau_dot = controller_yaw_.getTauDot();
  debug_msg.tau_sat = controller_yaw_.getTau_sat();
  debug_msg.a_term = controller_yaw_.getAntiWindupTerm();
  debug_msg.tau = tau_;
  debug_msg.state = controller_yaw_.state_;
  debug_msg.ref_raw = farol_utils::wrapTo2Pi(controller_yaw_.ref_raw_);
  debug_msg.ref_filt = farol_utils::wrapTo2Pi(controller_yaw_.ref_);
  debug_msg.dref_filt = controller_yaw_.dref_;
  debug_msg.ddref_filt = controller_yaw_.ddref_;

  if (controller_debug_["yaw"]) {
  auto it = debug_publishers_.find("yaw");
  if (it != debug_publishers_.end()) it->second->publish(debug_msg);
  }


  /* Update body wrench request */
  body_wrench_request_msg_.wrench.torque.z += tau_;

  return;
};

void PID::callControllerPitch() {
  /* Call PID controller */
  tau_ = controller_pitch_.callController(nav_state_.orientation.y, pitch_ref_, nav_state_.orientation_rate.y, 1.0/freq_);

  pid::msg::PidDebug debug_msg;
  debug_msg.header.stamp = clock_.now();
  debug_msg.error = controller_pitch_.getError();
  debug_msg.p_term = controller_pitch_.getProportionalTerm();
  debug_msg.i_term = controller_pitch_.getIntegralTerm();
  debug_msg.d_term = controller_pitch_.getDerivativeTerm();
  debug_msg.tau_d = controller_pitch_.getTau_d();
  debug_msg.tau_dot = controller_pitch_.getTauDot();
  debug_msg.tau_sat = controller_pitch_.getTau_sat();
  debug_msg.a_term = controller_pitch_.getAntiWindupTerm();
  debug_msg.tau = tau_;

  if (controller_debug_["pitch"]) {
  auto it = debug_publishers_.find("pitch");
  if (it != debug_publishers_.end()) it->second->publish(debug_msg);
  }


  /* Update body wrench request */
  body_wrench_request_msg_.wrench.torque.y += tau_;

  return;
};

void PID::callControllerRoll() {
  /* Call PID controller */
  tau_ = controller_roll_.callController(nav_state_.orientation.x, roll_ref_, nav_state_.orientation_rate.x, 1.0/freq_);

  pid::msg::PidDebug debug_msg;
  debug_msg.header.stamp = clock_.now();
  debug_msg.error = controller_roll_.getError();
  debug_msg.p_term = controller_roll_.getProportionalTerm();
  debug_msg.i_term = controller_roll_.getIntegralTerm();
  debug_msg.d_term = controller_roll_.getDerivativeTerm();
  debug_msg.tau_d = controller_roll_.getTau_d();
  debug_msg.tau_dot = controller_roll_.getTauDot();
  debug_msg.tau_sat = controller_roll_.getTau_sat();
  debug_msg.a_term = controller_roll_.getAntiWindupTerm();
  debug_msg.tau = tau_;

  if (controller_debug_["roll"]) {
  auto it = debug_publishers_.find("roll");
  if (it != debug_publishers_.end()) it->second->publish(debug_msg);
  }


  /* Update body wrench request */
  body_wrench_request_msg_.wrench.torque.x += tau_;

  return;
};

void PID::callControllerYawRate() {
  /* Call PI controller */
  tau_ = controller_yaw_rate_.callController(nav_state_.orientation_rate.z, yaw_rate_ref_, 1.0/freq_);

  pid::msg::PidDebug debug_msg;
  debug_msg.header.stamp = clock_.now();
  debug_msg.error = controller_yaw_rate_.getError();
  debug_msg.p_term = controller_yaw_rate_.getProportionalTerm();
  debug_msg.i_term = controller_yaw_rate_.getIntegralTerm();
  debug_msg.tau_d = controller_yaw_rate_.getTau_d();
  debug_msg.tau_dot = controller_yaw_rate_.getTauDot();
  debug_msg.tau_sat = controller_yaw_rate_.getTau_sat();
  debug_msg.a_term = controller_yaw_rate_.getAntiWindupTerm();
  debug_msg.tau = tau_;

  if (controller_debug_["yaw_rate"]) {
  auto it = debug_publishers_.find("yaw_rate");
  if (it != debug_publishers_.end()) it->second->publish(debug_msg);
  }

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.torque.z += tau_;

  return;
};

void PID::callControllerPitchRate() {
  /* Call PI controller */
  tau_ = controller_pitch_rate_.callController(nav_state_.orientation_rate.y, pitch_rate_ref_, 1.0/freq_);

  pid::msg::PidDebug debug_msg;
  debug_msg.header.stamp = clock_.now();
  debug_msg.error = controller_pitch_rate_.getError();
  debug_msg.p_term = controller_pitch_rate_.getProportionalTerm();
  debug_msg.i_term = controller_pitch_rate_.getIntegralTerm();
  debug_msg.tau_d = controller_pitch_rate_.getTau_d();
  debug_msg.tau_dot = controller_pitch_rate_.getTauDot();
  debug_msg.tau_sat = controller_pitch_rate_.getTau_sat();
  debug_msg.a_term = controller_pitch_rate_.getAntiWindupTerm();
  debug_msg.tau = tau_;

  if (controller_debug_["pitch_rate"]) {
  auto it = debug_publishers_.find("pitch_rate");
  if (it != debug_publishers_.end()) it->second->publish(debug_msg);
  }

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.torque.y += tau_;

  return;
};

void PID::callControllerRollRate() {
  /* Call PI controller */
  tau_ = controller_roll_rate_.callController(nav_state_.orientation_rate.x, roll_rate_ref_, 1.0/freq_);

  pid::msg::PidDebug debug_msg;
  debug_msg.header.stamp = clock_.now();
  debug_msg.error = controller_roll_rate_.getError();
  debug_msg.p_term = controller_roll_rate_.getProportionalTerm();
  debug_msg.i_term = controller_roll_rate_.getIntegralTerm();
  debug_msg.tau_d = controller_roll_rate_.getTau_d();
  debug_msg.tau_dot = controller_roll_rate_.getTauDot();
  debug_msg.tau_sat = controller_roll_rate_.getTau_sat();
  debug_msg.a_term = controller_roll_rate_.getAntiWindupTerm();
  debug_msg.tau = tau_;

  if (controller_debug_["roll_rate"]) {
  auto it = debug_publishers_.find("roll_rate");
  if (it != debug_publishers_.end()) it->second->publish(debug_msg);
  }

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.torque.x += tau_;

  return;
};

void PID::callControllerAttitude() {
  /* To be implemented */
  return;
};

void PID::resetBodyWrenchRequest() {
  /* Set body wrench request to 0 */
  body_wrench_request_msg_.wrench.force.x = 0.0;
  body_wrench_request_msg_.wrench.force.y = 0.0;
  body_wrench_request_msg_.wrench.force.z = 0.0;
  body_wrench_request_msg_.wrench.torque.x = 0.0;
  body_wrench_request_msg_.wrench.torque.y = 0.0;
  body_wrench_request_msg_.wrench.torque.z = 0.0;
}

/************************/
/*    PI CONTROLLER     */
/************************/

/* Constructor */
ControllerPI::ControllerPI(double kp, double ki, double lpf_wc, double tau_min, double tau_max) {
  /* Set parameters */
  kp_ = kp; 
  ki_ = ki; 
  lpf_wc_ = lpf_wc;
  tau_min_ = tau_min;
  tau_max_ = tau_max;
}

double ControllerPI::callController(double state, double state_ref, double dt) {
  /* Compute error */

  //check for nan
  
  error_ = state_ref - state;

  /*Compute derivative of Kp term if not in first iteration*/ 
  if (!first_it_) {
    state_dot_ = (state - state_prev_) / dt;





    /*Apply low pass filter due to noise amplification from derivative computation*/ 
  lpf_A_ = std::exp(- lpf_wc_*dt);
  lpf_B_ = 1 - lpf_A_;
  state_dot_filter_ = lpf_A_*state_dot_filter_prev_ + lpf_B_*state_dot_;

  } else {
    /*Reset first iteration flag*/ 
    first_it_ = false;
  }

  /*Add all PI terms*/ 
  tau_d_ = ki_*error_ - kp_*state_dot_filter_;

  /*Anti-windup*/ 
  Ka_ = 1.0/dt;
  tau_dot_ = tau_d_ - Ka_*(tau_prev_ - tau_sat_prev_);
  tau_ = tau_prev_ + tau_dot_*dt;
  tau_sat_ = std::clamp(tau_, tau_min_, tau_max_);

  state_prev_ = state;
  state_dot_filter_prev_ = state_dot_filter_;
  tau_prev_ = tau_;
  tau_sat_prev_ = tau_sat_;

  return tau_sat_;
}

void ControllerPI::setParams(double kp, double ki, double lpf_wc, double tau_min, double tau_max) {
  kp_ = kp; 
  ki_ = ki; 
  lpf_wc_ = lpf_wc; 
  tau_min_ = tau_min; 
  tau_max_ = tau_max;
}

/************************/
/*    PID CONTROLLER    */
/************************/

/* Constructor */
ControllerPID::ControllerPID(double kp, double ki, double kd, double lpf_wc, double tau_min, double tau_max, double kffv_lin, double kffv_sq, double kffa, bool wrapToPi, int lpf_order, std::string lpf_method, std::string lpf_design) {
  /* Set parameters */
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  kffv_lin_ = kffv_lin;
  kffv_sq_ = kffv_sq;
  kffa_ = kffa;
  lpf_wc_ = (lpf_wc > 0.0) ? lpf_wc : 1.0;
  tau_min_ = tau_min;
  tau_max_ = tau_max;
  wrapToPi_ = wrapToPi;

  lpf_.configure(lpf_wc_, 0.1, lpf_order, lpf_design, lpf_method, wrapToPi_); 
  // lpf_.configure(lpf_wc_, 0.1, lpf_design, lpf_method, wrapToPi_); 
}

/* Delta implementation for PID */
double ControllerPID::callController(double state, double state_ref, double state_rate, double dt) {
  ref_raw_ = state_ref;
  state_ = state;
  
  /* Pass reference signal through LPF to extract reference derivatives for ff terms*/
  lpf_.step(state_ref, dt);
  ref_ = lpf_.y();
  dref_ = lpf_.dy();
  ddref_ = lpf_.ddy();
  
  // Compute error 
  error_ = state - state_ref;
  if (wrapToPi_) // Wrap to [-pi, pi] if needed */
    error_ = farol_utils::wrapToPi(error_);  
  // Compute error derivative
  error_rate_ = state_rate - dref_;
  
  // Compute derivative of all terms execpt the integral
  if (!first_it_) {
    state_rate_dot_ = (state_rate - state_rate_prev_) / dt;
    error_dot_ = (error_ - error_prev_) / dt;
    error_rate_dot_ = (error_rate_ - error_rate_prev_) / dt;
    ddref_dot_ =  (ddref_ - ddref_prev_)/dt; 
  } else 
    first_it_ = false; // Reset first iteration flag 

  // Compute error second derivative
  // error_rate_dot_ = state_rate_dot_- ddref_;

  /* Add all PID terms */
  // tau_d_ = -ki_*error_ - kp_*error_rate_ - kd_*error_rate_dot_ + kffa_*dddref_ - kffv_lin_*state_rate_dot_- kffv_sq_*state_rate_dot_*abs(state_rate_dot_);
  tau_d_ = -ki_*error_ - kp_*error_dot_ - kd_*error_rate_dot_ + kffa_*ddref_dot_ - kffv_lin_*state_rate_dot_- kffv_sq_*state_rate_dot_*abs(state_rate_dot_);

  /* Anti-windup */
  Ka_ = 1.0/dt;
  tau_dot_ = tau_d_ - Ka_*(tau_prev_ - tau_sat_prev_);
  tau_ = tau_prev_ + tau_dot_*dt ;
  tau_sat_ = std::clamp(tau_, tau_min_, tau_max_);

  /* Set prev values */
  error_prev_ = error_;
  state_rate_prev_ = state_rate;
  error_rate_prev_= error_rate_;
  state_rate_dot_filter_prev_ = state_rate_dot_filter_;
  tau_prev_ = tau_;
  tau_sat_prev_ = tau_sat_;
  ddref_prev_ = ddref_;

  return tau_sat_;
}

/* Regular implementation for PID */
// double ControllerPID::callController(double state, double state_ref, double state_rate, double dt) {
//   ref_raw_ = state_ref;
//   state_ = state;
  
//   // Compute error 
//   error_ = state - state_ref;
//   if (wrapToPi_) // Wrap to [-pi, pi] if needed */
//     error_ = farol_utils::wrapToPi(error_);  

//     // Compute error derivative
//   error_rate_ = state_rate;
  

//   tau_d_ = -ki_*error_;

//   /* Anti-windup */
//   Ka_ = 1.0/dt;
//   tau_dot_ = tau_d_ - Ka_*(tau_prev_ - tau_sat_prev_);
//   tau_ = tau_prev_ + tau_dot_*dt ;
//   tau_sat_ = std::clamp(tau_, tau_min_, tau_max_);

//   /* Set prev values */
//   error_prev_ = error_;
//   state_rate_prev_ = state_rate;
//   error_rate_prev_= error_rate_;
//   state_rate_dot_filter_prev_ = state_rate_dot_filter_;
//   tau_prev_ = tau_;
//   tau_sat_prev_ = tau_sat_;
//   ddref_prev_ = ddref_;

//   return tau_sat_-kp_ *error_ + -kd_*error_rate_;
// }

void ControllerPID::setParams(double kp, double ki, double kd, double lpf_wc, double tau_min, double tau_max, double kffv_lin, double kffv_sq, double kffa) {
  kp_ = kp; 
  ki_ = ki; 
  kd_ = kd; 
  kffv_lin_ = kffv_lin; 
  kffv_sq_ = kffv_sq; 
  kffa_ = kffa; 
  lpf_wc_ = lpf_wc; 
  tau_min_ = tau_min; 
  tau_max_ = tau_max;
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
