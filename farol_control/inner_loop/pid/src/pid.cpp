#include "pid.hpp"

/* Constructor */
PID::PID() : Node("pid", 
                  rclcpp::NodeOptions()
                    .allow_undeclared_parameters(true)
                    .automatically_declare_parameters_from_overrides(true)) {
  
  /* Initialise body wrench request forces and torques as 0 */
  resetBodyWrenchRequest();
  
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
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
          controller_parameters_[name].insert({param_name, param.as_bool()});
          RCLCPP_DEBUG(get_logger(), "BOOL %s: %d", key.c_str(), param.as_bool());
        } else { /* Default case, parameter is a double */
          controller_parameters_[name].insert({param_name, param.as_double()});
          RCLCPP_DEBUG(get_logger(), "DOUBLE %s: %f", key.c_str(), param.as_double());
        }
      } catch(...) {
        RCLCPP_INFO(get_logger(), "Unexpected parameter (not boolean or double) found for controller %s: %s", name.c_str(), param_name.c_str());
        continue;
      }
    }
  }
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
  body_wrench_request_pub_ = create_publisher<control_allocation::msg::BodyWrenchRequest>(
                              get_parameter("control.inner_loop.pid.topics.publishers.body_wrench_request").as_string(), 1);
}

/**
 * @brief Initialise Services
 */
void PID::initialiseServices() {
  /* Service servers */
  /*... */

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
  timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq_*1000)), std::bind(&PID::timerCallback, this));
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


/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void PID::timerCallback() {
  /* Run controllers to update body wrench request */
  callControllers();

  /* Publish body wrench request message */
  body_wrench_request_msg_.header.stamp = clock_.now();
  body_wrench_request_pub_->publish(body_wrench_request_msg_);

  /* Set body wrench request to 0 */
  resetBodyWrenchRequest();

  return;
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
  /* Check if all parameters exist */
  if (controller_parameters_["surge"].count("kp") == 0 ||
      controller_parameters_["surge"].count("ki") == 0 ||
      controller_parameters_["surge"].count("lpf_pole") == 0 ||
      controller_parameters_["surge"].count("tau_min") == 0 ||
      controller_parameters_["surge"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Surge Controller missing parameters (kp, ki, lpf_pole, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  static ControllerPI controller_surge_ = ControllerPI(controller_parameters_["surge"]["kp"],
                                                       controller_parameters_["surge"]["ki"],
                                                       controller_parameters_["surge"]["lpf_pole"],
                                                       controller_parameters_["surge"]["tau_min"],
                                                       controller_parameters_["surge"]["tau_max"]);

  /* Call PI controller */
  tau_ = controller_surge_.callController(nav_state_.body_velocity_fluid.x, surge_ref_, 1.0/freq_);

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.force.x += tau_;

  return;
};

void PID::callControllerSway() {
  /* Check if all parameters exist */
  if (controller_parameters_["sway"].count("kp") == 0 ||
      controller_parameters_["sway"].count("ki") == 0 ||
      controller_parameters_["sway"].count("lpf_pole") == 0 ||
      controller_parameters_["sway"].count("tau_min") == 0 ||
      controller_parameters_["sway"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Sway Controller missing parameters (kp, ki, lpf_pole, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  static ControllerPI controller_sway_ = ControllerPI(controller_parameters_["sway"]["kp"],
                                                      controller_parameters_["sway"]["ki"],
                                                      controller_parameters_["sway"]["lpf_pole"],
                                                      controller_parameters_["sway"]["tau_min"],
                                                      controller_parameters_["sway"]["tau_max"]);

  /* Call PI controller */
  tau_ = controller_sway_.callController(nav_state_.body_velocity_fluid.y, sway_ref_, 1.0/freq_);

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.force.y += tau_;
  
  return;
};

void PID::callControllerHeave() {
  /* Check if all parameters exist */
  if (controller_parameters_["heave"].count("kp") == 0 ||
      controller_parameters_["heave"].count("ki") == 0 ||
      controller_parameters_["heave"].count("lpf_pole") == 0 ||
      controller_parameters_["heave"].count("tau_min") == 0 ||
      controller_parameters_["heave"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Surge Controller parameters gains (kp, ki, lpf_pole, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  static ControllerPI controller_heave_ = ControllerPI(controller_parameters_["heave"]["kp"],
                                                       controller_parameters_["heave"]["ki"],
                                                       controller_parameters_["heave"]["lpf_pole"],
                                                       controller_parameters_["heave"]["tau_min"],
                                                       controller_parameters_["heave"]["tau_max"]);

  /* Call PI controller */
  tau_ = controller_heave_.callController(nav_state_.body_velocity_fluid.z, heave_ref_, 1.0/freq_);

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.force.z += tau_;
  
  return;
};

void PID::callControllerYaw() {
  /* Check if all parameters exist */
  if (controller_parameters_["yaw"].count("kp") == 0 ||
      controller_parameters_["yaw"].count("ki") == 0 ||
      controller_parameters_["yaw"].count("kd") == 0 ||
      controller_parameters_["yaw"].count("lpf_pole") == 0 ||
      controller_parameters_["yaw"].count("tau_min") == 0 ||
      controller_parameters_["yaw"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Yaw Controller missing parameters (kp, ki, kd, lpf_pole, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  static ControllerPID controller_yaw_ = ControllerPID(controller_parameters_["yaw"]["kp"], 
                                                       controller_parameters_["yaw"]["ki"], 
                                                       controller_parameters_["yaw"]["kd"],
                                                       controller_parameters_["yaw"]["lpf_pole"],
                                                       controller_parameters_["yaw"]["tau_min"],
                                                       controller_parameters_["yaw"]["tau_max"],
                                                       true);

  /* Call PID controller */
  tau_ = controller_yaw_.callController(nav_state_.orientation.z, yaw_ref_, nav_state_.orientation_rate.z, 1.0/freq_);

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.torque.z += tau_;

  return;
};

void PID::callControllerPitch() {
  /* Check if all parameters exist */
  if (controller_parameters_["pitch"].count("kp") == 0 ||
      controller_parameters_["pitch"].count("ki") == 0 ||
      controller_parameters_["pitch"].count("kd") == 0 ||
      controller_parameters_["pitch"].count("lpf_pole") == 0 ||
      controller_parameters_["pitch"].count("tau_min") == 0 ||
      controller_parameters_["pitch"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Pitch Controller missing parameters (kp, ki, kd, lpf_pole, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  static ControllerPID controller_pitch_ = ControllerPID(controller_parameters_["pitch"]["kp"], 
                                                         controller_parameters_["pitch"]["ki"], 
                                                         controller_parameters_["pitch"]["kd"],
                                                         controller_parameters_["pitch"]["lpf_pole"],
                                                         controller_parameters_["pitch"]["tau_min"],
                                                         controller_parameters_["pitch"]["tau_max"],
                                                         true);

  /* Call PID controller */
  tau_ = controller_pitch_.callController(nav_state_.orientation.y, pitch_ref_, nav_state_.orientation_rate.y, 1.0/freq_);

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.torque.y += tau_;

  return;
};

void PID::callControllerRoll() {
  /* Check if all parameters exist */
  if (controller_parameters_["roll"].count("kp") == 0 ||
      controller_parameters_["roll"].count("ki") == 0 ||
      controller_parameters_["roll"].count("kd") == 0 ||
      controller_parameters_["roll"].count("lpf_pole") == 0 ||
      controller_parameters_["roll"].count("tau_min") == 0 ||
      controller_parameters_["roll"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Roll Controller missing parameters (kp, ki, kd, lpf_pole, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  static ControllerPID controller_roll_ = ControllerPID(controller_parameters_["roll"]["kp"], 
                                                        controller_parameters_["roll"]["ki"], 
                                                        controller_parameters_["roll"]["kd"],
                                                        controller_parameters_["roll"]["lpf_pole"],
                                                        controller_parameters_["roll"]["tau_min"],
                                                        controller_parameters_["roll"]["tau_max"],
                                                        true);

  /* Call PID controller */
  tau_ = controller_roll_.callController(nav_state_.orientation.x, roll_ref_, nav_state_.orientation_rate.x, 1.0/freq_);

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.torque.x += tau_;

  return;
};

void PID::callControllerYawRate() {
  /* Check if all parameters exist */
  if (controller_parameters_["yaw_rate"].count("kp") == 0 ||
      controller_parameters_["yaw_rate"].count("ki") == 0 ||
      controller_parameters_["yaw_rate"].count("lpf_pole") == 0 ||
      controller_parameters_["yaw_rate"].count("tau_min") == 0 ||
      controller_parameters_["yaw_rate"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Yaw Rate Controller missing parameters (kp, ki, lpf_pole, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  static ControllerPI controller_yaw_rate_ = ControllerPI(controller_parameters_["yaw_rate"]["kp"],
                                                          controller_parameters_["yaw_rate"]["ki"],
                                                          controller_parameters_["yaw_rate"]["lpf_pole"],
                                                          controller_parameters_["yaw_rate"]["tau_min"],
                                                          controller_parameters_["yaw_rate"]["tau_max"]);

  /* Call PI controller */
  tau_ = controller_yaw_rate_.callController(nav_state_.orientation_rate.z, yaw_rate_ref_, 1.0/freq_);

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.torque.z += tau_;

  return;
};

void PID::callControllerPitchRate() {
  /* Check if all parameters exist */
  if (controller_parameters_["pitch_rate"].count("kp") == 0 ||
      controller_parameters_["pitch_rate"].count("ki") == 0 ||
      controller_parameters_["pitch_rate"].count("lpf_pole") == 0 ||
      controller_parameters_["pitch_rate"].count("tau_min") == 0 ||
      controller_parameters_["pitch_rate"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Pitch Rate Controller missing parameters (kp, ki, lpf_pole, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  static ControllerPI controller_pitch_rate_ = ControllerPI(controller_parameters_["pitch_rate"]["kp"],
                                                            controller_parameters_["pitch_rate"]["ki"],
                                                            controller_parameters_["pitch_rate"]["lpf_pole"],
                                                            controller_parameters_["pitch_rate"]["tau_min"],
                                                            controller_parameters_["pitch_rate"]["tau_max"]);

  /* Call PI controller */
  tau_ = controller_pitch_rate_.callController(nav_state_.orientation_rate.y, pitch_rate_ref_, 1.0/freq_);

  /* Update body wrench request */
  body_wrench_request_msg_.wrench.torque.y += tau_;

  return;
};

void PID::callControllerRollRate() {
  /* Check if all parameters exist */
  if (controller_parameters_["roll_rate"].count("kp") == 0 ||
      controller_parameters_["roll_rate"].count("ki") == 0 ||
      controller_parameters_["roll_rate"].count("lpf_pole") == 0 ||
      controller_parameters_["roll_rate"].count("tau_min") == 0 ||
      controller_parameters_["roll_rate"].count("tau_max") == 0) {
    RCLCPP_ERROR(get_logger(), "Roll Rate Controller missing parameters (kp, ki, lpf_pole, tau_min or tau_max).");
    rclcpp::shutdown();
  }

  /* Create object for the controller itself */
  static ControllerPI controller_roll_rate_ = ControllerPI(controller_parameters_["roll_rate"]["kp"],
                                                           controller_parameters_["roll_rate"]["ki"],
                                                           controller_parameters_["roll_rate"]["lpf_pole"],
                                                           controller_parameters_["roll_rate"]["tau_min"],
                                                           controller_parameters_["roll_rate"]["tau_max"]);

  /* Call PI controller */
  tau_ = controller_roll_rate_.callController(nav_state_.orientation_rate.x, roll_rate_ref_, 1.0/freq_);

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
ControllerPI::ControllerPI(double kp, double ki, double lpf_pole, double tau_min, double tau_max) {
  /* Set parameters */
  kp_ = kp; 
  ki_ = ki; 
  lpf_pole_ = lpf_pole;
  tau_min_ = tau_min;
  tau_max_ = tau_max;
}

double ControllerPI::callController(double state, double state_ref, double dt) {
  static double error, tau_d;
  static bool first_it = true;
  static double state_prev = 0.0, state_dot = 0.0, state_dot_filter = 0.0, state_dot_filter_prev = 0.0;
  static double lpf_A = 0.0, lpf_B = 0.0;
  static double Ka, tau_dot, tau, tau_prev, tau_sat, tau_sat_prev;

  /* Compute error */
  error = state_ref - state;

  /* Compute derivative of Kp term if not in first iteration */
  if (!first_it) {
    state_dot = (state - state_prev) / dt;

    /* Apply low pass filter due to noise amplification from derivative computation */
    lpf_A = std::exp(- lpf_pole_*dt);
    lpf_B = 1 - lpf_A;
    state_dot_filter = lpf_A*state_dot_filter_prev + lpf_B*state_dot;

  } else {
    /* Reset first iteration flag */
    first_it = false;
  }

  /* Add all PI terms */
  tau_d = ki_*error - kp_*state_dot_filter;

  /* Anti-windup */
  Ka = 1.0/dt;
  tau_dot = tau_d - Ka*(tau_prev - tau_sat_prev);
  tau = tau_prev + tau_dot*dt;
  tau_sat = std::clamp(tau, tau_min_, tau_max_);

  /* Set prev values */
  state_prev = state;
  state_dot_filter_prev = state_dot_filter;
  tau_prev = tau;
  tau_sat_prev = tau_sat;

  return tau_sat;
}

void ControllerPI::setGainKp(double kp) {kp_ = kp;}
void ControllerPI::setGainKi(double ki) {ki_ = ki;}
void ControllerPI::setLPFPole(double lpf_pole) {lpf_pole_ = lpf_pole;}

/************************/
/*    PID CONTROLLER    */
/************************/

/* Constructor */
ControllerPID::ControllerPID(double kp, double ki, double kd, double lpf_pole, double tau_min, double tau_max, bool wrapToPi) {
  /* Set parameters */
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  lpf_pole_ = lpf_pole;
  tau_min_ = tau_min;
  tau_max_ = tau_max;
  wrapToPi_ = wrapToPi;
}

/* Delta implementation for PID */
double ControllerPID::callController(double state, double state_ref, double state_rate, double dt) {
  static double error, tau_d;
  static bool first_it = true;
  static double state_rate_prev = 0.0, state_rate_dot = 0.0, state_rate_dot_filter = 0.0, state_rate_dot_filter_prev = 0.0;
  static double lpf_A = 0.0, lpf_B = 0.0;
  static double Ka, tau_dot, tau, tau_prev, tau_sat, tau_sat_prev;

  /* Compute error */
  error = state_ref - state;

  /* Wrap to [-pi, pi] if needed */
  if (wrapToPi_) {
    error = std::fmod(error + M_PI, 2*M_PI);
    if (error < 0) error += 2*M_PI;
    error = error - M_PI;
  }

  /* Compute derivative of Kd term if not in first iteration */
  if (!first_it) {
    state_rate_dot = (state_rate - state_rate_prev) / dt;

    /* Apply low pass filter due to noise amplification from derivative computation */
    lpf_A = std::exp(- lpf_pole_*dt);
    lpf_B = 1 - lpf_A;
    state_rate_dot_filter = lpf_A*state_rate_dot_filter_prev + lpf_B*state_rate_dot;

  } else {
    /* Reset first iteration flag */
    first_it = false;
  }

  /* Add all PID terms */
  tau_d = ki_*error - kp_*state_rate - kd_*state_rate_dot_filter;

  /* Anti-windup */
  Ka = 1.0/dt;
  tau_dot = tau_d - Ka*(tau_prev - tau_sat_prev);
  tau = tau_prev + tau_dot*dt;
  tau_sat = std::clamp(tau, tau_min_, tau_max_);

  /* Set prev values */
  state_rate_prev = state_rate;
  state_rate_dot_filter_prev = state_rate_dot_filter;
  tau_prev = tau;
  tau_sat_prev = tau_sat;

  return tau_sat;
}

void ControllerPID::setGainKp(double kp) {kp_ = kp;}
void ControllerPID::setGainKi(double ki) {ki_ = ki;}
void ControllerPID::setGainKd(double kd) {kd_ = kd;}
void ControllerPID::setLPFPole(double lpf_pole) {lpf_pole_ = lpf_pole;}

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
