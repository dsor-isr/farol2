#include "PathFollowingNode.h"

/**
 * @brief  A method for initializing all the services. This method is called by
 * the constructor of the PathNode class upon creation
 */
void PathFollowingNode::initialiseServices() {
  /* Advertise the services */
  this->pf_start_srv_ = create_service<path_following::srv::StartPF>(
                          get_parameter("control.outer_loop.path_following.topics.services.start_pf").as_string(),
                          std::bind(&PathFollowingNode::StartPFService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_stop_srv_ = create_service<path_following::srv::StopPF>(
                        get_parameter("control.outer_loop.path_following.topics.services.stop_pf").as_string(),
                        std::bind(&PathFollowingNode::StopPFService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_update_gains_srv_ = create_service<path_following::srv::UpdateGainsPF>(
                                get_parameter("control.outer_loop.path_following.topics.services.updates_gains_pf").as_string(),
                                std::bind(&PathFollowingNode::UpdateGainsPFService, this, std::placeholders::_1, std::placeholders::_2));

  
  this->pf_marcelo_srv_ = create_service<path_following::srv::SetPF>(
                            get_parameter("control.outer_loop.path_following.topics.services.marcelo_pf").as_string(),
                            std::bind(&PathFollowingNode::SetMarceloService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_aguiar_srv_ = create_service<path_following::srv::SetPF>(
                          get_parameter("control.outer_loop.path_following.topics.services.aguiar_pf").as_string(),
                          std::bind(&PathFollowingNode::SetAguiarService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_breivik_srv_ = create_service<path_following::srv::SetPF>(
                            get_parameter("control.outer_loop.path_following.topics.services.breivik_pf").as_string(),
                            std::bind(&PathFollowingNode::SetBreivikService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_fossen_srv_ = create_service<path_following::srv::SetPF>(
                          get_parameter("control.outer_loop.path_following.topics.services.fossen_pf").as_string(),
                          std::bind(&PathFollowingNode::SetFossenService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_romulo_srv_ = create_service<path_following::srv::SetPF>(
                          get_parameter("control.outer_loop.path_following.topics.services.romulo_pf").as_string(),
                          std::bind(&PathFollowingNode::SetRomuloService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_lapierre_srv_ = create_service<path_following::srv::SetPF>(
                            get_parameter("control.outer_loop.path_following.topics.services.lapierre_pf").as_string(),
                            std::bind(&PathFollowingNode::SetLapierreService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_pramod_srv_ = create_service<path_following::srv::SetPF>(
                          get_parameter("control.outer_loop.path_following.topics.services.pramod_pf").as_string(),
                          std::bind(&PathFollowingNode::SetPramodService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_samson_srv_ = create_service<path_following::srv::SetPF>(
                          get_parameter("control.outer_loop.path_following.topics.services.samson_pf").as_string(),
                          std::bind(&PathFollowingNode::SetSamsonService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_relative_heading_srv_ = create_service<path_following::srv::SetPF>(
                                    get_parameter("control.outer_loop.path_following.topics.services.relative_heading_pf").as_string(),
                                    std::bind(&PathFollowingNode::SetRelativeHeadingService, this, std::placeholders::_1, std::placeholders::_2));

  
  this->pf_reset_vt_srv_ = create_service<path_following::srv::ResetVT>(
                            get_parameter("control.outer_loop.path_following.topics.services.reset_vt_pf").as_string(),
                            std::bind(&PathFollowingNode::ResetVirtualTargetService, this, std::placeholders::_1, std::placeholders::_2));
  
  /* Setup the waypoint client needed when mission finishes */
  this->wp_standard_client_ = create_client<waypoint::srv::SendWpType1>(
                                get_parameter("control.outer_loop.path_following.topics.services.wp_standard").as_string());
  
  /* Setup the reset DeadReckoning client needed when mission finishes */
  this->dr_reset_client_ = create_client<std_srvs::srv::Trigger>(
                            get_parameter("control.outer_loop.path_following.topics.services.reset_dr").as_string());
  
  /* Reset the path we are following and set the mode of operation */
  this->reset_path_client_ = create_client<paths::srv::ResetPath>(
                              get_parameter("control.outer_loop.path_following.topics.services.reset_path").as_string());

  this->set_path_mode_client_ = create_client<paths::srv::SetMode>(
                                  get_parameter("control.outer_loop.path_following.topics.services.set_path_mode").as_string());

}

/* Service to start running the path following algorithm that was chosen
 * previously */
void PathFollowingNode::StartPFService(const std::shared_ptr<path_following::srv::StartPF::Request> req,
                                       std::shared_ptr<path_following::srv::StartPF::Response> res) {

  /* Check if we have a path following algorithm allocated. If so, start the
   * timer callbacks */
  if (this->pf_algorithm_ != nullptr) {

    /* Update the last time the iteration of the path following run */
    this->prev_time_ = clock_.now();
    this->timer_->reset();
    res->success = true;

    /* Publish the code that simbolizes that path following has started */
    std_msgs::msg::Int8 msg;
    msg.data = FLAG_PF;
    this->mission_status_pub_->publish(msg);

    /* Run the first iteration of the algorithm */
    this->pf_algorithm_->start();

    /* Inform the user the path following algorithm will start */
    RCLCPP_INFO(this->get_logger(), "Path Following is starting.");

    return;
  }

  /* If there is not object for path following allocated, then print message to
   * console */
  RCLCPP_WARN(this->get_logger(), "There is not a path following method allocated. Please restart the "
      "node or set the PF to use.");
  res->success = false;
  return;
}

/* Service to stop the path following algorithm that was running */
void PathFollowingNode::StopPFService(const std::shared_ptr<path_following::srv::StopPF::Request> req,
                                      std::shared_ptr<path_following::srv::StopPF::Response> res) {

  /* Stop the path following only if it was already running */
  if (!this->timer_->is_canceled()) {
    /* Publish the code that simbolizes idle mode */
    std_msgs::msg::Int8 msg;
    msg.data = FLAG_IDLE;
    this->mission_status_pub_->publish(msg);
  }

  /* Return success */
  res->success = true;

  return;
}

/* Service to reset the virtual target value */
void PathFollowingNode::ResetVirtualTargetService(const std::shared_ptr<path_following::srv::ResetVT::Request> req,
                                                  std::shared_ptr<path_following::srv::ResetVT::Response> res) {

    /* Check if we have a path following algorithm instantiated */
    if(this->pf_algorithm_ == nullptr) {
      res->success = false;
      return;
    }

    /* Reset the virtual target */
    res->success = this->pf_algorithm_->resetVirtualTarget((float) req->value);
    return;
}

/* Service to update the gains of the path following algorithms live */
void PathFollowingNode::UpdateGainsPFService(const std::shared_ptr<path_following::srv::UpdateGainsPF::Request> req,
                                             std::shared_ptr<path_following::srv::UpdateGainsPF::Response> res) {

  /* Get the new gains */
  std::vector<double> new_gains = req->gains;

  /* Pass the new gains for the controller */
  if (this->pf_algorithm_ != nullptr) {
    /* Try to update the gains */
    bool result = this->pf_algorithm_->setPFGains(new_gains);

    /* Inform the user if the new gains were accepted or not */
    if (result == true) {
      RCLCPP_INFO(this->get_logger(), "Gains updated successfully!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Gains not accepted!");
    }

    /* Update the response message */
    res->success = result;
    return;
  }

  /* If the path following algorithm object is not allocated, some error ocurred
   * and we need to restart this node */
  RCLCPP_WARN(this->get_logger(), "There is not path following method allocated. Please restart the "
      "node or set the PF to use.");
  res->success = false;

  return;
}

/* Service to switch to the RelativeHeading Path Following method */
void PathFollowingNode::SetRelativeHeadingService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                                                  std::shared_ptr<path_following::srv::SetPF::Response> res) {

    /* Don't change if the algorithm is running */
    if (!this->timer_->is_canceled()) {
        RCLCPP_INFO(this->get_logger(), "Can't change algorithm when PF is running.");
        res->success = false;
        return;
    }

    /* Clear the memory used by the previous controller */
    this->deleteCurrentController();

    /* Create the publishers for the node */
    this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                  get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));
    this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                  get_parameter("control.outer_loop.path_following.topics.publishers.sway").as_string(), 1));
    this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                  get_parameter("control.outer_loop.path_following.topics.publishers.yaw").as_string(), 1));
    this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                  get_parameter("control.outer_loop.path_following.topics.publishers.rabbit").as_string(), 1));

    /* Read the control gains from the parameter server */
    double kx, ky, kz, yaw_offset;
    std::vector<double> p_sat;

    try {

        /* Read the gains for the controller */
        kx = get_parameter("control.outer_loop.path_following.controller_gains.relative_heading.kx").as_double();
        ky = get_parameter("control.outer_loop.path_following.controller_gains.relative_heading.ky").as_double();
        kz = get_parameter("control.outer_loop.path_following.controller_gains.relative_heading.kz").as_double();
        yaw_offset = get_parameter("control.outer_loop.path_following.controller_gains.relative_heading.yaw_offset").as_double();
        p_sat = get_parameter("control.outer_loop.path_following.controller_gains.relative_heading.p_sat").as_double_array();

        /* Assign the new controller */
        this->pf_algorithm_ = new RelativeHeading(kx, ky, kz, Eigen::Vector2d(p_sat.data()), yaw_offset, this->publishers_[0], this->publishers_[1], this->publishers_[2], this->publishers_[3]);

        /* Path the debug variables publisher to the class*/
        pf_algorithm_->setPFollowingDebugPublisher(create_publisher<farol_msgs::msg::PFDebug>(
                                                    get_parameter("control.outer_loop.path_following.topics.publishers.pfollowing_debug").as_string(), 1));

        /* Return success */
        res->success = true;

    } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Some error occured. Please reset the PF node for safety");
        res->success = false;
        return;
    }

    /* Return success */
    RCLCPP_INFO(this->get_logger(), "PF controller switched to RelativeHeading");
    return;
}

/* Service to switch to the Marcelo Path Following method */
void PathFollowingNode::SetMarceloService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                                          std::shared_ptr<path_following::srv::SetPF::Response> res) {

  /* Don't change if the algorithm is running */
  if (!this->timer_->is_canceled()) {
    RCLCPP_INFO(this->get_logger(), "Can't change algorithm when PF is running.");
    res->success = false;
    return;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Create the publishers for the node */
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.yaw_rate").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.rabbit").as_string(), 1));

  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.observer.x").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.observer.y").as_string(), 1));

  double delta, kz;
  double kk[2];
  double k_pos;
  double k_currents;
  std::vector<double> rd;
  std::vector<double> d;

  try {

    /* Read the gains for the controller */
    delta = get_parameter("control.outer_loop.path_following.controller_gains.marcelo.delta").as_double();
    kk[0] = get_parameter("control.outer_loop.path_following.controller_gains.marcelo.kx").as_double();
    kk[1] = get_parameter("control.outer_loop.path_following.controller_gains.marcelo.ky").as_double();
    kz = get_parameter("control.outer_loop.path_following.controller_gains.marcelo.kz").as_double();
    k_pos = get_parameter("control.outer_loop.path_following.controller_gains.marcelo.k_pos").as_double();
    k_currents = get_parameter("control.outer_loop.path_following.controller_gains.marcelo.k_currents").as_double();
    rd = get_parameter("control.outer_loop.path_following.controller_gains.marcelo.rd").as_double_array();
    d = get_parameter("control.outer_loop.path_following.controller_gains.marcelo.d").as_double_array();

    /* Assign the new controller */
    this->pf_algorithm_ =
      new Marcelo(delta, kk, kz, k_pos, k_currents, rd.data(), d.data(), this->publishers_[0],
          this->publishers_[1], this->publishers_[2], this->publishers_[3], this->publishers_[4]);

    pf_algorithm_->setPFollowingDebugPublisher(create_publisher<farol_msgs::msg::PFDebug>(
                                                get_parameter("control.outer_loop.path_following.topics.publishers.pfollowing_debug").as_string(), 1));
    
    res->success = true;

  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Some error occured. Please reset the PF node for safety");
    res->success = false;
    return;
  }

  /* Return success */
  RCLCPP_INFO(this->get_logger(), "PF controller switched to Marcelo");
  return;
}

/* Service to switch to the Aguiar Path Following method */
void PathFollowingNode::SetAguiarService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                                         std::shared_ptr<path_following::srv::SetPF::Response> res) {

  /* Don't change if the algorithm is running */
  if (!this->timer_->is_canceled()) {
    RCLCPP_INFO(this->get_logger(), "Can't change algorithm when PF is running.");
    res->success = false;
    return;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Create the publishers for the node */
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.yaw_rate").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.rabbit").as_string(), 1));

  double delta, kz;
  double kk[2];
  double k_pos;
  double k_currents;

  try {

    /* Read the gains for the controller */
    delta = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.delta").as_double();
    kk[0] = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.kx").as_double();
    kk[1] = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.ky").as_double();
    kz = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.kz").as_double();
    k_pos = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.k_pos").as_double();
    k_currents = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.k_currents").as_double();

    /* Assign the new controller */
    this->pf_algorithm_ =
      new Aguiar(delta, kk, kz, k_pos, k_currents, this->publishers_[0],
          this->publishers_[1], this->publishers_[2]);

    pf_algorithm_->setPFollowingDebugPublisher(create_publisher<farol_msgs::msg::PFDebug>(
                                                get_parameter("control.outer_loop.path_following.topics.publishers.pfollowing_debug").as_string(), 1));
    
    res->success = true;

  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Some error occured. Please reset the PF node for safety");
    res->success = false;
    return;
  }

  /* Return success */
  RCLCPP_INFO(this->get_logger(), "PF controller switched to Aguiar");
  return;
}

/* Service to switch to the Breivik Path Following method */
void PathFollowingNode::SetBreivikService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                                          std::shared_ptr<path_following::srv::SetPF::Response> res) {

  /* Don't change if the algorithm is running */
  if (!this->timer_->is_canceled()) {
    RCLCPP_INFO(this->get_logger(), "Can't change algorithm when PF is running.");
    res->success = false;
    return;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Create the publishers for the node */
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.yaw").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.rabbit").as_string(), 1));
  
  /* Read the gains for the controller */
  double delta_h;
  delta_h = get_parameter("control.outer_loop.path_following.controller_gains.breivik.delta_h").as_double();
 
  try {

    /* Assign the new controller */
    this->pf_algorithm_ = new Breivik(this->publishers_[0], 
        this->publishers_[1], this->publishers_[2], delta_h);
    res->success = true;
    pf_algorithm_->setPFollowingDebugPublisher(create_publisher<farol_msgs::msg::PFDebug>(
                                                get_parameter("control.outer_loop.path_following.topics.publishers.pfollowing_debug").as_string(), 1));

  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Some error occured. Please reset the PF node for safety");
    res->success = false;
    return;
  }

  /* Return success */
  RCLCPP_INFO(this->get_logger(), "PF controller switched to Breivik");
  return;
}

/* Service to switch to the Fossen Path Following method */
void PathFollowingNode::SetFossenService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                                         std::shared_ptr<path_following::srv::SetPF::Response> res) {

  /* Don't change if the algorithm is running */
  if (!this->timer_->is_canceled()) {
    RCLCPP_INFO(this->get_logger(), "Can't change algorithm when PF is running.");
    res->success = false;
    return;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Create the publishers for the node */
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.yaw").as_string(), 1));
   
  try {

    /* Assign the new controller */
    this->pf_algorithm_ = new Fossen(this->publishers_[0], this->publishers_[1], this->set_path_mode_client_);
    res->success = true;
    pf_algorithm_->setPFollowingDebugPublisher(create_publisher<farol_msgs::msg::PFDebug>(
                                                get_parameter("control.outer_loop.path_following.topics.publishers.pfollowing_debug").as_string(), 1));

  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Some error occured. Please reset the PF node for safety");
    res->success = false;
    return;
  }

  /* Return success */
  RCLCPP_INFO(this->get_logger(), "PF controller switched to Fossen. This algorithm uses the path closest point to make the computations");
  return;
}

/* Service to switch to the Romulo Path Following method */
void PathFollowingNode::SetRomuloService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                                         std::shared_ptr<path_following::srv::SetPF::Response> res) {

  /* Don't change if the algorithm is running */
  if (!this->timer_->is_canceled()) {
    RCLCPP_INFO(this->get_logger(), "Can't change algorithm when PF is running.");
    res->success = false;
    return;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Create the publishers for the node */
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.sway").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.rabbit").as_string(), 1));

  /* Variables to store the gains of the controller */
  std::vector<double> controller_gains;
  double kz;

  try {

    /* Read the gains for the controller */
    controller_gains = get_parameter("control.outer_loop.path_following.controller_gains.romulo.ke").as_double_array();
    kz = get_parameter("control.outer_loop.path_following.controller_gains.romulo.kz").as_double();

    controller_gains.push_back(kz);

    /* Assign the new controller */
    this->pf_algorithm_ =
      new Romulo(controller_gains, this->publishers_[0],
          this->publishers_[1], this->publishers_[2]);
    res->success = true;
    pf_algorithm_->setPFollowingDebugPublisher(create_publisher<farol_msgs::msg::PFDebug>(
                                                get_parameter("control.outer_loop.path_following.topics.publishers.pfollowing_debug").as_string(), 1));

  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Some error occured. Please reset the PF node for safety");
    res->success = false;
    return;
  }

  /* Return success */
  RCLCPP_INFO(this->get_logger(), "PF controller switched to Romulo");
  return;
}

/* Service to switch to the Lapierre Path Following method */
void PathFollowingNode::SetLapierreService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                                           std::shared_ptr<path_following::srv::SetPF::Response> res) {

  /* Don't change if the algorithm is running */
  if (!this->timer_->is_canceled()) {
    RCLCPP_INFO(this->get_logger(), "Can't change algorithm when PF is running.");
    res->success = false;
    return;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Create the publishers for the node */
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.yaw_rate").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.rabbit").as_string(), 1));

  /* Variables to store the gains of the controller */
  double k1, k2, k3, theta, k_delta;

  try {

    /* Read the gains for the controller */
    k1 = get_parameter("control.outer_loop.path_following.controller_gains.lapierre.k1").as_double();
    k2 = get_parameter("control.outer_loop.path_following.controller_gains.lapierre.k2").as_double();
    k3 = get_parameter("control.outer_loop.path_following.controller_gains.lapierre.k3").as_double();
    theta = get_parameter("control.outer_loop.path_following.controller_gains.lapierre.theta").as_double();
    k_delta = get_parameter("control.outer_loop.path_following.controller_gains.lapierre.k_delta").as_double();

    /* Assign the new controller */
    this->pf_algorithm_ = new Lapierre(k1, k2, k3, theta, k_delta, this->publishers_[0], this->publishers_[1], this->publishers_[2]);
    pf_algorithm_->setPFollowingDebugPublisher(create_publisher<farol_msgs::msg::PFDebug>(
                                                get_parameter("control.outer_loop.path_following.topics.publishers.pfollowing_debug").as_string(), 1));
    res->success = true;

  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Some error occured. Please reset the PF node for safety");
    res->success = false;
    return;
  }

  /* Return success */
  RCLCPP_INFO(this->get_logger(), "PF controller switched to Lapierre");
  return;
}

/* Service to switch to the Pramod Path Following method */
void PathFollowingNode::SetPramodService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                                         std::shared_ptr<path_following::srv::SetPF::Response> res) {

  /* Don't change if the algorithm is running */
  if (!this->timer_->is_canceled()) {
    RCLCPP_INFO(this->get_logger(), "Can't change algorithm when PF is running.");
    res->success = false;
    return;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Create the publishers for the node */
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.yaw").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.rabbit").as_string(), 1));

  /* Variables to store the gains of the controller */
  double kp, ki;
  std::vector<double> controller_gains;

  try {

    /* Read the gains for the controller */
    kp = get_parameter("control.outer_loop.path_following.controller_gains.pramod.kp").as_double();
    ki = get_parameter("control.outer_loop.path_following.controller_gains.pramod.ki").as_double();

    controller_gains.push_back(kp);
    controller_gains.push_back(ki);

    /* Assign the new controller */
    this->pf_algorithm_ = new Pramod(controller_gains, this->publishers_[0],
        this->publishers_[1], this->publishers_[2], this->set_path_mode_client_);
    pf_algorithm_->setPFollowingDebugPublisher(create_publisher<farol_msgs::msg::PFDebug>(
                                                get_parameter("control.outer_loop.path_following.topics.publishers.pfollowing_debug").as_string(), 1));
    res->success = true;

  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Some error occured. Please reset the PF node for safety");
    res->success = false;
    return;
  }

  /* Return success */
  RCLCPP_INFO(this->get_logger(), "PF controller switched to Pramod. This algorithm uses the path closest point to make the computations");
  return;
}

/* Service to switch to the Samson Path Following method */
void PathFollowingNode::SetSamsonService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                                         std::shared_ptr<path_following::srv::SetPF::Response> res) {

  /* Don't change if the algorithm is running */
  if (!this->timer_->is_canceled()) {
    RCLCPP_INFO(this->get_logger(), "Can't change algorithm when PF is running.");
    res->success = false;
    return;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Create the publishers for the node */
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.yaw_rate").as_string(), 1));

  /* Variables to store the gains of the controller */
  double k1, k2, k3, theta, k_delta;

  try {

    /* Read the gains for the controller */
    k1 = get_parameter("control.outer_loop.path_following.controller_gains.samson.k1").as_double();
    k2 = get_parameter("control.outer_loop.path_following.controller_gains.samson.k2").as_double();
    k3 = get_parameter("control.outer_loop.path_following.controller_gains.samson.k3").as_double();
    theta = get_parameter("control.outer_loop.path_following.controller_gains.samson.theta").as_double();
    k_delta = get_parameter("control.outer_loop.path_following.controller_gains.samson.k_delta").as_double();

    /* Assign the new controller */
    this->pf_algorithm_ = new Samson(k1, k2, k3, theta, k_delta, this->publishers_[0], this->publishers_[1], this->set_path_mode_client_ );
    pf_algorithm_->setPFollowingDebugPublisher(create_publisher<farol_msgs::msg::PFDebug>(
                                                get_parameter("control.outer_loop.path_following.topics.publishers.pfollowing_debug").as_string(), 1));
    res->success = true;

  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Some error occured. Please reset the PF node for safety");
    res->success = false;
    return;
  }

  /* Return success */
  RCLCPP_INFO(this->get_logger(), "PF controller switched to Samson. This algorithm uses the path closest point to make the computations");
  return;
}

