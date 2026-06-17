#include "PathFollowingNode.h"

/**
 * @brief  A method for initializing all the services. This method is called by
 * the constructor of the PathNode class upon creation
 */
void PathFollowingNode::initialiseServices() {
  /* Advertise the services */
  this->pf_start_srv_ = create_service<farol2_path_following::srv::StartPF>(
                          SERVICE_START_PF,
                          std::bind(&PathFollowingNode::StartPFService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_stop_srv_ = create_service<farol2_path_following::srv::StopPF>(
                        SERVICE_STOP_PF,
                        std::bind(&PathFollowingNode::StopPFService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_update_gains_srv_ = create_service<farol2_path_following::srv::UpdateGainsPF>(
                                SERVICE_UPDATE_GAINS_PF,
                                std::bind(&PathFollowingNode::UpdateGainsPFService, this, std::placeholders::_1, std::placeholders::_2));

  
  this->pf_marcelo_srv_ = create_service<farol2_path_following::srv::SetPF>(
                            SERVICE_MARCELO_PF,
                            std::bind(&PathFollowingNode::SetMarceloService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_aguiar_srv_ = create_service<farol2_path_following::srv::SetPF>(
                          SERVICE_AGUIAR_PF,
                          std::bind(&PathFollowingNode::SetAguiarService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_breivik_srv_ = create_service<farol2_path_following::srv::SetPF>(
                            SERVICE_BREIVIK_PF,
                            std::bind(&PathFollowingNode::SetBreivikService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_fossen_srv_ = create_service<farol2_path_following::srv::SetPF>(
                          SERVICE_FOSSEN_PF,
                          std::bind(&PathFollowingNode::SetFossenService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_romulo_srv_ = create_service<farol2_path_following::srv::SetPF>(
                          SERVICE_ROMULO_PF,
                          std::bind(&PathFollowingNode::SetRomuloService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_lapierre_srv_ = create_service<farol2_path_following::srv::SetPF>(
                            SERVICE_LAPIERRE_PF,
                            std::bind(&PathFollowingNode::SetLapierreService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_pramod_srv_ = create_service<farol2_path_following::srv::SetPF>(
                          SERVICE_PRAMOD_PF,
                          std::bind(&PathFollowingNode::SetPramodService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_ravi_srv_ = create_service<farol2_path_following::srv::SetPF>(
                        SERVICE_RAVI_PF,
                        std::bind(&PathFollowingNode::SetRaviService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_samson_srv_ = create_service<farol2_path_following::srv::SetPF>(
                          SERVICE_SAMSON_PF,
                          std::bind(&PathFollowingNode::SetSamsonService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_relative_heading_srv_ = create_service<farol2_path_following::srv::SetPF>(
                                    SERVICE_RELATIVE_HEADING_PF,
                                    std::bind(&PathFollowingNode::SetRelativeHeadingService, this, std::placeholders::_1, std::placeholders::_2));

  this->pf_ilos_srv_ = create_service<farol2_path_following::srv::SetPF>(
                        SERVICE_ILOS_PF,
                        std::bind(&PathFollowingNode::SetIlosService, this, std::placeholders::_1, std::placeholders::_2));

  
  this->pf_reset_vt_srv_ = create_service<farol2_path_following::srv::ResetVT>(
                            SERVICE_RESET_VT_PF,
                            std::bind(&PathFollowingNode::ResetVirtualTargetService, this, std::placeholders::_1, std::placeholders::_2));
  
  /* Setup the waypoint client needed when mission finishes */
  this->wp_standard_client_ = create_client<farol2_waypoint::srv::SendWpType1>(
                                SERVICE_WP_STANDARD);
  
  /* Setup the reset DeadReckoning client needed when mission finishes */
  this->dr_reset_client_ = create_client<std_srvs::srv::Trigger>(
                            SERVICE_RESET_DR);
  
  /* Reset the path we are following and set the mode of operation */
  this->reset_path_client_ = create_client<farol2_planning::srv::ResetPath>(
                  SERVICE_RESET_PATH);

  this->set_path_mode_client_ = create_client<farol2_planning::srv::SetMode>(
                    SERVICE_SET_PATH_MODE);

}

/* Service to start running the path following algorithm that was chosen
 * previously */
void PathFollowingNode::StartPFService(const std::shared_ptr<farol2_path_following::srv::StartPF::Request> req,
                                       std::shared_ptr<farol2_path_following::srv::StartPF::Response> res) {
  (void)req;

  /* Check if we have a path following algorithm allocated. If so, start the
   * timer callbacks */
  if (this->pf_algorithm_ != nullptr) {

    /* Update the last time the iteration of the path following run */
    this->prev_time_ = this->now();
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
void PathFollowingNode::StopPFService(const std::shared_ptr<farol2_path_following::srv::StopPF::Request> req,
                                      std::shared_ptr<farol2_path_following::srv::StopPF::Response> res) {
  (void)req;

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
void PathFollowingNode::ResetVirtualTargetService(const std::shared_ptr<farol2_path_following::srv::ResetVT::Request> req,
                                                  std::shared_ptr<farol2_path_following::srv::ResetVT::Response> res) {

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
void PathFollowingNode::UpdateGainsPFService(const std::shared_ptr<farol2_path_following::srv::UpdateGainsPF::Request> req,
                                             std::shared_ptr<farol2_path_following::srv::UpdateGainsPF::Response> res) {

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
void PathFollowingNode::SetRelativeHeadingService(const std::shared_ptr<farol2_path_following::srv::SetPF::Request> req,
                                                  std::shared_ptr<farol2_path_following::srv::SetPF::Response> res) {
  (void)req;
  res->success = this->switchController("relative_heading");
}

/* Service to switch to the Marcelo Path Following method */
void PathFollowingNode::SetMarceloService(const std::shared_ptr<farol2_path_following::srv::SetPF::Request> req,
                                          std::shared_ptr<farol2_path_following::srv::SetPF::Response> res) {
  (void)req;
  res->success = this->switchController("marcelo");
}

/* Service to switch to the Aguiar Path Following method */
void PathFollowingNode::SetAguiarService(const std::shared_ptr<farol2_path_following::srv::SetPF::Request> req,
                                         std::shared_ptr<farol2_path_following::srv::SetPF::Response> res) {
  (void)req;
  res->success = this->switchController("aguiar");
}

/* Service to switch to the Breivik Path Following method */
void PathFollowingNode::SetBreivikService(const std::shared_ptr<farol2_path_following::srv::SetPF::Request> req,
                                          std::shared_ptr<farol2_path_following::srv::SetPF::Response> res) {
  (void)req;
  res->success = this->switchController("breivik");
}

/* Service to switch to the Fossen Path Following method */
void PathFollowingNode::SetFossenService(const std::shared_ptr<farol2_path_following::srv::SetPF::Request> req,
                                         std::shared_ptr<farol2_path_following::srv::SetPF::Response> res) {
  (void)req;
  res->success = this->switchController("fossen");
}

/* Service to switch to the Romulo Path Following method */
void PathFollowingNode::SetRomuloService(const std::shared_ptr<farol2_path_following::srv::SetPF::Request> req,
                                         std::shared_ptr<farol2_path_following::srv::SetPF::Response> res) {
  (void)req;
  res->success = this->switchController("romulo");
}

/* Service to switch to the Lapierre Path Following method */
void PathFollowingNode::SetLapierreService(const std::shared_ptr<farol2_path_following::srv::SetPF::Request> req,
                                           std::shared_ptr<farol2_path_following::srv::SetPF::Response> res) {
  (void)req;
  res->success = this->switchController("lapierre");
}

/* Service to switch to the Pramod Path Following method */
void PathFollowingNode::SetPramodService(const std::shared_ptr<farol2_path_following::srv::SetPF::Request> req,
                                         std::shared_ptr<farol2_path_following::srv::SetPF::Response> res) {
  (void)req;
  res->success = this->switchController("pramod");
}

/* Service to switch to the Samson Path Following method */
void PathFollowingNode::SetRaviService(const std::shared_ptr<farol2_path_following::srv::SetPF::Request> req,
                                       std::shared_ptr<farol2_path_following::srv::SetPF::Response> res) {
  (void)req;
  res->success = this->switchController("ravi");
}

/* Service to switch to the Samson Path Following method */
void PathFollowingNode::SetSamsonService(const std::shared_ptr<farol2_path_following::srv::SetPF::Request> req,
                                         std::shared_ptr<farol2_path_following::srv::SetPF::Response> res) {
  (void)req;
  res->success = this->switchController("samson");
}

/* Service to switch to the ILOS Path Following method */
void PathFollowingNode::SetIlosService(const std::shared_ptr<farol2_path_following::srv::SetPF::Request> req,
                                       std::shared_ptr<farol2_path_following::srv::SetPF::Response> res) {
  (void)req;
  res->success = this->switchController("ilos");
}

