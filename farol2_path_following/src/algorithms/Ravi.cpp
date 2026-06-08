#include "Ravi.hpp"

#include <limits>

Ravi::Ravi(std::vector<double> gains,
           rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surge_pub,
           rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub,
           rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gamma_pub,
           rclcpp::Client<farol2_planning::srv::SetMode>::SharedPtr mode_client) :
  surge_pub_(surge_pub),
  yaw_pub_(yaw_pub),
  gamma_pub_(gamma_pub),
  mode_client_(mode_client) {
  /* NOTE: no gain checkup is performed here */
  this->setPFGains(gains);
}

/* Method to setup the gains of the controller */
bool Ravi::setPFGains(std::vector<double> gains) {

  /* Handle the case where the number of gains received is not correct */
  if(gains.size() != 4) return false;

  this->e_turn_ = gains[0];
  this->xi_ = gains[1];
  this->epsilon_current_ = gains[2];
  this->w0_min_ = gains[3];
  return true;
}

/* saturation as described in paper*/
static double sat(double input){
   if (input > 1)
    return 1;
  else if (input < -1)
    return -1;
  return input;
}

void Ravi::callPFController(double dt) {

  /* Get the path paramerters */
  Eigen::Vector2d path_pd;
  path_pd << this->path_state_.pd[0], this->path_state_.pd[1];
  double path_psi = this->path_state_.psi;
  double path_vd = this->path_state_.vd;
  double path_hg = this->path_state_.tangent_norm;

  /* Get the vehicle parameters */
  Eigen::Vector2d veh_p;
  veh_p << this->vehicle_state_.eta1[0], this->vehicle_state_.eta1[1];
  double veh_surge = 1.72;//this->vehicle_state_.v1[0];

  /* Compute the rotation matrix */
  Eigen::Matrix2d RI_F;
  RI_F << cos(path_psi), sin(path_psi), -sin(path_psi), cos(path_psi);

  /* Compute the position error */
  Eigen::Vector2d pos_error = RI_F * (veh_p - path_pd);
  double cross_track = pos_error[1];

  double w0 = veh_surge/e_turn_;
  double min_corridor = 4; // minimum corridor for integral action

  // compute current velocity orthogonal to the path
  double vc_x_I = vehicle_state_.vc_inertial(0);
  double vc_y_I = vehicle_state_.vc_inertial(1);
  double vc_y_P = -sin(path_psi)*vc_x_I + cos(path_psi)*vc_y_I;

  // no integral outside of corridor
  double kp = w0;
  double ki = 0.0;
  // compute integral corridor based on uncertainty of current estimate
  double integral_corridor = std::max(min_corridor, vc_y_P*epsilon_current_/kp);
  if (abs(cross_track) < integral_corridor) { 
    w0 = w0_min_;
    kp = 2.0*xi_*w0;
    ki = std::pow(w0, 2);
  }

  // hard cap on integral term based on maximum expected current disturbance
  double sigma_max = std::numeric_limits<double>::infinity();
  if (ki > 1e-12) {
    sigma_max = veh_surge / ki * std::sin(std::acos(vc_y_P * epsilon_current_ / veh_surge));
  }

//   std::cout << "integral_corridor: " << integral_corridor << std::endl;
//   std::cout << "w0: " << w0 << std::endl;
//   std::cout << "xi: " << xi_ << std::endl;
//   std::cout << "e_turn: " << e_turn_ << std::endl;
//   std::cout << "vc_y_P: " << vc_y_P << std::endl;
//   std::cout << "sigma_max: " << sigma_max << std::endl;
  
  // regular pramod control law
  double u = -kp / veh_surge * cross_track - ki / veh_surge * sigma_;

  double sigma_dot = cross_track;//(e_i*e_i)/(e_i*e_i + cross_track*cross_track) *cross_track;// + Ka * (u - sat(u, 0.9));

  /* integrate to obtain sigma */
  if(abs(cross_track) < integral_corridor && abs(u) < 1){
    sigma_ += sigma_dot*dt;
    sigma_ = std::clamp(sigma_, -sigma_max, sigma_max);
  }

  // u = -K1/U*e - K2/U*sigma + current_ff
  double p_term = -kp / veh_surge * cross_track;
  double i_term = -ki / veh_surge * sigma_;
  u = -kp / veh_surge * cross_track - ki / veh_surge * sigma_ - vc_y_P/veh_surge;

  /* psi_d = path_psi + asin(sat(u)) */
  double desired_yaw_rad = path_psi + asin(sat(u));

  this->desired_yaw_ = desired_yaw_rad;
  this->desired_surge_ = (path_vd + path_state_.vc) * path_hg;

  /* Path following values for debug */
  pfollowing_debug_.algorithm = "Ravi";
  pfollowing_debug_.cross_track_error = pos_error[1];
  pfollowing_debug_.along_track_error = pos_error[0];
  pfollowing_debug_.yaw = vehicle_state_.eta2[2];
  pfollowing_debug_.psi = path_state_.psi;
  pfollowing_debug_.gamma = path_state_.gamma;
  pfollowing_debug_.debug_values = {
    u,
    sigma_,
    integral_corridor,
    - vc_y_P/veh_surge,
    kp,
    ki
  };
}

/* Method to publish the control data */
void Ravi::publish_private() {
  std_msgs::msg::Float32 msg;

  /* Publish the control references */
  msg.data = this->desired_surge_;
  this->surge_pub_->publish(msg);

  // desired yaw in degrees
  msg.data = farol2_utils::rad2deg(farol2_utils::wrapTo2Pi(this->desired_yaw_));
  this->yaw_pub_->publish(msg);

  // Publish path gamma
  msg.data = this->path_state_.gamma;
  this->gamma_pub_->publish(msg);
}

/* Method that will run in the first iteration of the algorithm */
void Ravi::start() {
  /* Ravi does not used this start needs to use the closest point to the path
   * and NOT the default gamma <-> data communication, therefore we should
   * inform the path to be used in closest point mode */
  std::shared_ptr<farol2_planning::srv::SetMode::Request> req = std::make_shared<farol2_planning::srv::SetMode::Request>();
  req->closest_point_mode = true;
  this->mode_client_->async_send_request(req);
}

/* Method used to check whether we reached the end of the algorithm or not */
bool Ravi::stop() {

  /* If we have made all the path, then stop! */
  if(this->path_state_.gamma >= this->path_state_.gamma_max) {
    return true;
  }

  return false;
}

/* Method to reset all the algorithm data when the path following restarts */
bool Ravi::reset() {

  /* Reset the desired speed and yaw references */
  desired_surge_ = 0.0;
  desired_yaw_ = 0.0;
  sigma_ = 0.0;

  return true;
}
