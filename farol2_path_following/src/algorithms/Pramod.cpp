#include "Pramod.h"

Pramod::Pramod(std::vector<double> gains, 
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
bool Pramod::setPFGains(std::vector<double> gains) {
  // handle cases where tune will olly care kp and ki but from yaml there will be also es
  if(gains.size() == 2){
    this->kp_ = gains[0];
    this->ki_ = gains[1];
    return true;
  }
  if(gains.size() == 3){
    this->kp_ = gains[0];
    this->ki_ = gains[1];
    this->es_ = gains[2];
    return true;
  } 
  return false;
}

/* saturation as described in paper*/
double sat(double input, double es){
  constexpr double eps = 0.05;

  const double c1 = 1.0 / (4.0 * eps);
  const double c2 = 0.5 + (es / (2.0 * eps));
  const double c3 = (eps * eps - 2.0 * eps * es + es * es) / (4.0 * eps);

  if (std::abs(input) < (es - eps)) {
    return input;
  }
  if (input > (es + eps)) {
    return es;
  }
  if (input < (-es - eps)) {
    return -es;
  }
  if (input > (es - eps) && input <= (es + eps)) {
    return -c1 * input * input + c2 * input - c3;
  }
  // input in [-es - eps, -es + eps)
  return c1 * input * input + c2 * input + c3;
}

void Pramod::callPFController(double dt) {

  /* Get the path paramerters */
  Eigen::Vector2d path_pd;
  path_pd << this->path_state_.pd[0], this->path_state_.pd[1];
  double path_psi = this->path_state_.psi;
  double path_vd = this->path_state_.vd;
  double path_hg = this->path_state_.tangent_norm;

  /* Get the vehicle parameters */
  Eigen::Vector2d veh_p;
  veh_p << this->vehicle_state_.eta1[0], this->vehicle_state_.eta1[1];
  double veh_surge = 1.618;//this->vehicle_state_.v1[0];

  /* Compute the rotation matrix */
  Eigen::Matrix2d RI_F;
  RI_F << cos(path_psi), sin(path_psi), -sin(path_psi), cos(path_psi);

  /* Compute the position error */
  Eigen::Vector2d pos_error = RI_F * (veh_p - path_pd);
  double cross_track = pos_error[1];

  /* Notation used is the described in "A Path-Following Controller for Marine Vehicles Using a Two-Scale Inner-Outer Loop Approach" by Pramod Maurya (https://www.mdpi.com/1424-8220/22/11/4293)
  "cross_track" is the cross track error, represented by "e" in the paper
  "sigma" is the integral of the cross track error which has a dynamic of its own which is described id the paper
  "sat" is the saturation function described in the paper
  */
  
  /*Anti windup gain*/
  double Ka = veh_surge/this->kp_/dt; // = U/K_1 
  
  /*compute virtual control signal before antiwindup*/  
  double u = -this->kp_ / veh_surge * cross_track - this->ki_ / veh_surge * sigma_;
  
  /* sigma dynamics with anti-windup */ 
  double sigma_dot = cross_track + Ka * (u - sat(u, this->es_));
  
  /* integrate to obtain sigma */
  sigma_ += sigma_dot*dt;

  /* u = -K1/U*e - K2/U*sigma */
  u = -this->kp_ / veh_surge * cross_track - this->ki_ / veh_surge * sigma_;

  /* psi_d = path_psi + asin(sat(u)) */
  double desired_yaw_rad = path_psi + asin(sat(u, this->es_));

  this->desired_yaw_ = desired_yaw_rad;
  this->desired_surge_ = (path_vd + path_state_.vc) * path_hg;

  /* Path following values for debug */
  pfollowing_debug_.algorithm = "Pramod";
  pfollowing_debug_.cross_track_error = pos_error[1];
  pfollowing_debug_.along_track_error = pos_error[0];
  pfollowing_debug_.yaw = vehicle_state_.eta2[2];
  pfollowing_debug_.psi = path_state_.psi;
  pfollowing_debug_.gamma = path_state_.gamma;
}

/* Method to publish the control data */
void Pramod::publish_private() {
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
void Pramod::start() {
  /* Pramod does not used this start needs to use the closest point to the path
   * and NOT the default gamma <-> data communication, therefore we should
   * inform the path to be used in closest point mode */
  std::shared_ptr<farol2_planning::srv::SetMode::Request> req = std::make_shared<farol2_planning::srv::SetMode::Request>();
  req->closest_point_mode = true;
  this->mode_client_->async_send_request(req);
}

/* Method used to check whether we reached the end of the algorithm or not */
bool Pramod::stop() {

  /* If we have made all the path, then stop! */
  if(this->path_state_.gamma >= this->path_state_.gamma_max) {
    return true;
  }

  return false;
}

/* Method to reset all the algorithm data when the path following restarts */
bool Pramod::reset() {

  /* Reset the desired speed and yaw references */
  desired_surge_ = 0.0;
  desired_yaw_ = 0.0;
  sigma_ = 0.0;

  return true;
}

