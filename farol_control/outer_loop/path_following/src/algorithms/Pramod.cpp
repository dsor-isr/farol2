#include "Pramod.h"

Pramod::Pramod(std::vector<double> gains,
               rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surge_pub,
               rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub,
               rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gamma_pub,
               rclcpp::Client<paths::srv::SetMode>::SharedPtr mode_client) :
  surge_pub_(surge_pub),
  yaw_pub_(yaw_pub),
  gamma_pub_(gamma_pub),
  mode_client_(mode_client) {
  /* NOTE: no gain checkup is performed here */
  this->setPFGains(gains);
}

/* Method to setup the gains of the controller */
bool Pramod::setPFGains(std::vector<double> gains) {

  /* Handle the case where the number of gains received is not correct */
  if(gains.size() != 2) return false;

  this->gains_ = gains;
  return true;
}

/* saturation as described in paper*/
double sat(double input){
   if (input > 1)
    return 1;
  else if (input < -1)
    return -1;
  return input;
}


// double sat(double u, double es)
// {
//     return es * std::tanh(u / es);
// }0

// double sat(double x, double p)
// {
//     return x / std::pow(1.0 + std::pow(std::abs(x), p), 1.0 / p);
// }

double computeIntegralPreload(
    double e0,
    double Kp,
    double Ki,
    double xi,
    double omega_n,
    double dt,
    double alpha = 0.05)
{
    const double e0_abs = std::abs(e0);

    if (e0_abs < 1e-6 || omega_n < 1e-6) {
        return 0.0;
    }

    double dt_pred = std::min(dt, 1.0 / (50.0 * omega_n));
    dt_pred = std::clamp(dt_pred, 0.001, 0.05);

    double t_max = 3.0 / (xi * omega_n);  // approx 5% settling envelope
    t_max = std::clamp(t_max, 2.0, 60.0);

    double e = e0;
    double z = 0.0;
    double area = 0.0;

    double t = 0.0;
    while (t < t_max) {
        if (std::abs(e) <= alpha * e0_abs) {
            break;
        }

        // predicted unsaturated second-order dynamics:
        // e_dot = -Kp * e - Ki * z
        // z_dot = e
        double e_dot = -Kp * e - Ki * z;
        double z_dot = e;

        area += e * dt_pred;

        e += e_dot * dt_pred;
        z += z_dot * dt_pred;

        t += dt_pred;
    }

    return -area;
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
  finally, each important line of code has above it the cooresponent equation from the paper
  */
  
  /*Anti windup gain*/
  // double Ka = veh_surge/this->gains_[0]/dt; // = U/K_1 
  
  
  double e_max = 17.60;
  double xi = 1.0;
  double vcy_max = 0.5*veh_surge; 
  double w0 = veh_surge/e_max;
  double epsilon_current = 0.2; // confidance on current estimation 
  double min_corridor = 1.5; // minimum corridor for integral action
  // double e_min = 0;
  // double w_max = 0.1;
  // double abs_cross_track = std::abs(cross_track);
  // double w0;
  // if (abs_cross_track >= e_max) {
  //   w0 = w_min;
  // } else if (abs_cross_track <= e_min) {
  //   w0 = w_max;
  // } else {
  //   double e_n = (abs_cross_track - e_max)/(e_min - e_max);
  //   // double slope = (w_min - w_max ) / (e_max - e_min);
  //   // w0 = w_max + slope * (abs_cross_track - e_min);
  //   w0 = w_min + (1-cos(M_PI/2*e_n)) * (w_max - w_min);
  // }
  double vc_x_I = vehicle_state_.vc_inertial(0);
  double vc_y_I = vehicle_state_.vc_inertial(1);
  double vc_y_P = -sin(path_psi)*vc_x_I + cos(path_psi)*vc_y_I;

  // no integral outside of corridor 
  this->gains_[0] = w0;
  this->gains_[1] = 0.0;
  double integral_corridor = std::max(min_corridor, vc_y_P*epsilon_current/this->gains_[0]);
  if (abs(cross_track) < integral_corridor) {
    this->gains_[0] = 2.0*xi*w0;
    this->gains_[1] = std::pow(w0, 2);
  }
  
  
  double sigma_max = veh_surge/this->gains_[1] *sin( acos(vc_y_P*epsilon_current/veh_surge) );
  
  
  double u = -this->gains_[0] / veh_surge * cross_track - this->gains_[1] / veh_surge * sigma_;
  
  /* sigma dynamics with anti-windup */ 
  // double u_aw = Ka * (u - sat(u, 2));
  // double e_i = 3.0;
  double sigma_dot = cross_track;//(e_i*e_i)/(e_i*e_i + cross_track*cross_track) *cross_track;// + Ka * (u - sat(u, 0.9));
  
  /* integrate to obtain sigma */
  if(abs(cross_track) < integral_corridor && abs(u) < 1){
    sigma_ += sigma_dot*dt;
    sigma_ = std::clamp(sigma_, -sigma_max, sigma_max);
  }

  /* u = -K1/U*e - K2/U*sat */
  double p_term = -this->gains_[0] / veh_surge * cross_track;
  double i_term = -this->gains_[1] / veh_surge * sigma_;
  u = -this->gains_[0] / veh_surge * cross_track - this->gains_[1] / veh_surge * sigma_ - vc_y_P/veh_surge;
  // double yaw_correction = -this->gains_[0] / veh_surge * cross_track - this->gains_[1] / veh_surge * sigma_;

  /* psi_d = path_psi + asin(sat(u)) */
  double desired_yaw_rad = path_psi + asin(sat(u));

  this->desired_yaw_ = desired_yaw_rad;
  this->desired_surge_ = (path_vd + path_state_.vc) * path_hg;

  /* Path following values for debug */
  pfollowing_debug_.algorithm = "Pramod";
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
    this->gains_[0],
    this->gains_[1]
  };
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
  std::shared_ptr<paths::srv::SetMode::Request> req = std::make_shared<paths::srv::SetMode::Request>();
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

