#include "ILOS.h"

/* ------------------------------------------------------------------ */
/* Constructor                                                         */
/* ------------------------------------------------------------------ */
ILOS::ILOS(double delta,
           double ki,
           rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr surge_pub,
           rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub,
           rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gamma_pub,
           rclcpp::Client<farol2_planning::srv::SetMode>::SharedPtr mode_client) :
  surge_pub_(surge_pub),
  yaw_pub_(yaw_pub),
  gamma_pub_(gamma_pub),
  mode_client_(mode_client) {
  this->delta_ = delta;
  this->sigma_    = ki;
}

/* ------------------------------------------------------------------ */
/* setPFGains                                                          */
/* ------------------------------------------------------------------ */
bool ILOS::setPFGains(std::vector<double> gains) {
  if (gains.size() != 2) return false;

  this->delta_ = gains[0];
  this->sigma_    = gains[1];
  return true;
}

/* ------------------------------------------------------------------ */
/* callPFController                                                    */
/* ------------------------------------------------------------------ */
void ILOS::callPFController(double dt) {

  /* --- Extract path state ---*/
  Eigen::Vector2d path_pd;
  path_pd << this->path_state_.pd[0], this->path_state_.pd[1];
  double path_psi  = this->path_state_.psi;
  double path_vd   = this->path_state_.vd;
  double path_hg   = this->path_state_.tangent_norm;

  /* --- Extract vehicle state --- */
  Eigen::Vector2d veh_p;
  veh_p << this->vehicle_state_.eta1[0], this->vehicle_state_.eta1[1];

  /* --- Rotation matrix to path Frenet frame --- */
  Eigen::Matrix2d RI_F;
  RI_F <<  cos(path_psi), sin(path_psi),
          -sin(path_psi), cos(path_psi);

  /* --- Position error in Frenet frame --- */
  Eigen::Vector2d pos_error = RI_F * (veh_p - path_pd);
  double along_track = pos_error[0];
  double cross_track = pos_error[1];   // y_e in the paper

  /* -------------------------------------------------------------------
   * ILOS guidance law (Caharija et al., 2016) 
   *
   *   y_int_dot = y Δ / (Δ² + (y + σ*y_int)²)
   *   yaw_d     = psi_path + atan2(-(y + σ*y_int), Δ)
   * ----------------------------------------------------------------- */
  double y_int_dot = (cross_track * delta_) / (pow(cross_track + sigma_*y_int_, 2) + pow(delta_, 2)); 

  // only integrate if we are within the lookahead distance to prevent windup when far from the path
  if(cross_track < this->delta_/2){
		std::cout << "Integrating y_int with y_int_dot: " << y_int_dot << std::endl;
    y_int_ += y_int_dot * dt;
	}

  double desired_yaw_rad = path_psi + std::atan2(-(cross_track + sigma_ * y_int_), this->delta_);


  /* --- Surge reference --- */
  this->desired_surge_ = (path_vd + this->path_state_.vc) * path_hg;
  this->desired_yaw_   = desired_yaw_rad;

  /* --- Debug fields --- */
  pfollowing_debug_.algorithm        = "ILOS";
  pfollowing_debug_.cross_track_error = cross_track;
  pfollowing_debug_.along_track_error = along_track;
  pfollowing_debug_.yaw              = vehicle_state_.eta2[2];
  pfollowing_debug_.psi              = path_state_.psi;
  pfollowing_debug_.gamma            = path_state_.gamma;
  pfollowing_debug_.debug_values     = {y_int_, sigma_*y_int_, y_int_dot};
}

/* ------------------------------------------------------------------ */
/* publish_private                                                     */
/* ------------------------------------------------------------------ */
void ILOS::publish_private() {
  std_msgs::msg::Float32 msg;

  msg.data = this->desired_surge_;
  this->surge_pub_->publish(msg);

  /* Yaw reference in degrees, wrapped to [0, 360) */
  msg.data = farol2_utils::rad2deg(farol2_utils::wrapTo2Pi(this->desired_yaw_));
  this->yaw_pub_->publish(msg);

  /* Gamma for cooperative path following */
  msg.data = this->path_state_.gamma;
  this->gamma_pub_->publish(msg);
}

/* ------------------------------------------------------------------ */
/* start                                                               */
/* ------------------------------------------------------------------ */
void ILOS::start() {
  /* Request closest-point path mode — ILOS uses the nearest point on the
   * path rather than the virtual-target gamma integration */
  std::shared_ptr<farol2_planning::srv::SetMode::Request> req =
    std::make_shared<farol2_planning::srv::SetMode::Request>();
  req->closest_point_mode = true;
  this->mode_client_->async_send_request(req);
}

/* ------------------------------------------------------------------ */
/* stop                                                                */
/* ------------------------------------------------------------------ */
bool ILOS::stop() {
  if (this->path_state_.gamma >= this->path_state_.gamma_max) return true;
  return false;
}

/* ------------------------------------------------------------------ */
/* reset                                                               */
/* ------------------------------------------------------------------ */
bool ILOS::reset() {
  this->desired_surge_ = 0.0;
  this->desired_yaw_   = 0.0;
  this->sigma_         = 0.0;
  return true;
}
