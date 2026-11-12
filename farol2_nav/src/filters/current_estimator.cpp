#include <farol2_nav/filters/current_estimator.hpp>

#include <farol2_utils/angles.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace farol2_nav
{
namespace filters
{

void CurrentEstimatorFilter::configure(rclcpp::Node & node)
{
  tune_ekf_srv_ = node.create_service<farol2_nav::srv::TuneCurrentEstimator>(
    "current_estimator/tune",
    [this](
      const std::shared_ptr<farol2_nav::srv::TuneCurrentEstimator::Request> req,
      std::shared_ptr<farol2_nav::srv::TuneCurrentEstimator::Response> res)
    {
      this->on_tune_ekf(req, res);
    });

  debug_pub_ = node.create_publisher<farol2_nav::msg::CurrentEstimatorDebug>("current_estimator/debug", 10);

  q_pos_ = node.declare_parameter<double>("plugins.current_estimator.process_noise_pos", 0.1);
  q_current_ = node.declare_parameter<double>("plugins.current_estimator.process_noise_current", 0.01);
  r_pos_ = node.declare_parameter<double>("plugins.current_estimator.measurement_noise_pos", 1.0);
  p0_pos_ = node.declare_parameter<double>("plugins.current_estimator.init_cov_pos", 25.0);
  p0_current_ = node.declare_parameter<double>("plugins.current_estimator.init_cov_current", 1.0);
  init_current_speed_ = node.declare_parameter<double>("plugins.current_estimator.init_current_speed", 0.0);
  init_current_direction_ = node.declare_parameter<double>("plugins.current_estimator.init_current_direction", 0.0);

  H_.setZero();
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;

  Q_.setZero();
  Q_.diagonal() << q_pos_, q_pos_, q_current_, q_current_;

  R_.setIdentity();
  R_ *= std::max(1e-9, r_pos_);
}

void CurrentEstimatorFilter::on_tune_ekf(
  const std::shared_ptr<farol2_nav::srv::TuneCurrentEstimator::Request> req,
  std::shared_ptr<farol2_nav::srv::TuneCurrentEstimator::Response> res)
{
  if (req->process_noise_pos <= 0.0 ||
    req->process_noise_current <= 0.0 ||
    req->measurement_noise_pos <= 0.0)
  {
    res->success = false;
    res->message =
      "All EKF noise parameters must be > 0 (process_noise_pos, process_noise_current, measurement_noise_pos).";
    return;
  }

  q_pos_ = req->process_noise_pos;
  q_current_ = req->process_noise_current;
  r_pos_ = req->measurement_noise_pos;

  Q_.setZero();
  Q_.diagonal() << q_pos_, q_pos_, q_current_, q_current_;
  R_.setIdentity();
  R_ *= std::max(1e-9, r_pos_);

  res->success = true;
  res->message = "Position-current EKF noise parameters updated.";
}

void CurrentEstimatorFilter::compute(double dt_s, const MeasurementSnapshot & m, State & s)
{
  // Measurement is taken from the pass-through state (already populated from GNSS/UTM inputs).
  // This keeps one canonical measurement path in the pipeline instead of each filter
  // reading raw topics independently.
  const bool has_position_measurement = (m.gnss != nullptr) || (m.utm_ned != nullptr);
  // Get pseudo measurments from the state 
  const double measured_x_m = s.northing;
  const double measured_y_m = s.easting;
  const double u_model = s.velocity_through_water_body(0);
  const double v_model = s.velocity_through_water_body(1);
  const double psi = std::atan2(s.rotation_bn(1, 0), s.rotation_bn(0, 0));
  const double vx_model = u_model * std::cos(psi) - v_model * std::sin(psi);
  const double vy_model = u_model * std::sin(psi) + v_model * std::cos(psi);
  const double dt = std::max(0.0, dt_s);


  // initialize ekf whith the first measurement
  if (!initialized_) {
    if (!has_position_measurement) {
      return;
    }

    x_.setZero();
    x_(0) = measured_x_m;
    x_(1) = measured_y_m;
    x_(2) = init_current_speed_ * std::cos(farol2_utils::deg2rad(init_current_direction_));
    x_(3) = init_current_speed_ * std::sin(farol2_utils::deg2rad(init_current_direction_));

    P_.setZero();
    P_(0, 0) = p0_pos_;
    P_(1, 1) = p0_pos_;
    P_(2, 2) = p0_current_;
    P_(3, 3) = p0_current_;

    initialized_ = true;
  }
  
  // ---------------------------
  // EKF prediction step (model)
  // ---------------------------
  // State definition:
  // x = [x_pos, y_pos, current_x, current_y]^T
  //
  // Kinematics model:
  //   x_dot = v_x_model + current_x
  //   y_dot = v_y_model + current_y
  // where (v_x_model, v_y_model) comes from RPM-based surge estimate projected with yaw.
  //
  // Covariance model:
  //   P_k|k-1 = F P_k-1|k-1 F^T + Q_d
  // ---------------------------
  x_(0) += dt * (vx_model + x_(2));
  x_(1) += dt * (vy_model + x_(3));

  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;

  const Eigen::Matrix4d Qd = Q_ * dt;
  P_ = F * P_ * F.transpose() + Qd;

  // -------------------------------
  // EKF correction step (measurement)
  // -------------------------------
  // We only update when a fresh position measurement exists this cycle.
  // Measurement model:
  //   z = H x + v,  with H selecting [x_pos, y_pos]
  if (has_position_measurement) {
    Eigen::Vector2d z;
    z << measured_x_m, measured_y_m;

    y_ = z - H_ * x_;
    S_ = H_ * P_ * H_.transpose() + R_;
    K_ = P_ * H_.transpose() * S_.inverse();

    x_ += K_ * y_;
    P_ = (Eigen::Matrix4d::Identity() - K_ * H_) * P_;
  }

  // Write estimator output back to the shared pipeline state.
  s.current_velocity_ned(0) = x_(2);
  s.current_velocity_ned(1) = x_(3);
  s.current_velocity_ned(2) = 0.0;

  // publish debug message
  debug_msg_.northing_est = x_(0);
  debug_msg_.easting_est = x_(1);
  debug_msg_.innovation_northing = y_(0);
  debug_msg_.innovation_easting = y_(1);
  debug_msg_.gain_correction = psi;
  debug_pub_->publish(debug_msg_);

}

}  // namespace filters
}  // namespace farol2_nav
