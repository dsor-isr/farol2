#include <farol_utils/filters/second_order_lpf.hpp>
#include <farol_utils/angles.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <stdexcept>
#include <cstdio>
#include <iostream>

namespace farol_utils {

static inline bool unwrap(double &u, double x){
  if (u - x > M_PI){
    u -= 2.0 * M_PI;
    return true;
  }
  else if (u - x < -M_PI){
    u += 2.0 * M_PI;
    return true;
  }
  return false;
}

SecondOrderLowPass::SecondOrderLowPass() {
  reset();
}

void SecondOrderLowPass::configure(double wc, double Ts, std::string design, std::string method, bool wrap_angle) {
  if (wc <= 0.0)   throw std::invalid_argument("wc must be > 0");
  if (Ts <= 0.0)   throw std::invalid_argument("Ts must be > 0");
  wrap_angle_ = wrap_angle;
  wc_ = wc;
  Ts_ = Ts;

  // design choice
  if(design == "butterworth")
    design_butterworth();
  else if(design == "bessel")
    design_bessel();

  // discretization method
  if(method == "tustin")
    discretize_tustin();
  else if (method == "zoh")
    discretize_zoh();
  else
    discretize_euler();
  
  configured_ = true;
}

void SecondOrderLowPass::step(double u) {
  if (!configured_) {
    throw std::runtime_error("SecondOrderLowPass: call configure*() before step()");
  }

  bool did_wrap = false;
  if (wrap_angle_) 
    did_wrap = unwrap(u, x_(0));
  
  last_u_ = u;
  last_x_ = x_;
  x_ = Ad_ * x_ + Bd_ * u;

  if (did_wrap) 
    x_(0) = wrapTo2Pi(x_(0));
}


void SecondOrderLowPass::reset(double u_hat0, double du_hat0) {
  x_ << u_hat0, du_hat0;
  last_u_ = u_hat0;
}

double SecondOrderLowPass::y() const {
  return x_(0);
}

double SecondOrderLowPass::dy() const {
  return x_(1);
}

double SecondOrderLowPass::ddy() const {
  // computing again \dot x_2 = (A_ * last_x_ + B_ * last_u_)[1] DOES NOT WORK IN DISCRETE TIME !!!

  // This is the actual correct value based on the discretization
  return (x_[1] - last_x_[1])/Ts_;
}


void SecondOrderLowPass::discretize_euler(){
  Ad_ = Eigen::Matrix2d::Identity() + A_* Ts_;
  Bd_ = B_*Ts_;
}

void SecondOrderLowPass::discretize_zoh(){
  Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
  M.block<2,2>(0,0) = A_;
  M.block<2,1>(0,2) = B_;
  Eigen::Matrix3d Phi = (M * Ts_).exp();
  Ad_ = Phi.block<2,2>(0,0);
  Bd_ = Phi.block<2,1>(0,2);
}

void SecondOrderLowPass::discretize_tustin(){
  const Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
  const Eigen::Matrix2d M = (I - 0.5 * Ts_* A_);  // (I - Ts/2 A)

  // Solve M * X = RHS instead of explicit inverse (more stable)
  Ad_ = M.lu().solve(I + 0.5 * Ts_ * A_);
  Bd_ = M.lu().solve(Ts_ * B_);
}

void SecondOrderLowPass::design_butterworth(){
  A_ << 0.0, 1.0,
      -wc_*wc_, -2.0*0.707107*wc_;
  B_ << 0.0,
      wc_*wc_;
}

void SecondOrderLowPass::design_bessel(){
  A_ << 0.0, 1.0,
      -3*wc_*wc_, -3.0*wc_;
  B_ << 0.0,
       3*wc_*wc_;
}

} // namespace farol_utils
