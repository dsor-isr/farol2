#include <farol_utils/filters/low_pass_filter.hpp>
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

LowPassFilter::LowPassFilter() {
  reset();
}

void LowPassFilter::configure(double wc, double Ts, int order, std::string design, std::string method, bool wrap_angle) {
  if (wc <= 0.0)   throw std::invalid_argument("wc must be > 0");
  if (Ts <= 0.0)   throw std::invalid_argument("Ts must be > 0");
  if (order < 2)   throw std::invalid_argument("order must be >= 2");

  wrap_angle_ = wrap_angle;
  wc_ = wc;
  Ts_ = Ts;
  n_ = order;
  method_ = method;

  // design choice
  if(design == "butterworth")
    design_butterworth();
  else if(design == "bessel")
    design_bessel();
  else 
    design_bessel();
  
  configured_ = true;
  // std::cout << "\nA_:\n" << A_ << "\nB_:\n" << B_ << "\nAd_:\n" << Ad_ << "\nBd_:\n" << Bd_ << "\nx_:\n" << x_ << std::endl;
}

void LowPassFilter::step(double u, double dt) {
  if (!configured_) {
    throw std::runtime_error("LowPassFilter: call configure*() before step()");
  }

  bool did_wrap = false;
  if (wrap_angle_) 
    did_wrap = unwrap(u, x_(0));

  last_u_ = u;
  last_x_ = x_;

  if(method_ == "tustin")
    discretize_tustin(dt);
  else if (method_ == "zoh")
    discretize_zoh(dt);
  else
    discretize_euler(dt);

  // computation step
  x_ = Ad_ * x_ + Bd_ * u;

  if (did_wrap) 
    x_(0) = wrapTo2Pi(x_(0));
}


void LowPassFilter::reset(double u_hat0, double du_hat0)
{
  // If not configured yet, just remember the last input and bail
  if (n_ <= 0) {
    last_u_ = u_hat0;
    return;
  }

  if (x_.size() != n_) {
    x_.setZero(n_);
    last_x_.setZero(n_);
  } else {
    x_.setZero();
    last_x_.setZero();
  }

  // initialize y and dy; higher derivatives start at 0
  x_(0) = u_hat0;
  if (n_ >= 2) x_(1) = du_hat0;

  last_u_ = u_hat0;
}

double LowPassFilter::y() const {
  return x_(0);
}

double LowPassFilter::dy() const {
  return x_(1);
}

double LowPassFilter::ddy() const {
  if (n_ < 3) {
    // fallback finite difference if only 2nd order
    return (x_(1) - last_x_(1)) / Ts_;
  }
  return x_(2);
}

void LowPassFilter::design_butterworth()
{
  // Reallocate state-space matrices/vectors to the requested order
  A_.setZero(n_, n_);
  Ad_.setZero(n_, n_);
  B_.setZero(n_);
  Bd_.setZero(n_);
  x_.setZero(n_);
  last_x_.setZero(n_);
  
  using cd = std::complex<double>;

  // Denominator poly D(s) = Π (s - p_k), stored ascending powers: c[0] + c[1] s + ... + c[n] s^n
  std::vector<cd> c(1, cd(1.0, 0.0));

  for (int k = 0; k < n_; ++k) {
    const double th = M_PI/2.0 + (2.0*k + 1.0) * M_PI / (2.0*n_);
    const cd p = wc_ * std::polar(1.0, th);          // pole

    std::vector<cd> nxt(c.size() + 1, cd(0.0, 0.0));
    nxt[0] = -p * c[0];
    for (size_t i = 1; i < c.size(); ++i) nxt[i] = -p * c[i] + c[i - 1];
    nxt[c.size()] = c.back();
    c.swap(nxt);
  }

  const double b0 = c[0].real();                     // unity DC gain => numerator = a0

  // Companion form (derivative chain): x0=y, x1=dy, ..., x_{n-1}=y^(n-1)
  A_.topRightCorner(n_ - 1, n_ - 1).setIdentity();     // superdiagonal = 1
  for (int i = 0; i < n_; ++i) A_(n_ - 1, i) = -c[i].real();
  B_(n_ - 1) = b0;
}


void LowPassFilter::design_bessel()
{
  // Reallocate state-space matrices/vectors to the requested order
  A_.setZero(n_, n_);
  Ad_.setZero(n_, n_);
  B_.setZero(n_);
  Bd_.setZero(n_);
  x_.setZero(n_);
  last_x_.setZero(n_);

  auto fact = [](int m) -> double { return std::tgamma(m + 1.0); }; // m! for small m

  // Denominator poly D(s) = sum_{k=0}^n c[k] s^k, with c[n]=1 (monic)
  std::vector<double> c(n_+ 1, 0.0);
  for (int k = 0; k <= n_; ++k) {
    const double ak = fact(2*n_ - k) / (std::pow(2.0, n_ - k) * fact(n_ - k) * fact(k));
    c[k] = ak * std::pow(wc_, n_ - k); // scale by wc
  }

  const double b0 = c[0]; // unity DC gain => numerator = c0

  // Companion form (derivative chain): x0=y, x1=dy, ..., x_{n-1}=y^(n-1)
  A_.topRightCorner(n_ - 1, n_ - 1).setIdentity();      // superdiagonal = 1
  for (int i = 0; i < n_; ++i) A_(n_ - 1, i) = -c[i];   // -a0..-a_{n-1}
  B_(n_ - 1) = b0;
}



void LowPassFilter::discretize_euler(double dt){
  Ad_ = Eigen::MatrixXd::Identity(n_,n_) + A_* dt;
  Bd_ = B_*dt;
}

void LowPassFilter::discretize_zoh(double dt)
{
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n_ + 1, n_ + 1);
  M.block(0, 0, n_, n_) = A_;
  M.block(0, n_, n_, 1) = B_;
  const Eigen::MatrixXd Phi = (M * dt).exp();
  Ad_ = Phi.block(0, 0, n_, n_);
  Bd_ = Phi.block(0, n_, n_, 1);
}

void LowPassFilter::discretize_tustin(double dt)
{
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_, n_);
  const Eigen::MatrixXd M = I - 0.5 * dt * A_;      // (I - Ts/2 A)
  const Eigen::MatrixXd N = I + 0.5 * dt * A_;      // (I + Ts/2 A)

  // Solve M * Ad = N  and  M * Bd = Ts*B  (more stable than inverse)
  Eigen::FullPivLU<Eigen::MatrixXd> lu(M);
  Ad_ = lu.solve(N);
  Bd_ = lu.solve(dt * B_);
}


} // namespace farol_utils
