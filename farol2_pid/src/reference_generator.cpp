#include "farol2_pid/reference_generator.hpp"

#include <algorithm>
#include <cmath>

#include <farol2_utils/angles.hpp>

namespace farol_control {

void ReferenceGenerator::configure(bool wrap_to_pi,
                                   bool use_rate_limiter,
                                   double rate_limit,
                                   bool use_ref_lpf,
                                   bool use_filtered_ref_for_control,
                                   double lpf_wc,
                                   int lpf_order,
                                   std::string lpf_design,
                                   std::string lpf_method) {
  wrap_to_pi_ = wrap_to_pi;
  use_rate_limiter_ = use_rate_limiter;
  use_ref_lpf_ = use_ref_lpf;
  use_filtered_ref_for_control_ = use_filtered_ref_for_control;

  rate_limit_ = std::max(0.0, rate_limit);
  if (wrap_to_pi_) {
    rate_limit_ = farol2_utils::deg2rad(rate_limit_);
  }

  const double lpf_wc_safe = (lpf_wc > 0.0) ? lpf_wc : 1.0;
  ref_lpf_.configure(lpf_wc_safe, 0.1, lpf_order, lpf_design, lpf_method, wrap_to_pi_);
  reset();
}

ReferenceGeneratorOutput ReferenceGenerator::update(double ref_raw, double dt_ref) {
  output_.ref_raw = ref_raw;

  if (!initialized_) {
    initialized_ = true;
    ref_rate_limited_ = ref_raw;
    ref_lpf_.reset(ref_raw);

    output_.ref_limited = ref_raw;
    output_.ref_filt = ref_raw;
    output_.ref_used_for_control = ref_raw;
    output_.dref = 0.0;
    output_.ddref = 0.0;
    return output_;
  }

  double limited_ref = ref_raw;
  if (use_rate_limiter_ && rate_limit_ > 0.0 && dt_ref > 0.0) {
    double delta_ref = ref_raw - ref_rate_limited_;
    if (wrap_to_pi_) {
      delta_ref = farol2_utils::wrapToPi(delta_ref);
    }

    const double max_step = rate_limit_ * dt_ref;
    delta_ref = std::clamp(delta_ref, -max_step, max_step);
    ref_rate_limited_ += delta_ref;

    if (wrap_to_pi_) {
      ref_rate_limited_ = farol2_utils::wrapToPi(ref_rate_limited_);
    }

    limited_ref = ref_rate_limited_;
  } else {
    ref_rate_limited_ = ref_raw;
    limited_ref = ref_raw;
  }

  output_.ref_limited = limited_ref;

  if (use_ref_lpf_) {
    if (dt_ref > 0.0) {
      ref_lpf_.step(limited_ref, dt_ref);
      output_.ref_filt = ref_lpf_.y();
      output_.dref = ref_lpf_.dy();
      output_.ddref = ref_lpf_.ddy();
    } else {
      // Invalid ref cadence: keep deterministic behavior and avoid derivative spikes.
      ref_lpf_.reset(limited_ref);
      output_.ref_filt = limited_ref;
      output_.dref = 0.0;
      output_.ddref = 0.0;
    }
  } else {
    output_.ref_filt = limited_ref;
    output_.dref = 0.0;
    output_.ddref = 0.0;
  }

  output_.ref_used_for_control = use_filtered_ref_for_control_ ? output_.ref_filt : output_.ref_limited;

  return output_;
}

void ReferenceGenerator::reset(double ref_raw) {
  initialized_ = false;
  ref_rate_limited_ = ref_raw;

  output_.ref_raw = ref_raw;
  output_.ref_limited = ref_raw;
  output_.ref_filt = ref_raw;
  output_.ref_used_for_control = ref_raw;
  output_.dref = 0.0;
  output_.ddref = 0.0;
}

}  // namespace farol_control
