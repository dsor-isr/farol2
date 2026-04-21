#ifndef REFERENCE_GENERATOR_HPP_
#define REFERENCE_GENERATOR_HPP_

#include <string>

#include <farol2_utils/filters/low_pass_filter.hpp>

namespace farol_control {

struct ReferenceGeneratorOutput {
  double ref_raw{0.0};
  double ref_limited{0.0};
  double ref_filt{0.0};
  double ref_used_for_control{0.0};
  double dref{0.0};
  double ddref{0.0};
};

class ReferenceGenerator {
 public:
  ReferenceGenerator() = default;

  void configure(bool wrap_to_pi,
                 bool use_rate_limiter,
                 double rate_limit,
                 bool use_ref_lpf,
                 bool use_filtered_ref_for_control,
                 double lpf_wc,
                 int lpf_order,
                 std::string lpf_design,
                 std::string lpf_method);

  ReferenceGeneratorOutput update(double ref_raw, double dt_ref);

  void reset(double ref_raw = 0.0);

  const ReferenceGeneratorOutput &output() const { return output_; }

 private:
  bool wrap_to_pi_{false};
  bool use_rate_limiter_{false};
  bool use_ref_lpf_{false};
  bool use_filtered_ref_for_control_{false};
  double rate_limit_{0.0};

  bool initialized_{false};
  double ref_rate_limited_{0.0};

  farol2_utils::LowPassFilter ref_lpf_;
  ReferenceGeneratorOutput output_;
};

}  // namespace farol_control

#endif  // REFERENCE_GENERATOR_HPP_
