#include "farol2_sim/sensors.hpp"
#include "farol2_utils/parameter_utils.hpp"
#include "farol2_utils/validation.hpp"

namespace farol2_sim {

SensorNoise::SensorNoise(rclcpp::Node & node, const std::string & sensor, std::uint32_t seed)
: prefix_("sensor." + sensor),
  enabled_(node.declare_parameter<bool>(prefix_ + ".noise_enabled", false)), random_(seed) {}

NoiseParameters SensorNoise::load(rclcpp::Node & node, const std::string & name) {
  const std::string prefix = prefix_ + "." + name;
  NoiseParameters parameters{
    farol2_utils::declareVector3Parameter(node, prefix + ".bias"),
    farol2_utils::declareVector3Parameter(node, prefix + ".standard_deviation")};
  for (int i = 0; i < 3; ++i) {
    farol2_utils::requireNonnegative(parameters.standard_deviation[i], prefix + ".standard_deviation");
  }
  return parameters;
}

double SensorNoise::sample(const NoiseParameters & parameters, int axis) {
  return enabled_ ? parameters.bias[axis] + parameters.standard_deviation[axis] * normal_(random_) : 0.0;
}

}  // namespace farol2_sim
