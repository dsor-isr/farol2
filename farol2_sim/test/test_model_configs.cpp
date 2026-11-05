#include "farol2_utils/geometry.hpp"
#include "farol2_utils/angles.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <set>
#include "farol2_sim/actuators.hpp"
#include "farol2_sim/integrator.hpp"
#include "farol2_sim/kinematics.hpp"

using namespace farol2_sim;

// Exercise the shipped YAML with ROS's actual parameter parser, without
// creating a ROS participant or requiring DDS sockets.
class Config {
public:
  explicit Config(const std::string & vehicle)
  : data_(rcl_yaml_node_struct_init(rcutils_get_default_allocator()), rcl_yaml_node_struct_fini) {
    const std::string file = std::string(FAROL2_SOURCE_ROOT) + (vehicle == "example" ?
      "/farol2_sim/config/vehicle.yaml" : "/farol2_bringup/config/" + vehicle + "/default/sim.yaml");
    if (!data_ || !rcl_parse_yaml_file(file.c_str(), data_.get())) {
      throw std::runtime_error("Cannot parse " + file);
    }
    const std::string world = std::string(FAROL2_SOURCE_ROOT) +
      "/farol2_bringup/config/simple_world.yaml";
    if (!rcl_parse_yaml_file(world.c_str(), data_.get())) {
      throw std::runtime_error("Cannot parse " + world);
    }
    for (size_t i = 0; i < data_->num_nodes; ++i) {
      const std::string name(data_->node_names[i]);
      if (name.find("vehicle_sim") != std::string::npos) { node_ = name; }
    }
    if (node_.empty()) { throw std::runtime_error("Missing vehicle_sim config"); }
  }
  rcl_variant_t & get(const std::string & key) {
    auto * value = rcl_yaml_node_struct_get(node_.c_str(), key.c_str(), data_.get());
    if (!value) { throw std::runtime_error("Missing parameter " + key); }
    return *value;
  }
  double scalar(const std::string & key) {
    const auto & value = get(key);
    if (!value.double_value) { throw std::runtime_error("Expected double: " + key); }
    return *value.double_value;
  }
  Eigen::VectorXd vector(const std::string & key, int size) {
    const auto * values = get(key).double_array_value;
    if (!values || values->size != static_cast<size_t>(size)) {
      throw std::runtime_error("Expected double array: " + key);
    }
    return Eigen::Map<const Eigen::VectorXd>(values->values, size);
  }
private:
  std::unique_ptr<rcl_params_t, decltype(&rcl_yaml_node_struct_fini)> data_;
  std::string node_;
};

class ShippedModel : public ::testing::TestWithParam<const char *> {};

TEST_P(ShippedModel, ParametersParseAndDynamicsRemainFinite) {
  Config config(GetParam());
  DynamicsParameters p;
  p.mass = config.scalar("vehicle.mass");
  p.inertia = config.vector("vehicle.inertia_tensor", 3).asDiagonal();
  p.center_of_gravity = config.vector("vehicle.center_of_gravity", 3);
  p.center_of_buoyancy = config.vector("vehicle.center_of_buoyancy", 3);
  p.added_mass = config.vector("vehicle.added_mass_tensor", 6).asDiagonal();
  p.linear_damping = config.vector("vehicle.linear_damping_tensor", 6).asDiagonal();
  p.quadratic_damping = config.vector("vehicle.quadratic_damping_tensor", 6).asDiagonal();
  p.displaced_volume = config.scalar("vehicle.restoring.displaced_volume");
  p.buoyancy_sphere_radius = config.scalar("vehicle.restoring.sphere_radius");
  const auto * dofs = config.get("vehicle.active_dofs").integer_array_value;
  ASSERT_NE(dofs, nullptr);
  p.active_dofs.assign(dofs->values, dofs->values + dofs->size);
  Dynamics dynamics(p);
  Environment e;
  e.fluid_density = config.scalar("environment.fluid_density");
  e.gravity = config.scalar("environment.gravity");
  e.water_surface_z = config.scalar("environment.water_surface_z");
  const double current_speed = config.scalar("environment.current.speed");
  const double current_angle = farol2_utils::deg2rad(config.scalar("environment.current.angle"));
  ASSERT_GE(current_speed, 0.0);
  ASSERT_TRUE(std::isfinite(current_speed));
  ASSERT_TRUE(std::isfinite(current_angle));
  e.current_ned = Eigen::Vector3d(current_speed * std::cos(current_angle),
    current_speed * std::sin(current_angle), 0.0);
  EXPECT_DOUBLE_EQ(e.fluid_density, 1025.0);
  State state;
  state.position_ned = config.vector("initial_state.position", 3);
  state.rotation_body_to_ned = farol2_utils::rotationFromRpy(config.vector("initial_state.orientation", 3));
  state.velocity_through_water_body.head<3>() = config.vector("initial_state.body_velocity", 3);
  state.velocity_through_water_body.tail<3>() = config.vector("initial_state.orientation_rate", 3);
  const auto * frequency = config.get("node_frequency").integer_value;
  ASSERT_NE(frequency, nullptr);
  const double dt = 1.0 / *frequency;
  for (int tick = 0; tick < 20 * *frequency; ++tick) {
    integrate(state, dynamics, e, Vector6d::Zero(), dt);
  }
  EXPECT_TRUE(state.velocity_through_water_body.allFinite());
  EXPECT_TRUE(state.position_ned.allFinite());
  EXPECT_LT(state.velocity_through_water_body.norm(), 10.0);
}

INSTANTIATE_TEST_SUITE_P(Vehicles, ShippedModel,
  ::testing::Values("bluerov", "delfim", "magicelectric", "mblack", "mred", "myellow"));

class SensorConfig : public ::testing::TestWithParam<const char *> {};

TEST_P(SensorConfig, ListedSensorsHaveTheirOwnConfiguration) {
  Config config(GetParam());
  const auto * names = config.get("sensor_list").string_array_value;
  ASSERT_NE(names, nullptr);
  std::set<std::string> seen;
  for (size_t i = 0; i < names->size; ++i) {
    const std::string name = names->data[i];
    ASSERT_TRUE(seen.insert(name).second) << "Duplicate sensor: " << name;
    const std::string prefix = "sensor." + name + ".";
    if (name == "thruster_rpm") {
      const auto * frames = config.get("vehicle.actuators.frames").string_array_value;
      ASSERT_NE(frames, nullptr);
      EXPECT_GT(frames->size, 0u);
      continue;
    }
    if (name == "control_surface_angle") {
      const auto * enabled = config.get("vehicle.rudder.enabled").bool_value;
      ASSERT_NE(enabled, nullptr);
      EXPECT_TRUE(*enabled);
      continue;
    }
    const auto * sensor_frame = config.get(prefix + "frame").string_value;
    ASSERT_NE(sensor_frame, nullptr);
    EXPECT_FALSE(std::string(sensor_frame).empty());
    if (name == "altimeter") {
      const double period = config.scalar(prefix + "period");
      EXPECT_TRUE(std::isfinite(period));
      EXPECT_GT(period, 0.0);
      continue;
    }
    ASSERT_NE(config.get(prefix + "noise_enabled").bool_value, nullptr);
    std::vector<std::string> noise_names;
    if (name == "gnss") {
      noise_names = {"position_noise", "velocity_noise"};
      ASSERT_NE(config.get(prefix + "measure_velocity_over_ground").bool_value, nullptr);
      ASSERT_NE(config.get(prefix + "measure_utm_ned").bool_value, nullptr);
    } else if (name == "imu") {
      noise_names = {"orientation_noise", "angular_velocity_noise"};
    } else if (name == "dvl") {
      noise_names = {"velocity_noise"};
      const auto * mode = config.get(prefix + "mode").string_value;
      const auto * frame = config.get(prefix + "output_frame").string_value;
      ASSERT_NE(mode, nullptr);
      ASSERT_NE(frame, nullptr);
      EXPECT_TRUE(std::string(mode) == "water_track" || std::string(mode) == "bottom_track");
      EXPECT_TRUE(std::string(frame) == "sensor" || std::string(frame) == "body" ||
        std::string(frame) == "inertial");
    } else {
      ASSERT_EQ(name, "depth_cell");
      noise_names = {"position_noise"};
    }
    for (const auto & noise : noise_names) {
      EXPECT_TRUE(config.vector(prefix + noise + ".bias", 3).allFinite());
      const auto sigma = config.vector(prefix + noise + ".standard_deviation", 3);
      EXPECT_TRUE(sigma.allFinite());
      EXPECT_TRUE((sigma.array() >= 0.0).all());
    }
  }
}

INSTANTIATE_TEST_SUITE_P(VehiclesAndExample, SensorConfig,
  ::testing::Values("bluerov", "delfim", "magicelectric", "mblack", "mred", "myellow", "example"));
