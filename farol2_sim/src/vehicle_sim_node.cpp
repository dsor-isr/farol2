#include "farol2_utils/geometry.hpp"
#include "farol2_utils/validation.hpp"
#include "farol2_utils/angles.hpp"
#include "vehicle_sim_node.hpp"
#include "farol2_utils/parameter_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>

using namespace farol2_sim;
namespace {
constexpr double degrees = farol2_utils::rad2deg(1.0);
geometry_msgs::msg::Vector3 vectorMessage(const Eigen::Vector3d & value) {
  geometry_msgs::msg::Vector3 message;
  message.x = value.x(); message.y = value.y(); message.z = value.z();
  return message;
}
}  // namespace

VehicleSimNode::VehicleSimNode() : Node("vehicle_sim") {
  loadParameters();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_,
    get_node_base_interface(), get_node_logging_interface(), get_node_parameters_interface(),
    get_node_topics_interface());
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  initialize_subscribers();
  initialize_publishers();
  initialize_sensors();
  geometry_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { initializeGeometry(); });
  timer_ = create_timer(std::chrono::nanoseconds(static_cast<int64_t>(std::llround(node_period_ * 1e9))),
    [this]() { tick(); });
}

void VehicleSimNode::initialize_subscribers() {
  rpm_sub_ = create_subscription<farol2_interfaces::msg::ThrusterRPM>("rpm_command", 1,
    [this](farol2_interfaces::msg::ThrusterRPM::ConstSharedPtr message) {
      if (message->rpm.size() != rpm_commands_.size() ||
          !std::all_of(message->rpm.begin(), message->rpm.end(), [](double v) { return std::isfinite(v); })) {
        RCLCPP_WARN(get_logger(), "Ignoring invalid RPM command: expected %zu finite values", rpm_commands_.size());
        return;
      }
      rpm_commands_ = message->rpm;
    });
  if (rudder_) {
    if (rudder_rate_mode_) {
      rudder_rate_sub_ = create_subscription<std_msgs::msg::Float32>("rudder_cmd", 1,
        [this](std_msgs::msg::Float32::ConstSharedPtr message) {
          if (std::isfinite(message->data)) {
            rudder_rate_command_ = std::clamp(static_cast<double>(message->data), -1.0, 1.0);
          }
        });
    } else {
      rudder_sub_ = create_subscription<farol2_interfaces::msg::ControlSurfaceAngle>("control_surface_angle_ref", 1,
        [this](farol2_interfaces::msg::ControlSurfaceAngle::ConstSharedPtr message) {
          if (message->angle.size() == 1 && std::isfinite(message->angle[0])) {
            rudder_command_ = std::clamp(message->angle[0] / degrees, rudder_min_, rudder_max_);
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring invalid rudder command: expected one finite angle in degrees");
          }
        });
    }
  }
}

void VehicleSimNode::initialize_publishers() {
  if (rudder_) {
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  }
  position_pub_ = create_publisher<geometry_msgs::msg::Vector3>("position", 1);
  body_velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3>("body_velocity", 1);
  orientation_pub_ = create_publisher<geometry_msgs::msg::Vector3>("orientation", 1);
  orientation_rate_pub_ = create_publisher<geometry_msgs::msg::Vector3>("orientation_rate", 1);
  body_acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3>("body_acceleration", 1);
  angular_acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3>("angular_acceleration", 1);
}

void VehicleSimNode::initialize_sensors() {
  const auto names = declare_parameter<std::vector<std::string>>("sensor_list");
  farol2_utils::requireUnique(names, "sensor_list");
  for (const auto & name : names) {
    if (name != "gnss" && name != "imu" && name != "dvl" && name != "depth_cell" &&
        name != "altimeter" && name != "thruster_rpm" && name != "control_surface_angle") {
      throw std::invalid_argument("Unknown sensor in sensor_list: " + name +
        ". Supported sensors: gnss, imu, dvl, depth_cell, altimeter, thruster_rpm, control_surface_angle");
    }
    if (name == "control_surface_angle" && !rudder_) {
      throw std::invalid_argument("control_surface_angle sensor requires an enabled rudder");
    }
  }
  for (const auto & name : names) {
    if (name == "gnss") { sensors_.push_back(std::make_unique<GnssSensor>(*this)); }
    else if (name == "imu") { sensors_.push_back(std::make_unique<ImuSensor>(*this)); }
    else if (name == "dvl") { sensors_.push_back(std::make_unique<DvlSensor>(*this)); }
    else if (name == "depth_cell") { sensors_.push_back(std::make_unique<DepthCellSensor>(*this)); }
    else if (name == "altimeter") { sensors_.push_back(std::make_unique<AltimeterSensor>(*this)); }
    else if (name == "thruster_rpm") { sensors_.push_back(std::make_unique<ThrusterRpmSensor>(*this, base_frame_)); }
    else if (name == "control_surface_angle") {
      sensors_.push_back(std::make_unique<ControlSurfaceAngleSensor>(*this, base_frame_));
    }
  }
}

void VehicleSimNode::loadParameters() {
  const int frequency = declare_parameter<int>("node_frequency");
  farol2_utils::requireInRange(frequency, 1, 1000000, "node_frequency");
  node_period_ = 1.0 / frequency;
  frame_prefix_ = declare_parameter<std::string>("frame_prefix", "");
  environment_.fluid_density = declare_parameter<double>("environment.fluid_density");
  environment_.gravity = declare_parameter<double>("environment.gravity", 9.8);
  environment_.water_surface_z = declare_parameter<double>("environment.water_surface_z");
  farol2_utils::requirePositive(environment_.fluid_density, "environment.fluid_density");
  farol2_utils::requirePositive(environment_.gravity, "environment.gravity");
  farol2_utils::requireFinite(environment_.water_surface_z, "environment.water_surface_z");
  const double current_speed = declare_parameter<double>("environment.current.speed");
  const double current_angle = declare_parameter<double>("environment.current.angle");
  farol2_utils::requireNonnegative(current_speed, "environment.current.speed");
  farol2_utils::requireFinite(current_angle, "environment.current.angle");
  const double current_angle_rad = farol2_utils::deg2rad(current_angle);
  environment_.current_ned = Eigen::Vector3d(
    current_speed * std::cos(current_angle_rad), current_speed * std::sin(current_angle_rad), 0.0);
  DynamicsParameters p;
  p.mass = declare_parameter<double>("vehicle.mass");
  p.inertia = farol2_utils::declareMatrixParameter(*this, "vehicle.inertia_tensor", 3);
  p.center_of_gravity = farol2_utils::declareVector3Parameter(*this, "vehicle.center_of_gravity");
  p.center_of_buoyancy = farol2_utils::declareVector3Parameter(*this, "vehicle.center_of_buoyancy");
  p.added_mass = farol2_utils::declareMatrixParameter(*this, "vehicle.added_mass_tensor", 6);
  p.linear_damping = farol2_utils::declareMatrixParameter(*this, "vehicle.linear_damping_tensor", 6);
  p.quadratic_damping = farol2_utils::declareMatrixParameter(*this, "vehicle.quadratic_damping_tensor", 6);
  p.displaced_volume = declare_parameter<double>("vehicle.restoring.displaced_volume");
  p.buoyancy_sphere_radius = declare_parameter<double>("vehicle.restoring.sphere_radius", 0.0);
  const auto dofs = declare_parameter<std::vector<int64_t>>("vehicle.active_dofs");
  p.active_dofs.clear();
  for (const auto index : dofs) {
    if (index < 0 || index > 5) { throw std::invalid_argument("active_dofs indices must be in 0..5"); }
    p.active_dofs.push_back(static_cast<int>(index));
  }
  dynamics_ = std::make_unique<Dynamics>(p);

  state_.position_ned = farol2_utils::declareVector3Parameter(*this, "initial_state.position");
  state_.rotation_body_to_ned = farol2_utils::rotationFromRpy(farol2_utils::declareVector3Parameter(*this, "initial_state.orientation"));
  state_.velocity_through_water_body.head<3>() = farol2_utils::declareVector3Parameter(*this, "initial_state.body_velocity");
  state_.velocity_through_water_body.tail<3>() = farol2_utils::declareVector3Parameter(*this, "initial_state.orientation_rate");
  const auto reference = declare_parameter<std::string>("initial_state.velocity_reference");
  if (reference == "ground") {
    state_.velocity_through_water_body.head<3>() -= state_.rotation_body_to_ned.transpose() * environment_.current_ned;
  } else if (reference != "water") {
    throw std::invalid_argument("initial_state.velocity_reference must be ground or water");
  }
  if (!state_.velocity_through_water_body.isApprox(dynamics_->constrainVelocity(state_.velocity_through_water_body), 1e-10)) {
    throw std::invalid_argument("Initial water-relative velocity violates active_dofs");
  }
  validateState(state_);
  base_frame_ = declare_parameter<std::string>("vehicle.actuators.base_frame");
  thruster_frames_ = declare_parameter<std::vector<std::string>>("vehicle.actuators.frames");
  farol2_utils::requireNonEmpty(base_frame_, "vehicle.actuators.base_frame");
  farol2_utils::requireNonEmpty(thruster_frames_, "vehicle.actuators.frames");
  const auto prefix = [this](const std::string & frame) {
    farol2_utils::requireNonEmpty(frame, "actuator frame");
    return frame.rfind(frame_prefix_, 0) == 0 ? frame : frame_prefix_ + frame;
  };
  base_frame_ = prefix(base_frame_);
  for (auto & frame : thruster_frames_) { frame = prefix(frame); }
  farol2_utils::requireUnique(thruster_frames_, "vehicle.actuators.frames");
  ThrusterParameters t;
  const auto model = declare_parameter<std::string>("vehicle.actuators.model");
  if (model == "polynomial") {
    t.positive_curve = farol2_utils::declareVector3Parameter(*this, "vehicle.actuators.lump_param_positive");
    t.negative_curve = farol2_utils::declareVector3Parameter(*this, "vehicle.actuators.lump_param_negative");
  } else if (model == "propeller") {
    t.model = ThrusterParameters::Model::Propeller;
    t.thrust_coefficient = declare_parameter<double>("vehicle.actuators.thrust_coefficient");
    t.diameter = declare_parameter<double>("vehicle.actuators.diameter");
    t.pitch = declare_parameter<double>("vehicle.actuators.pitch");
    t.thrust_scale = declare_parameter<double>("vehicle.actuators.thrust_scale", 1.0);
  } else { throw std::invalid_argument("Unknown actuator model: " + model); }
  t.minimum_rpm = declare_parameter<double>("vehicle.actuators.minimum_rpm");
  t.maximum_rpm = declare_parameter<double>("vehicle.actuators.maximum_rpm");
  t.deadzone = declare_parameter<double>("vehicle.actuators.deadzone");
  t.pole = declare_parameter<double>("vehicle.actuators.pole");
  t.delay = declare_parameter<double>("vehicle.actuators.delay");
  fixed_rpm_ = declare_parameter<bool>("vehicle.actuators.fixed_rpm", false);
  fixed_rpm_value_ = declare_parameter<double>("vehicle.actuators.fixed_rpm_value", 0.0);
  farol2_utils::requireFinite(fixed_rpm_value_, "vehicle.actuators.fixed_rpm_value");
  for (size_t i = 0; i < thruster_frames_.size(); ++i) { thrusters_.emplace_back(t); }
  rpm_commands_.assign(thrusters_.size(), 0.0);
  if (declare_parameter<bool>("vehicle.rudder.enabled", false)) {
    RudderParameters r;
    r.minimum_angle = declare_parameter<double>("vehicle.rudder.minimum_angle");
    r.maximum_angle = declare_parameter<double>("vehicle.rudder.maximum_angle");
    r.rate_limit = declare_parameter<double>("vehicle.rudder.rate_limit");
    r.lift_coefficient = declare_parameter<double>("vehicle.rudder.lift_coefficient");
    r.drag_constant = declare_parameter<double>("vehicle.rudder.drag_constant");
    r.drag_quadratic = declare_parameter<double>("vehicle.rudder.drag_quadratic");
    rudder_geometry_.position_body = farol2_utils::declareVector3Parameter(*this, "vehicle.rudder.position_body");
    rudder_geometry_.rotation_to_body = farol2_utils::rotationFromRpy(farol2_utils::declareVector3Parameter(*this, "vehicle.rudder.orientation_body"));
    rudder_joint_ = declare_parameter<std::string>("vehicle.rudder.joint");
    const auto mode = declare_parameter<std::string>("vehicle.rudder.command_mode", "angle");
    if (mode != "angle" && mode != "rate") { throw std::invalid_argument("Rudder command_mode must be angle or rate"); }
    rudder_rate_mode_ = mode == "rate";
    rudder_speed_ = declare_parameter<double>("vehicle.rudder.command_speed", 0.0);
    farol2_utils::requireNonnegative(rudder_speed_, "vehicle.rudder.command_speed");
    if (rudder_rate_mode_) { farol2_utils::requirePositive(rudder_speed_, "vehicle.rudder.command_speed"); }
    rudder_min_ = r.minimum_angle; rudder_max_ = r.maximum_angle;
    rudder_ = std::make_unique<Rudder>(r);
  }

}

void VehicleSimNode::initializeGeometry() {
  std::vector<ActuatorGeometry> geometry;
  try {
    for (const auto & frame : thruster_frames_) {
      const auto transform = tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
      const auto & t = transform.transform.translation;
      const auto & r = transform.transform.rotation;
      const Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
      ActuatorGeometry g;
      g.position_body << t.x, t.y, t.z;
      farol2_utils::requireFinite(g.position_body, "actuator position: " + frame);
      farol2_utils::requireFinite(q.coeffs(), "actuator quaternion: " + frame);
      if (q.norm() < 1e-9) {
        throw std::invalid_argument("Invalid actuator transform: " + frame);
      }
      g.rotation_to_body = q.normalized().toRotationMatrix();
      geometry.push_back(g);
    }
    for (const auto & sensor : sensors_) { sensor->initializeGeometry(*tf_buffer_, base_frame_); }
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for actuator/sensor geometry: %s", error.what());
    return;
  }
  thruster_geometry_ = std::move(geometry);
  geometry_ready_ = true;
  geometry_timer_->cancel();
  RCLCPP_INFO(get_logger(), "Vehicle ready with %zu thrusters%s", thrusters_.size(), rudder_ ? " and a rudder" : "");
}

void VehicleSimNode::tick() {
  if (!geometry_ready_) { return; }
  try {
    Vector6d wrench = Vector6d::Zero();
    for (size_t i = 0; i < thrusters_.size(); ++i) {
      wrench += thrusters_[i].step(fixed_rpm_ ? fixed_rpm_value_ : rpm_commands_[i],
        node_period_, thruster_geometry_[i], state_, environment_);
    }
    if (rudder_) {
      if (rudder_rate_mode_) {
        const double direction = rudder_rate_command_ > 0.5 ? 1.0 : (rudder_rate_command_ < -0.5 ? -1.0 : 0.0);
        rudder_command_ = std::clamp(rudder_->angle() + direction * rudder_speed_ * node_period_, rudder_min_, rudder_max_);
      }
      wrench += rudder_->step(rudder_command_, node_period_, rudder_geometry_, state_, environment_);
    }
    integrate(state_, *dynamics_, environment_, wrench, node_period_);
    // Evaluate acceleration at the same post-step state supplied to sensors.
    acceleration_ = dynamics_->acceleration(state_, environment_, wrench);
    const auto stamp = now();
    position_pub_->publish(vectorMessage(state_.position_ned));
    body_velocity_pub_->publish(vectorMessage(state_.velocity_through_water_body.head<3>()));
    orientation_pub_->publish(vectorMessage(degrees * farol2_utils::rpyFromRotation(state_.rotation_body_to_ned)));
    orientation_rate_pub_->publish(vectorMessage(degrees * state_.velocity_through_water_body.tail<3>()));
    body_acceleration_pub_->publish(vectorMessage(acceleration_.head<3>()));
    angular_acceleration_pub_->publish(vectorMessage(acceleration_.tail<3>()));
    if (rudder_) {
      sensor_msgs::msg::JointState joints;
      joints.header.stamp = stamp;
      joints.name.push_back(rudder_joint_);
      joints.position.push_back(rudder_->angle());
      joints_pub_->publish(joints);
    }
    publishWorldTransform(stamp);
    SensorState sample{state_, acceleration_, environment_, stamp, {}, {}};
    sample.thruster_rpm.reserve(thrusters_.size());
    for (const auto & thruster : thrusters_) { sample.thruster_rpm.push_back(thruster.rpm()); }
    if (rudder_) { sample.control_surface_angles.push_back(rudder_->angle()); }
    for (const auto & sensor : sensors_) { sensor->publish(sample); }
  } catch (const std::exception & error) {
    timer_->cancel();
    RCLCPP_FATAL(get_logger(), "Simulation stopped: %s", error.what());
    throw;
  }
}

void VehicleSimNode::publishWorldTransform(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = "world";
  transform.child_frame_id = frame_prefix_ + "base_link";
  transform.transform.translation.x = state_.position_ned.x();
  transform.transform.translation.y = state_.position_ned.y();
  transform.transform.translation.z = state_.position_ned.z();

  const Eigen::Quaterniond q(state_.rotation_body_to_ned);
  transform.transform.rotation = tf2::toMsg(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));

  tf_broadcaster_->sendTransform(transform);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VehicleSimNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vehicle_sim"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
