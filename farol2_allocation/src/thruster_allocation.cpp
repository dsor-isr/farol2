#include <thruster_allocation.hpp>

using namespace std::chrono_literals;

ThrusterAllocation::ThrusterAllocation() : Node("thruster_allocation") {
  clock_ = this->get_clock();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseTimers();
}

ThrusterAllocation::~ThrusterAllocation() = default;

void ThrusterAllocation::loadParams() {
  node_frequency_ = declare_parameter<double>("node_frequency");

  const auto allocation_type = declare_parameter<std::string>("allocation.type");
  allocation_type_ = parseAllocationType(allocation_type);

  base_frame_ = declare_parameter<std::string>("allocation.thrusters.base_frame");
  if (base_frame_.empty()) {
    throw std::runtime_error("allocation.thrusters.base_frame cannot be empty");
  }

  const auto thrust_axis = declare_parameter<std::vector<double>>("allocation.thrusters.thrust_axis");
  if (thrust_axis.size() != 3) {
    throw std::runtime_error("allocation.thrusters.thrust_axis must have exactly 3 values");
  }
  thrust_axis_ << thrust_axis[0], thrust_axis[1], thrust_axis[2];
  if (thrust_axis_.norm() <= 1e-9) {
    throw std::runtime_error("allocation.thrusters.thrust_axis cannot have near-zero norm");
  }
  thrust_axis_.normalize();

  thruster_frames_ = declare_parameter<std::vector<std::string>>("allocation.thrusters.frames");
  if (thruster_frames_.empty()) {
    throw std::runtime_error("allocation.thrusters.frames must contain at least one TF frame");
  }
  for (const auto & frame : thruster_frames_) {
    if (frame.empty()) {
      throw std::runtime_error("allocation.thrusters.frames cannot contain empty frame names");
    }
  }

  if (allocation_type_ == AllocationType::THRUST_RUDDER) {
    rudder_angle_min_ = declare_parameter<double>("allocation.rudder.limits.min") / 180.0 * M_PI;
    rudder_angle_max_ = declare_parameter<double>("allocation.rudder.limits.max") / 180.0 * M_PI;
    rudder_cm_distance_ = declare_parameter<double>("allocation.rudder.cm_distance");

    K_s_ = declare_parameter<double>("allocation.model.K_s");
    K_L_ = declare_parameter<double>("allocation.model.K_L");
    K_D0_ = declare_parameter<double>("allocation.model.K_D0");
    K_D1_ = declare_parameter<double>("allocation.model.K_D1");

    open_loop_ = declare_parameter<bool>("allocation.thrusters.open_loop");
  }
}

void ThrusterAllocation::initialiseSubscribers() {
  body_wrench_request_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    TOPIC_SUB_BODY_WRENCH_REQUEST,
    rclcpp::QoS(1),
    [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg){bodyWrenchRequestCallback(msg);});

  if (allocation_type_ == AllocationType::THRUST_RUDDER) {
    nav_state_sub_ = create_subscription<farol2_interfaces::msg::NavigationState>(
      TOPIC_SUB_NAV_STATE,
      rclcpp::QoS(1),
      [this](farol2_interfaces::msg::NavigationState::SharedPtr msg){nav_state_ = *msg;});

    mission_status_sub_ = create_subscription<std_msgs::msg::Int8>(
      TOPIC_SUB_MISSION_STATUS,
      rclcpp::QoS(1),
      [this](std_msgs::msg::Int8::SharedPtr msg){mission_status_ = msg->data;});
  }
}

void ThrusterAllocation::initialisePublishers() {
  thruster_force_pub_ = create_publisher<farol2_allocation::msg::ThrusterForce>(
    TOPIC_PUB_THRUSTER_FORCE,
    rclcpp::QoS(1));

  if (allocation_type_ == AllocationType::THRUST_RUDDER) {
    rudder_command_pub_ = create_publisher<std_msgs::msg::Float32>(
      TOPIC_PUB_RUDDER_COMMAND,
      rclcpp::QoS(1));
  }
}

void ThrusterAllocation::initialiseTimers() {
  tf_initialisation_timer_ = create_wall_timer(
    500ms,
    [this](){initialiseAllocationFromTF();});
}

ThrusterAllocation::AllocationType ThrusterAllocation::parseAllocationType(
  const std::string & allocation_type) const {
  if (allocation_type == "thrust_rudder" || allocation_type == "thruster_rudder" ||
      allocation_type == "thruster_rudder_allocation") {
    return AllocationType::THRUST_RUDDER;
  }

  if (allocation_type == "thrust" || allocation_type == "static_thruster_allocation") {
    return AllocationType::THRUST;
  }

  throw std::runtime_error("Unknown allocation.type '" + allocation_type + "'");
}

void ThrusterAllocation::initialiseAllocationFromTF() {
  if (allocation_ready_) {
    return;
  }

  if (!buildThrusterFramesFromTF()) {
    return;
  }

  if (!buildAllocationMatrixFromTF()) {
    return;
  }

  thrust_allocation_matrix_pseudo_inv_.resize(nr_thrusters_, 6);
  forces_.resize(nr_thrusters_);
  thrust_allocation_matrix_pseudo_inv_ =
    thrust_allocation_matrix_.completeOrthogonalDecomposition().pseudoInverse();

  allocation_ready_ = true;
  tf_initialisation_timer_->cancel();

  RCLCPP_INFO(
    get_logger(),
    "Thruster allocation ready with %zu TF thruster frames using base frame '%s'.",
    nr_thrusters_,
    base_frame_.c_str());
}

bool ThrusterAllocation::buildThrusterFramesFromTF() {
  nr_thrusters_ = thruster_frames_.size();
  return true;
}

bool ThrusterAllocation::buildAllocationMatrixFromTF() {
  thrust_allocation_matrix_.resize(6, nr_thrusters_);

  for (size_t i = 0; i < nr_thrusters_; ++i) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        base_frame_,
        thruster_frames_[i],
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *clock_,
        5000,
        "Waiting for transform %s -> %s: %s",
        base_frame_.c_str(),
        thruster_frames_[i].c_str(),
        ex.what());
      return false;
    }

    const auto & t = transform.transform.translation;
    const auto & q_msg = transform.transform.rotation;
    Eigen::Vector3d l(t.x, t.y, t.z);
    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    Eigen::Vector3d f = q.normalized().toRotationMatrix() * thrust_axis_;

    thrust_allocation_matrix_.block<3,1>(0, i) = f;
    thrust_allocation_matrix_.block<3,1>(3, i) = l.cross(f);
  }

  return true;
}

void ThrusterAllocation::bodyWrenchRequestCallback(geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  if (!allocation_ready_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000, "Ignoring wrench request while waiting for TF allocation.");
    return;
  }

  if (allocation_type_ == AllocationType::THRUST_RUDDER && mission_status_ == 0) {
    return;
  }

  tau_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
          msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

  if (allocation_type_ == AllocationType::THRUST_RUDDER) {
    computeRudderAngle(tau_[5]);
    tau_common_mode_ << tau_[0], 0.0, 0.0,
                        0.0, 0.0, 0.0;
    forces_ = thrust_allocation_matrix_pseudo_inv_ * tau_common_mode_;

    if (!open_loop_) {
      thruster_force_msg_.header.stamp = clock_->now();
      std::vector<double> forces_vec(forces_.data(), forces_.data() + forces_.size());
      thruster_force_msg_.force = forces_vec;
      thruster_force_pub_->publish(thruster_force_msg_);
    }

    rudder_command_msg_.data = farol2_utils::rad2deg(rudder_angle_);
    rudder_command_pub_->publish(rudder_command_msg_);
    return;
  }

  forces_ = thrust_allocation_matrix_pseudo_inv_ * tau_;
  thruster_force_msg_.header.stamp = clock_->now();
  std::vector<double> forces_vec(forces_.data(), forces_.data() + forces_.size());
  thruster_force_msg_.force = forces_vec;
  thruster_force_pub_->publish(thruster_force_msg_);
}

void ThrusterAllocation::computeRudderAngle(double tau_r)
{
  nav_state_.velocity_through_water_body.x =
    (std::abs(nav_state_.velocity_through_water_body.x) < 0.05)
      ? std::copysign(0.05, nav_state_.velocity_through_water_body.x == 0.0 ? 1.0 : nav_state_.velocity_through_water_body.x)
      : nav_state_.velocity_through_water_body.x;

  sideslip_angle_ = (nav_state_.velocity_through_water_body.x != 0.0) ? atan2(nav_state_.velocity_through_water_body.y, nav_state_.velocity_through_water_body.x) : 0.0;
  course_angle_ = nav_state_.attitude.z + sideslip_angle_;

  V_cm_ = Eigen::Vector2d(std::cos(course_angle_), std::sin(course_angle_)) * std::hypot(nav_state_.velocity_through_water_body.x, nav_state_.velocity_through_water_body.y);
  V_r_ = Eigen::Vector2d(std::sin(nav_state_.attitude.z), -std::cos(nav_state_.attitude.z)) * rudder_cm_distance_ * nav_state_.angular_velocity.z;
  V_s_ = V_cm_ + V_r_;

  gamma_ = farol2_utils::wrapToPi(std::atan2(V_s_(1),V_s_(0)) - nav_state_.attitude.z);

  rudder_angle_ = tau_r / (K_s_ * rudder_cm_distance_ * 1.75);
  rudder_angle_ = (rudder_angle_ > rudder_angle_max_) ? rudder_angle_max_ : ((rudder_angle_ < rudder_angle_min_) ? rudder_angle_min_ : rudder_angle_);

  V_s_angle_ = (V_s_[0] != 0.0) ? atan2(V_s_[1], V_s_[0]) : 0.0;
  flow_to_rudder_angle_ = rudder_angle_ + V_s_angle_ - nav_state_.attitude.z;

  L = K_L_ * flow_to_rudder_angle_ * V_s_.dot(V_s_);
  D = (K_D0_ + K_D1_ * std::pow(flow_to_rudder_angle_, 2)) * V_s_.dot(V_s_);

  rudder_x_body_drag_ = D*std::cos(flow_to_rudder_angle_) + L*std::sin(-flow_to_rudder_angle_);
}

double ThrusterAllocation::solve_delta_from_tau(double tau_r, double gamma, double V_sq)
{
  double eps = 1e-12;

  const double a = K_D1_ * std::sin(gamma);
  const double b = K_L_  * std::cos(gamma);
  const double c = K_D0_ * std::sin(gamma) - tau_r/(rudder_cm_distance_ * V_sq);

  if (std::abs(a) < eps) {
    if (std::abs(b) < eps) {
      RCLCPP_WARN_STREAM(get_logger(), "tau_r basically independent of delta");
      return 0.0;
    }

    double alpha = -c / b;
    return alpha - gamma;
  }

  double D = b*b - 4.0*a*c;

  if (D < 0.0 && D > -1e-12) D = 0.0;

  if (D < 0.0) {
    RCLCPP_WARN_STREAM(get_logger(), "No real solution: D < 0.0");
    return 0.0;
  }

  const double sign_b = (b >= 0.0) ? 1.0 : -1.0;
  const double q = -0.5 * (b + sign_b * std::sqrt(D));

  double alpha1, alpha2;
  if (std::abs(q) < eps) {
    alpha1 = (-b + std::sqrt(D)) / (2.0*a);
    alpha2 = (-b - std::sqrt(D)) / (2.0*a);
  } else {
    alpha1 = q / a;
    alpha2 = c / q;
  }

  const double d1 = alpha1 - gamma;
  const double d2 = alpha2 - gamma;

  double d = (std::abs(d1) <= std::abs(d2)) ? d1 : d2;

  return d;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterAllocation>());
  rclcpp::shutdown();
  return 0;
}
