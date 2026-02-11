#include <thruster_rudder_allocation.hpp>

/* Constructor */
ThrusterRudderAllocation::ThrusterRudderAllocation() : Node("thruster_rudder_allocation",
                                      rclcpp::NodeOptions()
                                        .allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true)) {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
}

/* Destructor */
ThrusterRudderAllocation::~ThrusterRudderAllocation() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Initialise Subscribers
 */
void ThrusterRudderAllocation::initialiseSubscribers() {
  body_wrench_request_sub_ = create_subscription<control_allocation::msg::BodyWrenchRequest>(
                              get_parameter("actuation.thruster_rudder_allocation.topics.subscribers.body_wrench_request").as_string(),
                              1, std::bind(&ThrusterRudderAllocation::bodyWrenchRequestCallback, this, std::placeholders::_1));

  nav_state_sub_ = create_subscription<farol_msgs::msg::NavigationState>(
                    get_parameter("actuation.thruster_rudder_allocation.topics.subscribers.nav_state").as_string(),
                    1, std::bind(&ThrusterRudderAllocation::navStateCallback, this, std::placeholders::_1));

  mission_status_sub_ = create_subscription<std_msgs::msg::Int8>(
                          get_parameter("actuation.thruster_rudder_allocation.topics.subscribers.mission_status").as_string(), 
                          1, std::bind(&ThrusterRudderAllocation::missionStatusCallback, this, std::placeholders::_1));

  return;
}

/**
 * @brief Load parameters
 * Thruster configuration parameters are loaded under some assumptions.
 * In the future, if ROS2 enables native parameter loading for dicts and
 * other complex types, this method should be adapted for further robustness.
 */
void ThrusterRudderAllocation::loadParams() {
  /* Angular limits for the rudder */
  rudder_angle_min_ = get_parameter("actuation.rudder.limits.min").as_double()/180*M_PI;
  rudder_angle_max_ = get_parameter("actuation.rudder.limits.max").as_double()/180*M_PI;

  /* Rudder distance to center of mass */
  rudder_cm_distance_ = get_parameter("actuation.rudder.cm_distance").as_double();

  /* Gains */
  K_s_ = get_parameter("actuation.model.K_s").as_double();
  K_L_ = get_parameter("actuation.model.K_L").as_double();
  K_D0_ = get_parameter("actuation.model.K_D0").as_double();
  K_D1_ = get_parameter("actuation.model.K_D1").as_double();

  /* Get thruster configuration */
  thruster_configuration_ = getThrusterConfiguration(*this);

  /* Number of thrusters */
  nr_thrusters_ = (int)thruster_configuration_.size();

  /* Get thrust allocation matrix */
  thrust_allocation_matrix_ = getThrustAllocationMatrix(thruster_configuration_, nr_thrusters_);

  /* Set size of pseudo-inverse and forces output */
  thrust_allocation_matrix_pseudo_inv_.resize(nr_thrusters_, 6);
  forces_.resize(nr_thrusters_);

  /* Compute pseudo inverse */
  thrust_allocation_matrix_pseudo_inv_ = thrust_allocation_matrix_.completeOrthogonalDecomposition().pseudoInverse();

  // std::cout << "TAM:\n" << thrust_allocation_matrix_ << std::endl;
  // std::cout << "pinv(TAM):\n" << thrust_allocation_matrix_pseudo_inv_ << std::endl;
}

/**
 * @brief Initialise Publishers
 */
void ThrusterRudderAllocation::initialisePublishers() {
  thruster_force_pub_ = create_publisher<control_allocation::msg::ThrusterForce>(
                          get_parameter("actuation.thruster_rudder_allocation.topics.publishers.thruster_force").as_string(), 1);

  rudder_angle_ref_pub_ = create_publisher<std_msgs::msg::Float32>(
                            get_parameter("actuation.thruster_rudder_allocation.topics.publishers.rudder_angle_ref").as_string(), 1);

  debug1_pub_ = create_publisher<std_msgs::msg::Float32>("debug1", 1);
  // rudder_angle_ref_pub_ = create_publisher<std_msgs::msg::Float32>("debug1", 1);
}

/**
 * @brief Initialise Services
 */
void ThrusterRudderAllocation::initialiseServices() {
  /* Service servers */
  /* ... */

  /* service clients */
  /* ... */

  return;
}

/**
 * @brief Initialise Timers
 */
void ThrusterRudderAllocation::initialiseTimers() {
  /* Get node frequency from parameters */
  int freq = get_parameter("actuation.thruster_rudder_allocation.node_frequency").as_int();

  /* Create timer */
  timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq*1000)), std::bind(&ThrusterRudderAllocation::timerCallback, this));
}

/**
 * @brief Compute force for each thruster based on body wrench (force and torque) request.
 */
void ThrusterRudderAllocation::bodyWrenchRequestCallback(const control_allocation::msg::BodyWrenchRequest &msg) {
  /* Body wrench request */
  tau_ << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
          msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;

  /* Compute rudder angle based on requested torque around the Z axis */
  /* and expected drag along the body's X axis                        */
  computeRudderAngle(tau_[5]);

  /* Set requested forces and torques, accounting for drag caused by rudder */
  /* Since common mode is used, all forces and torques are set to 0, except */
  /* for force along X axis */
  // tau_common_mode_ << tau_[0] + rudder_x_body_drag_, 0.0, 0.0,
  //                     0.0, 0.0, 0.0;
  tau_common_mode_ << tau_[0], 0.0, 0.0,
                      0.0, 0.0, 0.0;

  /* Compute vector of forces for each thruster based on body wrench request */
  /* f = pinv(T).τ */
  forces_ = thrust_allocation_matrix_pseudo_inv_*tau_common_mode_;

  /* Create message to publish thruster force */
  thruster_force_msg_.header.stamp = clock_.now();

  std::vector<double> forces_vec(forces_.data(), forces_.data() + forces_.size());
  thruster_force_msg_.force = forces_vec;

  /* If we are not in a mission, don't publish */
  if (mission_status_ != 0) {
    thruster_force_pub_->publish(thruster_force_msg_);

    /* Create message to publish rudder angle reference */
    rudder_angle_ref_msg_.data = rudder_angle_;

    rudder_angle_ref_pub_->publish(rudder_angle_ref_msg_);
  }
}

void ThrusterRudderAllocation::computeRudderAngle(double tau_r) 
{
  /* Cap velocity to avoid division by 0 on later computations */
  nav_state_.body_velocity_fluid.x =
    (std::abs(nav_state_.body_velocity_fluid.x) < 0.05)
      ? std::copysign(0.05, nav_state_.body_velocity_fluid.x == 0.0 ? 1.0 : nav_state_.body_velocity_fluid.x)
      : nav_state_.body_velocity_fluid.x;

  /* Course angle = Heading + Sideslip */
  sideslip_angle_ = (nav_state_.body_velocity_fluid.x != 0.0) ? atan2(nav_state_.body_velocity_fluid.y, nav_state_.body_velocity_fluid.x) : 0.0;
  course_angle_ = nav_state_.orientation.z + sideslip_angle_;

  /* Compute velocity at the rudder */
  /* V_r = r * l * [sin(yaw), -cos(yaw)], r -> yaw rate, l -> distance from rudder to center of mass */
  V_cm_ = Eigen::Vector2d(std::cos(course_angle_), std::sin(course_angle_)) * std::hypot(nav_state_.body_velocity_fluid.x, nav_state_.body_velocity_fluid.y);
  V_r_ = Eigen::Vector2d(std::sin(nav_state_.orientation.z), -std::cos(nav_state_.orientation.z)) * rudder_cm_distance_ * nav_state_.orientation_rate.z;
  V_s_ = V_cm_ + V_r_;

  // angle between Vs and x_body of the boat
  gamma_ = farol_utils::wrapToPi(std::atan2(V_s_(1),V_s_(0)) - nav_state_.orientation.z);

  /* Compute rudder angle according to Fossen model, in "A Survey of Control Allocation Methods for Underwater Vehicles", p. 126 */
  /* N = K.l.v^2.δ */
  rudder_angle_ = tau_r / (K_s_ * rudder_cm_distance_ * V_s_.dot(V_s_));

  /* Compute rudder angle using inversion of the function ... See ... i am the documentation bruh */
  // rudder_angle_ = solve_delta_from_tau(tau_r, gamma_, V_s_.dot(V_s_));

  /* Saturate rudder_angle */
  rudder_angle_ = (rudder_angle_ > rudder_angle_max_) ? rudder_angle_max_ : ((rudder_angle_ < rudder_angle_min_) ? rudder_angle_min_ : rudder_angle_);

  /* Compute fluid flow to rudder angle */
  V_s_angle_ = (V_s_[0] != 0.0) ? atan2(V_s_[1], V_s_[0]) : 0.0;
  flow_to_rudder_angle_ = rudder_angle_ + V_s_angle_ - nav_state_.orientation.z;

  /* Compute lift and drag */
  L = K_L_ * flow_to_rudder_angle_ * V_s_.dot(V_s_);
  D = (K_D0_ + K_D1_ * std::pow(flow_to_rudder_angle_, 2)) * V_s_.dot(V_s_);

  /* Compute rudder induced drag along the body X axis */
  rudder_x_body_drag_ = D*std::cos(flow_to_rudder_angle_) + L*std::sin(-flow_to_rudder_angle_);
}


double ThrusterRudderAllocation::solve_delta_from_tau(double tau_r, double gamma, double V_sq)
{
  double eps = 1e-12;

  // Solve a*alpha^2 + b*alpha + c = 0 for alpha = delta + gamma
  const double a = K_D1_ * std::sin(gamma);
  const double b = K_L_  * std::cos(gamma);
  const double c = K_D0_ * std::sin(gamma) - tau_r/(rudder_cm_distance_ * V_sq);


  // Very small drag (linear)
  if (std::abs(a) < eps) {
    if (std::abs(b) < eps) {
      RCLCPP_WARN_STREAM(get_logger(), "tau_r basically independent of delta");
      return 0.0;
    }

    double alpha = -c / b;
    // RCLCPP_INFO_STREAM(get_logger(), "solution found for small drag");
    return alpha - gamma;
  }

  // Quadratic discriminant
  double D = b*b - 4.0*a*c;

  // Treat tiny negative as zero (floating point noise)
  if (D < 0.0 && D > -1e-12) D = 0.0;

  // Never happens in practice
  if (D < 0.0) {
    RCLCPP_WARN_STREAM(get_logger(), "No real solution: D < 0.0");
    return 0.0;
  }

  // Quadratic formula but more stable
  const double sign_b = (b >= 0.0) ? 1.0 : -1.0;
  const double q = -0.5 * (b + sign_b * std::sqrt(D));

  double alpha1, alpha2;
  if (std::abs(q) < eps) {
    // fallback to classic formula (only if D is close to 0)
    alpha1 = (-b + std::sqrt(D)) / (2.0*a);
    alpha2 = (-b - std::sqrt(D)) / (2.0*a);
  } else {
    alpha1 = q / a;
    alpha2 = c / q;
  }

  const double d1 = alpha1 - gamma;
  const double d2 = alpha2 - gamma;

  // prefer solution closest to previous rudder angle
  // double d = (std::abs(d1 - delta_prev) <= std::abs(d2 - delta_prev)) ? d1 : d2;
  double d = (std::abs(d1) <= std::abs(d2)) ? d1 : d2;

  return d;
}


/**
 * @brief Callback for navigation state.
 */
void ThrusterRudderAllocation::navStateCallback(const farol_msgs::msg::NavigationState &msg) {
  nav_state_ = msg;
}

/**
 * @brief Callback for navigation state.
 */
void ThrusterRudderAllocation::missionStatusCallback(const std_msgs::msg::Int8 &msg) {
  mission_status_ = msg.data;
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void ThrusterRudderAllocation::timerCallback() {
  /* Check if body velocity relative to fluid is being published on */
  if (nav_state_.body_velocity_fluid.x == 0.0 && nav_state_.body_velocity_fluid.y == 0.0 && nav_state_.body_velocity_fluid.z == 0.0) {
    RCLCPP_WARN(get_logger(), "Body Velocity relative to the fluid is 0. Is it not being updated?");
  }

  return;
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterRudderAllocation>());
  rclcpp::shutdown();
  return 0;
}
