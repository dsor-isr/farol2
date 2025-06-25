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
  tau_common_mode_ << tau_[0] + rudder_x_body_drag_, 0.0, 0.0, 
                      0.0, 0.0, 0.0;
  
  /* Compute vector of forces for each thruster based on body wrench request */
  /* f = pinv(T).τ */
  forces_ = thrust_allocation_matrix_pseudo_inv_*tau_common_mode_;

  /* Create message to publish thruster force */
  thruster_force_msg_.header.stamp = clock_.now();

  std::vector<double> forces_vec(forces_.data(), forces_.data() + forces_.size());
  thruster_force_msg_.force = forces_vec;
  
  thruster_force_pub_->publish(thruster_force_msg_);

  /* Create message to publish rudder angle reference */
  rudder_angle_ref_msg_.data = rudder_angle_;

  rudder_angle_ref_pub_->publish(rudder_angle_ref_msg_);
}

void ThrusterRudderAllocation::computeRudderAngle(double tau_r) {
  /* If vehicle has no forward motion, output rudder angle 0 */
  if (nav_state_.body_velocity_fluid.x == 0.0) {
    rudder_angle_ = 0.0;
    rudder_x_body_drag_ = 0.0;
    return;
  }

  /* Course angle = Heading + Sideslip */
  sideslip_angle_ = (nav_state_.body_velocity_fluid.x != 0.0) ? atan2(nav_state_.body_velocity_fluid.y, nav_state_.body_velocity_fluid.x) : 0.0;
  course_angle_ = nav_state_.orientation.z + sideslip_angle_;

  /* Compute velocity at the rudder */
  /* V_r = r * l * [sin(yaw), -cos(yaw)], r -> yaw rate, l -> distance from rudder to center of mass */
  V_cm_ = Eigen::Vector2d(std::cos(course_angle_), std::sin(course_angle_)) * std::hypot(nav_state_.body_velocity_fluid.x, nav_state_.body_velocity_fluid.y);
  V_r_ = Eigen::Vector2d(std::sin(nav_state_.orientation.z), -std::cos(nav_state_.orientation.z)) * rudder_cm_distance_ * nav_state_.orientation_rate.z;
  V_s_ = V_cm_ + V_r_;

  /* Compute rudder angle according to Fossen model, in "A Survey of Control Allocation Methods for Underwater Vehicles", p. 126 */
  /* N = K.l.v^2.δ */
  rudder_angle_ = tau_r / (K_s_ * rudder_cm_distance_ * V_s_.dot(V_s_));

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

/**
 * @brief Callback for navigation state.
 */
void ThrusterRudderAllocation::navStateCallback(const farol_msgs::msg::NavigationState &msg) {
  nav_state_ = msg;
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
