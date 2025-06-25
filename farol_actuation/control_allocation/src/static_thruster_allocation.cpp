#include <static_thruster_allocation.hpp>

/* Constructor */
StaticThrusterAllocation::StaticThrusterAllocation() : Node("static_thruster_allocation", 
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
StaticThrusterAllocation::~StaticThrusterAllocation() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Initialise Subscribers
 */
void StaticThrusterAllocation::initialiseSubscribers() {
  body_wrench_request_sub_ = create_subscription<control_allocation::msg::BodyWrenchRequest>(
                              get_parameter("actuation.static_thruster_allocation.topics.subscribers.body_wrench_request").as_string(), 
                              1, std::bind(&StaticThrusterAllocation::bodyWrenchRequestCallback, this, std::placeholders::_1));

  return;
}

/**
 * @brief Load parameters
 * Thruster configuration parameters are loaded under some assumptions.
 * In the future, if ROS2 enables native parameter loading for dicts and 
 * other complex types, this method should be adapted for further robustness.
 */
void StaticThrusterAllocation::loadParams() {
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
void StaticThrusterAllocation::initialisePublishers() {
  thruster_force_pub_ = create_publisher<control_allocation::msg::ThrusterForce>(
                          get_parameter("actuation.static_thruster_allocation.topics.publishers.thruster_force").as_string(), 1);
}

/**
 * @brief Initialise Services
 */
void StaticThrusterAllocation::initialiseServices() {
  /* Service servers */
  /* ... */

  /* service clients */
  /* ... */

  return;
}

/**
 * @brief Initialise Timers
 */
void StaticThrusterAllocation::initialiseTimers() {
  /* Get node frequency from parameters */
  int freq = get_parameter("actuation.static_thruster_allocation.node_frequency").as_int();

  /* Create timer */
  timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq*1000)), std::bind(&StaticThrusterAllocation::timerCallback, this));
}

/**
 * @brief Compute force for each thruster based on body wrench (force and torque) request.
 */
void StaticThrusterAllocation::bodyWrenchRequestCallback(const control_allocation::msg::BodyWrenchRequest &msg) {
  /* Body wrench request */
  tau_ << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
          msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;
  
  /* Compute vector of forces for each thruster based on body wrench request */
  /* f = pinv(T).τ */
  forces_ = thrust_allocation_matrix_pseudo_inv_*tau_;

  /* Create message to publish */
  msg_.header.stamp = clock_.now();
  
  std::vector<double> forces_vec(forces_.data(), forces_.data() + forces_.size());
  msg_.force = forces_vec;
  
  thruster_force_pub_->publish(msg_);
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void StaticThrusterAllocation::timerCallback() {
  return;
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticThrusterAllocation>());
  rclcpp::shutdown();
  return 0;
}
