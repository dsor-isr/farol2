#include <static_thruster_allocation.hpp>

/* Constructor */
StaticThrusterAllocation::StaticThrusterAllocation() : Node("static_thruster_allocation"){
  clock_ = this->get_clock();
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  // initialiseServices();
}

/* Destructor */
StaticThrusterAllocation::~StaticThrusterAllocation() = default;

/**
 * @brief Initialise Subscribers
 */
void StaticThrusterAllocation::initialiseSubscribers() {
  body_wrench_request_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    TOPIC_SUB_BODY_WRENCH_REQUEST,
    rclcpp::QoS(1),
    [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg){bodyWrenchRequestCallback(msg);});
}

/**
 * @brief Load parameters
 * Thruster configuration parameters are loaded under some assumptions.
 * In the future, if ROS2 enables native parameter loading for dicts and 
 * other complex types, this method should be adapted for further robustness.
 */
void StaticThrusterAllocation::loadParams() {
  node_frequency_ = declare_parameter<double>("node_frequency");  

  nr_thrusters_ = declare_parameter<int>("thrusters.nr_thrusters");
  for (size_t i = 0; i < nr_thrusters_; i++){
    declare_parameter<std::string>("thrusters.configuration."+ std::to_string(i) + ".name");
    declare_parameter<std::vector<double>>("thrusters.configuration."+ std::to_string(i) + ".moment_arms");
    declare_parameter<std::vector<double>>("thrusters.configuration."+ std::to_string(i) + ".angles");
  }

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
  thruster_force_pub_ = create_publisher<farol2_allocation::msg::ThrusterForce>(
    TOPIC_PUB_THRUSTER_FORCE,
    rclcpp::QoS(1));
}

/**
 * @brief Initialise Services
 */
void StaticThrusterAllocation::initialiseServices() {}

/**
 * @brief Compute force for each thruster based on body wrench (force and torque) request.
 */
void StaticThrusterAllocation::bodyWrenchRequestCallback(geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  /* Body wrench request */
  tau_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
          msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  
  /* Compute vector of forces for each thruster based on body wrench request */
  /* f = pinv(T).τ */
  forces_ = thrust_allocation_matrix_pseudo_inv_*tau_;

  /* Create message to publish */
  msg_.header.stamp = clock_->now();
  
  std::vector<double> forces_vec(forces_.data(), forces_.data() + forces_.size());
  msg_.force = forces_vec;
  
  thruster_force_pub_->publish(msg_);
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
