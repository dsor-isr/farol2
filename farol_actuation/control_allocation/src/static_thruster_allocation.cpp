#include <static_thruster_allocation.hpp>

/* Constructor */
StaticThrusterAllocation::StaticThrusterAllocation() : Node("static_thruster_allocation", 
                                      rclcpp::NodeOptions()
                                        .allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true)) {
  initialiseSubscribers();
  loadParams();
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
  int idx = 0;
  std::string key_name;
  /* Get raw flatten thruster configuration parameters */
  if (get_node_parameters_interface()->get_parameters_by_prefix(
        "actuation.thrusters", raw_thruster_configuration_)) {
    /* Iterate through std::map to create new map with thruster configurations */
    for (const auto & [key, param] : raw_thruster_configuration_) {
      /* Get number at the beginning of the key and param name */
      try {
        idx = std::stoi(key.substr(0, key.find('.')));
        key_name = key.substr(key.find('.') + 1);
      } catch (...) {
        continue;
      }
      
      /* Create element in thruster_configuration vector with map or add info to already existing map */
      if (idx >= (int)thruster_configuration_.size()) {
        RCLCPP_DEBUG(get_logger(), "NEW ENTRY: %d -- %ld", idx, thruster_configuration_.size());
        thruster_configuration_.push_back({{key_name, getFieldFromParameter(param, key_name)}});
      } else {
        thruster_configuration_[idx].insert({key_name, getFieldFromParameter(param, key_name)});
      }
    }
  }

  buildThrustAllocationMatrix((int)thruster_configuration_.size());
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
  forces_ = thrust_allocation_matrix_pseudo_inv_*tau_;

  /* Create message to publish */
  msg_.header.stamp = clock_.now();
  
  std::vector<double> forces_vec(forces_.data(), forces_.data() + forces_.size());
  msg_.force = forces_vec;
  
  thruster_force_pub_->publish(msg_);
}

/**
 * @brief Get field from ROS parameter according to key name.
 */
std::variant<std::string, std::vector<double>> StaticThrusterAllocation::getFieldFromParameter(rclcpp::Parameter param, std::string key_name) {
  /* According to key name, parse ros parameter properly */
  if (key_name == "name") {
    return param.as_string();
  } else if (key_name == "moment_arms" || key_name == "angles") {
    return param.as_double_array();
  } else {
    RCLCPP_WARN(get_logger(), "Unknown loaded data from config in thruster configuration.");
    return "Unknown";
  }
}

/**
 * @brief Build thrust allocation matrix based on thruster configuration.
 */
void StaticThrusterAllocation::buildThrustAllocationMatrix(const int nr_thrusters) {
  /* Set size of thrust allocation matrix, its pseudo-inverse and forces output */
  thrust_allocation_matrix_.resize(6, nr_thrusters);
  thrust_allocation_matrix_pseudo_inv_.resize(nr_thrusters, 6);
  forces_.resize(nr_thrusters);
  
  /* Normalised vector of forces in the thruster's referential */
  const Eigen::Vector3d unit_vector(1.0, 0.0, 0.0);

  /* Build matrix column by column */
  for (int i = 0; i < nr_thrusters; i++) {
    /* Compute vector of forces */
    f_ = getRotationMatrixThruster2Body(std::get<std::vector<double>>(thruster_configuration_[i]["angles"])[0]/180*M_PI,
                                        std::get<std::vector<double>>(thruster_configuration_[i]["angles"])[1]/180*M_PI,
                                        std::get<std::vector<double>>(thruster_configuration_[i]["angles"])[2]/180*M_PI)*unit_vector;

    /* Get vector of moment arms */
    l_ << std::get<std::vector<double>>(thruster_configuration_[i]["moment_arms"])[0],
          std::get<std::vector<double>>(thruster_configuration_[i]["moment_arms"])[1],
          std::get<std::vector<double>>(thruster_configuration_[i]["moment_arms"])[2];
    
    /* Each column of the Thrust Allocation Matrix:                            */
    /* tau_i = |    f_i    | <- forces vector                                  */
    /*         | l_i x f_i | <- cross product of moment arms and forces vector */
    thrust_allocation_matrix_.block<3,1>(0,i) = f_;
    thrust_allocation_matrix_.block<3,1>(3,i) = l_.cross(f_);
  }

  /* Pseudo Inverse */
  thrust_allocation_matrix_pseudo_inv_ = thrust_allocation_matrix_.completeOrthogonalDecomposition().pseudoInverse();
  
  // std::cout << "TAM:\n" << thrust_allocation_matrix_ << std::endl;
  // std::cout << "pinv(TAM):\n" << thrust_allocation_matrix_pseudo_inv_ << std::endl;
}

/**
 * @brief Get rotation matrix from thruster frame to body frame, given the roll,
 * pitch and yaw angles, which are the angles of rotation from body to thruster,
 * in the following order: yaw first, then pitch, finally roll.
 */
Eigen::Matrix3d StaticThrusterAllocation::getRotationMatrixThruster2Body(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).matrix();
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
