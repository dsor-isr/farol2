#include <simulation.hpp>

/* Constructor */
Simulation::Simulation() : Node("simulation", 
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
Simulation::~Simulation() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void Simulation::loadParams() {
  param1_ = get_parameter("sim.simulation.param1").as_string();
  list_of_params_ = get_parameter("sim.simulation.list_of_params").as_string_array();
  param2_ = get_parameter("sim.simulation.param2").as_int();
}

/**
 * @brief Initialise Subscribers
 */
void Simulation::initialiseSubscribers() {
  return;
}

/**
 * @brief Initialise Publishers
 */
void Simulation::initialisePublishers() {
  return;
}

/**
 * @brief Initialise Services
 */
void Simulation::initialiseServices() {
  /* Service servers */
  /* ... */

  /* service clients */
  /* ... */

  return;
}

/**
 * @brief Initialise Timers
 */
void Simulation::initialiseTimers() {
  /* Get node frequency from parameters */
  int freq = get_parameter("sim.simulation.node_frequency").as_int();

  /* Create timer */
  timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq*1000)), std::bind(&Simulation::timerCallback, this));
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void Simulation::timerCallback() {
  return;
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulation>());
  rclcpp::shutdown();
  return 0;
}
