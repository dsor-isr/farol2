#include "CpfNode.hpp"

/**
 * @brief  Constructor for the CpfNode
 */
CpfNode::CpfNode() : rclcpp::Node("cpf_node",
                                  rclcpp::NodeOptions()
                                    .automatically_declare_parameters_from_overrides(true)) {
  clock_ = this->get_clock();
  this->initializeSubscribers();
  this->initializePublishers();
  this->initializeServices();
  this->initializeTimer();

  /* Allocate memory for the default CPF Algorithm - Event Triggered */
  this->cooperative_ = this->createEventTriggeredControl();
}

/**
 * @brief  Node class destructor
 */
CpfNode::~CpfNode() {
  /* Stop the timer */
  if (timer_) {
    timer_->cancel();
  }

  /* Free the memory allocated for the cooperative object */
  if (this->cooperative_) {
    delete this->cooperative_;
    this->cooperative_ = nullptr;
  }
}

/**
 * @brief  Creates the default Event Triggered CPF controller
 */
CPFControl* CpfNode::createEventTriggeredControl() {
  int ID = 0;
  std::vector<int64_t> adj_matrix;
  double k_epsilon = 0.0, c0 = 0.0, c1 = 0.0, alpha = 0.0;

  /* Declare and read parameters */
  this->declare_parameter<int>("ID", 0);
  this->declare_parameter<std::vector<int64_t>>("adjency_matrix", std::vector<int64_t>{});
  this->declare_parameter<double>("gains.event_triggered.c0", 0.0);
  this->declare_parameter<double>("gains.event_triggered.c1", 0.0);
  this->declare_parameter<double>("gains.event_triggered.alpha", 0.0);
  this->declare_parameter<double>("gains.event_triggered.k_epsilon", 0.0);

  ID = this->get_parameter("ID").as_int();
  adj_matrix = this->get_parameter("adjency_matrix").as_integer_array();
  c0 = this->get_parameter("gains.event_triggered.c0").as_double();
  c1 = this->get_parameter("gains.event_triggered.c1").as_double();
  alpha = this->get_parameter("gains.event_triggered.alpha").as_double();
  k_epsilon = this->get_parameter("gains.event_triggered.k_epsilon").as_double();

  /* Save the ID in the node */
  this->ID_ = static_cast<unsigned int>(ID);

  /* Compute the sqrt of the size of the adjacency matrix to check if it is square */
  int num_vehicles = static_cast<int>(std::sqrt(static_cast<double>(adj_matrix.size())));

  if (std::pow(num_vehicles, 2) != static_cast<int>(adj_matrix.size())) {
    throw std::invalid_argument("The adjacency Matrix in the configuration file is not square!");
  }

  /* Generate an Eigen Matrix from the vector that contains the adjacency matrix */
  this->adjency_matrix_.resize(num_vehicles, num_vehicles);

  for (int i = 0; i < num_vehicles; i++) {
    for (int j = 0; j < num_vehicles; j++) {
      this->adjency_matrix_(i, j) = adj_matrix[(i * num_vehicles) + j];
    }
  }

  /* Allocate a cooperative path following control node */
  return new EventTriggered(this->adjency_matrix_, ID, k_epsilon, c0, c1, alpha);
}


/**
 * @brief  Method to stop the current path following algorithm
 */
bool CpfNode::stop() {
  /* Stop the timer */
  this->timer_->cancel();

  /* Reset the auxiliary variables */
  this->gamma_ = 0.0;
  this->vd_ = 0.0;
  this->seq_ = 0;

  /* Call the reset method in the CPF algorithm */
  if (this->cooperative_) {
    this->cooperative_->reset();
  }

  return true;
}


/**
 * @brief  Method to initialize all the subscribers
 */
void CpfNode::initializeSubscribers() {
  RCLCPP_INFO(this->get_logger(), "Initializing Subscribers for CpfNode");

  this->declare_parameter<std::string>("topics.subscribers.internal_gamma", "/internal_gamma");
  this->declare_parameter<std::string>("topics.subscribers.external_gamma", "/external_gamma");

  std::string gamma_topic = this->get_parameter("topics.subscribers.internal_gamma").as_string();
  std::string external_gamma_topic = this->get_parameter("topics.subscribers.external_gamma").as_string();

  this->internal_gamma_sub_ = this->create_subscription<farol2_planning::msg::PathData>(
    gamma_topic, 10,
    [this](const farol2_planning::msg::PathData::SharedPtr msg) {
      this->internalInfoCallback(*msg);
    });

  this->external_gamma_sub_ = this->create_subscription<farol2_interfaces::msg::CPFGamma>(
    external_gamma_topic, 10,
    [this](const farol2_interfaces::msg::CPFGamma::SharedPtr msg) {
      this->externalInfoCallback(*msg);
    });
}

/**
 * @brief  Method to initialize all the publishers
 */
void CpfNode::initializePublishers() {
  RCLCPP_INFO(this->get_logger(), "Initializing Publishers for CpfNode");

  this->declare_parameter<std::string>("topics.publishers.vc", "/vc");
  this->declare_parameter<std::string>("topics.publishers.cpf_server_input", "/cpf_server_input");

  std::string vc_topic = this->get_parameter("topics.publishers.vc").as_string();
  std::string cpf_server_input_topic = this->get_parameter("topics.publishers.cpf_server_input").as_string();

  this->vc_pub_ = this->create_publisher<std_msgs::msg::Float64>(vc_topic, 1);
  this->cpf_server_pub_ = this->create_publisher<farol2_interfaces::msg::CPFGamma>(cpf_server_input_topic, 1);
}

/**
 * @brief  Method to create the timer that will do all the work
 */
void CpfNode::initializeTimer() {
  /* Get node frequency from parameters */
  this->declare_parameter<double>("node_frequency", 2.0);
  double node_frequency = this->get_parameter("node_frequency").as_double();
  RCLCPP_INFO(this->get_logger(), "Node will run at : %lf [hz]", node_frequency);

  /* Create timer */
  auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / node_frequency));
  this->timer_ = this->create_timer(period, [this]() { timerIterCallback(); });

  /* Wait for the start service to start the CPF */
  this->timer_->cancel();
}

/**
 * @brief  Callback for the timer interruption. Where all the logic of the algorithms
 * is executed with a fixed period
 */
void CpfNode::timerIterCallback() {
  double t = this->clock_->now().seconds();

  /* Run the coordination controller */
  double vc = this->cooperative_->coordinationController(t);

  /* Publish the correction factor to the VC topic */
  auto vc_msg = std_msgs::msg::Float64();
  vc_msg.data = vc;
  this->vc_pub_->publish(vc_msg);

  /* Check if it is time to publish the current gamma to the vehicle network */
  bool pub = this->cooperative_->publishCurrentGamma(t);

  if (pub) {
    /* Publish the current gamma to the network */
    farol2_interfaces::msg::CPFGamma msg;

    msg.header.stamp = this->clock_->now();
    msg.header.frame_id = "";

    msg.id = this->cooperative_->getCurrentVehicleID();
    msg.gamma = this->gamma_;
    msg.vd = this->vd_;

    this->cpf_server_pub_->publish(msg);
    this->seq_++;
  }
}

/**
 * @brief  Callback for receiving the external vehicle data
 */
void CpfNode::externalInfoCallback(const farol2_interfaces::msg::CPFGamma & msg) {
  double t = this->clock_->now().seconds();

  /* Interpret the message */
  unsigned int vehicle_ID = msg.id;
  double gamma = msg.gamma;
  double vd = msg.vd;

  /* Ignore an external message that contains data respective to our vehicle */
  if (vehicle_ID == this->ID_) {
    return;
  }

  /* Update the data inside the cooperative library */
  if (this->cooperative_) {
    this->cooperative_->updateVehiclesInformation(t, vehicle_ID, gamma, vd);
  }
}


/**
 * @brief  Callback for receiving the data from this vehicle
 */
void CpfNode::internalInfoCallback(const farol2_planning::msg::PathData & msg) {
  double t = this->clock_->now().seconds();

  /* Update the temporary variables */
  this->gamma_ = msg.gamma;
  this->vd_ = msg.vd;

  /* Update the data inside the cooperative library */
  if (this->cooperative_) {
    this->cooperative_->updateVehiclesInformation(t, this->ID_, this->gamma_, this->vd_);
  }
}

/**
 * @brief  The main function. The entry point for this ROS2 node
 */
int main(int argc, char ** argv) {
  /* Initialize ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CpfNode>());
  rclcpp::shutdown();

  return 0;
}

