#include "CpfNode.hpp"


/**
 * @brief  Method to initialize all the services
 */
void CpfNode::initializeServices() {
  RCLCPP_INFO(this->get_logger(), "Initializing Services for CpfNode");

  /* Declare parameters and get service names */
  this->declare_parameter<std::string>("topics.services.start_cpf", "/start_cpf");
  this->declare_parameter<std::string>("topics.services.stop_cpf", "/stop_cpf");
  this->declare_parameter<std::string>("topics.services.change_topology", "/change_topology");

  std::string start_cpf_name = this->get_parameter("topics.services.start_cpf").as_string();
  std::string stop_cpf_name = this->get_parameter("topics.services.stop_cpf").as_string();
  std::string change_topology_name = this->get_parameter("topics.services.change_topology").as_string();

  /* Create ROS2 services */
  startCPF_srv_ = this->create_service<farol2_cpf_controller::srv::StartStop>(
    start_cpf_name,
    [this](const std::shared_ptr<rmw_request_id_t> /*req_header*/,
           const std::shared_ptr<farol2_cpf_controller::srv::StartStop::Request> req,
           std::shared_ptr<farol2_cpf_controller::srv::StartStop::Response> res) {
      (void)req;
      this->StartService(*req, *res);
    });

  stopCPF_srv_ = this->create_service<farol2_cpf_controller::srv::StartStop>(
    stop_cpf_name,
    [this](const std::shared_ptr<rmw_request_id_t> /*req_header*/,
           const std::shared_ptr<farol2_cpf_controller::srv::StartStop::Request> req,
           std::shared_ptr<farol2_cpf_controller::srv::StartStop::Response> res) {
      (void)req;
      this->StopService(*req, *res);
    });

  change_topology_srv_ = this->create_service<farol2_cpf_controller::srv::ChangeTopology>(
    change_topology_name,
    [this](const std::shared_ptr<rmw_request_id_t> /*req_header*/,
           const std::shared_ptr<farol2_cpf_controller::srv::ChangeTopology::Request> req,
           std::shared_ptr<farol2_cpf_controller::srv::ChangeTopology::Response> res) {
      this->ChangeTopologyService(*req, *res);
    });
}


/* Start Service callback */
bool CpfNode::StartService(farol2_cpf_controller::srv::StartStop::Request & /*req*/, farol2_cpf_controller::srv::StartStop::Response & res) {
  if (!timer_) {
    RCLCPP_ERROR(this->get_logger(), "Timer not initialized");
    res.success = false;
    return true;
  }

  /* If timer is already active, inform and return */
  if (!timer_->is_canceled()) {
    RCLCPP_INFO(this->get_logger(), "CPF is already running");
    res.success = true;
    return true;
  }

  /* Check and start timer */
  if (this->cooperative_ != nullptr) {
    RCLCPP_INFO(this->get_logger(), "CPF will start.");
    timer_->reset();
    res.success = true;
    gamma_ = 0.0;
    vd_ = 0.0;
  } else {
    RCLCPP_ERROR(this->get_logger(), "CPF algorithm is not instantiated. Restart this node");
    timer_->cancel();
    res.success = false;
  }

  return true;
}

/* Stop Service callback */
bool CpfNode::StopService(farol2_cpf_controller::srv::StartStop::Request & /*req*/, farol2_cpf_controller::srv::StartStop::Response & res) {
  if (timer_) {
    if (timer_->is_canceled()) {
      RCLCPP_INFO(this->get_logger(), "CPF was not running.");
    } else {
      RCLCPP_INFO(this->get_logger(), "CPF will stop.");
    }
  }

  /* Stop and reset CPF algorithm */
  this->stop();

  res.success = true;
  return true;
}


/* Service to change the topology of the network */
bool CpfNode::ChangeTopologyService(farol2_cpf_controller::srv::ChangeTopology::Request & req, farol2_cpf_controller::srv::ChangeTopology::Response & res) {
  int new_matrix_size = static_cast<int>(req.adjency_matrix.size());

  /* Check the size of the received adjency_matrix in the form of an std::vector */
  if (new_matrix_size != this->adjency_matrix_.rows() * this->adjency_matrix_.cols()) {
    RCLCPP_INFO(this->get_logger(), "New Matrix does not have the same size as the current Adjacency Matrix");
    res.success = false;
    return true;
  }

  /* Update adjacency matrix */
  for (int i = 0; i < this->adjency_matrix_.rows(); ++i) {
    for (int j = 0; j < this->adjency_matrix_.cols(); ++j) {
      this->adjency_matrix_(i, j) = req.adjency_matrix[(i * this->adjency_matrix_.rows()) + j];
    }
  }

  /* Update the value inside the CPF control class */
  if (this->cooperative_) {
    this->cooperative_->updateAdjencyMatrix(this->adjency_matrix_);
  }

  RCLCPP_INFO(this->get_logger(), "Updated new Adjacency Matrix successfully!");
  res.success = true;
  return true;
}

