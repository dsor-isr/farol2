#include "CpfNode.hpp"


/**
 * @brief  Method to initialize all the services
 */
void CpfNode::initializeServices() {
  RCLCPP_INFO(this->get_logger(), "Initializing Services for CpfNode");

  this->startCPF_srv_ = create_service<farol2_cpf_controller::srv::StartStop>(
    SERVICE_START_CPF,
    std::bind(&CpfNode::StartService, this, std::placeholders::_1, std::placeholders::_2));

  this->stopCPF_srv_ = create_service<farol2_cpf_controller::srv::StartStop>(
    SERVICE_STOP_CPF,
    std::bind(&CpfNode::StopService, this, std::placeholders::_1, std::placeholders::_2));

  this->change_topology_srv_ = create_service<farol2_cpf_controller::srv::ChangeTopology>(
    SERVICE_CHANGE_TOPOLOGY,
    std::bind(&CpfNode::ChangeTopologyService, this, std::placeholders::_1, std::placeholders::_2));
}


/* Start Service callback */
bool CpfNode::StartService(const std::shared_ptr<farol2_cpf_controller::srv::StartStop::Request> req,
                                       std::shared_ptr<farol2_cpf_controller::srv::StartStop::Response> res) {
  if (!timer_) {
    RCLCPP_ERROR(this->get_logger(), "Timer not initialized");
    res->success = false;
    return true;
  }

  /* If timer is already active, inform and return */
  if (!timer_->is_canceled()) {
    RCLCPP_INFO(this->get_logger(), "CPF is already running");
    res->success = true;
    return true;
  }

  /* Check and start timer */
  if (this->cooperative_ != nullptr) {
    RCLCPP_INFO(this->get_logger(), "CPF will start.");
    timer_->reset();
    res->success = true;
    gamma_ = 0.0;
    vd_ = 0.0;
  } else {
    RCLCPP_ERROR(this->get_logger(), "CPF algorithm is not instantiated. Restart this node");
    timer_->cancel();
    res->success = false;
  }

  return true;
}

/* Stop Service callback */
bool CpfNode::StopService(const std::shared_ptr<farol2_cpf_controller::srv::StartStop::Request> req,
                                      std::shared_ptr<farol2_cpf_controller::srv::StartStop::Response> res) {
  if (timer_) {
    if (timer_->is_canceled()) {
      RCLCPP_INFO(this->get_logger(), "CPF was not running.");
    } else {
      RCLCPP_INFO(this->get_logger(), "CPF will stop.");
    }
  }

  /* Stop and reset CPF algorithm */
  this->stop();

  res->success = true;
  return true;
}


/* Service to change the topology of the network */
bool CpfNode::ChangeTopologyService(const std::shared_ptr<farol2_cpf_controller::srv::ChangeTopology::Request> req,
                                           std::shared_ptr<farol2_cpf_controller::srv::ChangeTopology::Response> res) {
  int new_matrix_size = static_cast<int>(req->adjency_matrix.size());

  /* Check the size of the received adjency_matrix in the form of an std::vector */
  if (new_matrix_size != this->adjency_matrix_.rows() * this->adjency_matrix_.cols()) {
    RCLCPP_INFO(this->get_logger(), "New Matrix does not have the same size as the current Adjacency Matrix");
    res->success = false;
    return true;
  }

  /* Update adjacency matrix */
  for (int i = 0; i < this->adjency_matrix_.rows(); ++i) {
    for (int j = 0; j < this->adjency_matrix_.cols(); ++j) {
      this->adjency_matrix_(i, j) = req->adjency_matrix[(i * this->adjency_matrix_.rows()) + j];
    }
  }

  /* Update the value inside the CPF control class */
  if (this->cooperative_) {
    this->cooperative_->updateAdjencyMatrix(this->adjency_matrix_);
  }

  RCLCPP_INFO(this->get_logger(), "Updated new Adjacency Matrix successfully!");
  res->success = true;
  return true;
}

