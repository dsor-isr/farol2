#pragma once

#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include "CPFControl.h"
#include "EventTriggered.h"

#include "farol2_planning/msg/path_data.hpp"
#include "farol2_interfaces/msg/cpf_gamma.hpp"
#include "std_msgs/msg/float64.hpp"

#include "farol2_cpf_controller/srv/start_stop.hpp"
#include "farol2_cpf_controller/srv/change_topology.hpp"

/**
 * @brief ROS2 node to perform Cooperative Path Following (CPF)
 */
class CpfNode : public rclcpp::Node {
public:
  CpfNode();
  ~CpfNode();

private:
  /* CPF algorithm */
  CPFControl * cooperative_{nullptr};
  Eigen::MatrixXi adjency_matrix_;

  /* Temporary variables for this vehicle */
  double gamma_{0.0};
  double vd_{0.0};

  /* Vehicle ID and sequence counter */
  unsigned int ID_{0};
  unsigned int seq_{0};

  /* ROS2 interfaces */
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<farol2_planning::msg::PathData>::SharedPtr internal_gamma_sub_;
  rclcpp::Subscription<farol2_interfaces::msg::CPFGamma>::SharedPtr external_gamma_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vc_pub_;
  rclcpp::Publisher<farol2_interfaces::msg::CPFGamma>::SharedPtr cpf_server_pub_;

  rclcpp::Service<farol2_cpf_controller::srv::StartStop>::SharedPtr startCPF_srv_;
  rclcpp::Service<farol2_cpf_controller::srv::StartStop>::SharedPtr stopCPF_srv_;
  rclcpp::Service<farol2_cpf_controller::srv::ChangeTopology>::SharedPtr change_topology_srv_;

  /* Initialization helpers */
  void initializeSubscribers();
  void initializePublishers();
  void initializeServices();
  void initializeTimer();

  /* Create default CPF controller */
  CPFControl * createEventTriggeredControl();

  /* Stop/reset CPF */
  bool stop();

  /* Timer callback where the CPF logic is executed */
  void timerIterCallback();

  /* Message callbacks */
  void externalInfoCallback(const farol2_interfaces::msg::CPFGamma & msg);
  void internalInfoCallback(const farol2_planning::msg::PathData & msg);
  
  /* Service callbacks (kept signature style compatible with being called from lambdas) */
  bool StartService(farol2_cpf_controller::srv::StartStop::Request & req, farol2_cpf_controller::srv::StartStop::Response & res);
  bool StopService(farol2_cpf_controller::srv::StartStop::Request & req, farol2_cpf_controller::srv::StartStop::Response & res);
  bool ChangeTopologyService(farol2_cpf_controller::srv::ChangeTopology::Request & req, farol2_cpf_controller::srv::ChangeTopology::Response & res);
};