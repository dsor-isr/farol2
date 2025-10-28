#include "PathFollowingNode.h"
/* Contains auxiliary functions for angle wrap */
#include "utils.hpp"

PathFollowingNode::PathFollowingNode() : Node("path_following", 
                                              rclcpp::NodeOptions()
                                                .allow_undeclared_parameters(true)
                                                .automatically_declare_parameters_from_overrides(true)) {
  this->initialiseSubscribers();
  this->initialisePublishers();
  /* NOTE: initialiseServices is implemented inside PathFollowingServices.cpp */
  this->initialiseServices();
  this->initialiseTimer();

  /* Allocate memory for the default Path Following Algorithm - Breivik */
  this->pf_algorithm_ = getDefaultControllerBreivik();

  /* Set PF Debug publisher */
  pf_algorithm_->setPFollowingDebugPublisher(
    create_publisher<farol_msgs::msg::PFDebug>(
      get_parameter("control.outer_loop.path_following.topics.publishers.pfollowing_debug").as_string(), 1)
  );
}

/**
 * @brief  Node class destructor
 */
PathFollowingNode::~PathFollowingNode() {

  /* Shutdown all the publishers and deleting the memory allocated for the PF
   * controller */
  this->deleteCurrentController();
  
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief  Alocates memory for the default controller to be used. In this case
 * is Lapierre
 */
PathFollowing *PathFollowingNode::getDefaultControllerLapierre() {
  /* Create the publishers for the node */
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));

  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.yaw_rate").as_string(), 1));

  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.rabbit").as_string(), 1));

  /* Read the gains for the controller */
  double k1, k2, k3, theta, k_delta;
  
  k1 = get_parameter("control.outer_loop.path_following.controller_gains.lapierre.k1").as_double();
  k2 = get_parameter("control.outer_loop.path_following.controller_gains.lapierre.k2").as_double();
  k3 = get_parameter("control.outer_loop.path_following.controller_gains.lapierre.k3").as_double();
  theta = get_parameter("control.outer_loop.path_following.controller_gains.lapierre.theta").as_double();
  k_delta = get_parameter("control.outer_loop.path_following.controller_gains.lapierre.k_delta").as_double();

  /* Return the Path Following object */
  return new Lapierre(k1, k2, k3, theta, k_delta,
                      this->publishers_[0], this->publishers_[1], this->publishers_[2]);
}

/**
 * @brief Alocates memory for a default controller. In this case is Breivik
 */
PathFollowing *PathFollowingNode::getDefaultControllerBreivik() {
  /* Create the publishers for the node */
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));

  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.yaw").as_string(), 1));

  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.rabbit").as_string(), 1));

  /* Read the gains for the controller */
  double delta_h;

  delta_h = get_parameter("control.outer_loop.path_following.controller_gains.breivik.delta_h").as_double();
 
  /* Assign the new controller */
  return new Breivik(this->publishers_[0], this->publishers_[1], this->publishers_[2], delta_h);
}

PathFollowing *PathFollowingNode::getDefaultControllerAguiar() {
  /* Create the publishers for the node */
  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.surge").as_string(), 1));

  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.yaw_rate").as_string(), 1));

  this->publishers_.push_back(create_publisher<std_msgs::msg::Float32>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.rabbit").as_string(), 1));

  double delta, kz;
  double kk[2];
  double k_pos;
  double k_currents;

  /* Read the gains for the controller */
  delta = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.delta").as_double();
  kk[0] = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.kx").as_double();
  kk[1] = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.ky").as_double();
  kz = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.kz").as_double();
  k_pos = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.k_pos").as_double();
  k_currents = get_parameter("control.outer_loop.path_following.controller_gains.aguiar.k_currents").as_double();

  /* Assign the new controller */
  return new Aguiar(delta, kk, kz, k_pos, k_currents, 
                    this->publishers_[0], this->publishers_[1], this->publishers_[2]);
}

/**
 * @brief  Deletes safely the current path following object
 */
void PathFollowingNode::deleteCurrentController() {

  /* Delete the path following object */
  if (this->pf_algorithm_) {
    delete this->pf_algorithm_;
    this->pf_algorithm_ = nullptr;
  }

  /* Empty the list of publishers */
  this->publishers_.clear();
}

/**
 * @brief  Initialise all the subscribers
 */
void PathFollowingNode::initialiseSubscribers() {

  this->state_sub_ = create_subscription<farol_msgs::msg::NavigationState>(
                      get_parameter("control.outer_loop.path_following.topics.subscribers.state").as_string(), 
                      1, std::bind(&PathFollowingNode::vehicleStateCallback, this, std::placeholders::_1));

  this->path_data_sub_ = create_subscription<paths::msg::PathData>(
                          get_parameter("control.outer_loop.path_following.topics.subscribers.path_data").as_string(), 
                          1, std::bind(&PathFollowingNode::pathStateCallback, this, std::placeholders::_1));

  this->vc_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("control.outer_loop.path_following.topics.subscribers.vc").as_string(), 
                    1, std::bind(&PathFollowingNode::vcCallback, this, std::placeholders::_1));

  this->mission_status_sub_ = create_subscription<std_msgs::msg::Int8>(
                                get_parameter("control.outer_loop.path_following.topics.subscribers.mission_status").as_string(), 
                                1, std::bind(&PathFollowingNode::missionStatusCallback, this, std::placeholders::_1));
}

/**
 * @brief  Method to initialise the publishers of the path following node
 */
void PathFollowingNode::initialisePublishers() {

  this->mission_status_pub_ = create_publisher<std_msgs::msg::Int8>(
                                get_parameter("control.outer_loop.path_following.topics.publishers.mission_status").as_string(), 1);
}

/**
 * @brief  Vc callback used to update the velocity correction for cooperative
 * path following
 *
 * @param msg  A pointer to an std_msgs::msg::Float32 with the synchronization speed
 * for the virtual target gamma
 */
void PathFollowingNode::vcCallback(const std_msgs::msg::Float32 &msg) {

  /* Update the desired synchronization correction term */
  this->path_state_.vc = msg.data;
}

/**
 * @brief  Path state callback to update the path data
 *
 * @param msg  A pointer to a dsor_paths::PathData that contains information of
 * the path
 */
void PathFollowingNode::pathStateCallback(const paths::msg::PathData &msg) {

  /* If the algorithm is running, signal that we have received data from the
   * path */
  if (!this->timer_->is_canceled()) {
    has_received_path_state = true;
  }

  /* Update the gamma used to make the computations */
  this->path_state_.gamma = msg.gamma;

  /* Update the path position, derivative and second derivative */
  this->path_state_.pd << msg.pd[0], msg.pd[1], msg.pd[2];
  this->path_state_.d_pd << msg.d_pd[0], msg.d_pd[1], msg.d_pd[2];
  this->path_state_.dd_pd << msg.dd_pd[0], msg.dd_pd[1], msg.dd_pd[2];

  /* Update the angle of the tangent to the path */
  this->path_state_.psi = msg.tangent;
  this->path_state_.curvature = msg.curvature;
  this->path_state_.tangent_norm = msg.derivative_norm;

  /* The desired velocity is the combination of the path speed + coordination
   * speed */
  this->path_state_.vd = msg.vd;
  this->path_state_.d_vd = msg.d_vd;
  this->path_state_.vehicle_speed = msg.vehicle_speed;

  /* Update the bounds of the path following path parameter gamma */
  this->path_state_.gamma_min = msg.gamma_min;
  this->path_state_.gamma_max = msg.gamma_max;
}

/**
 * @brief  Vehicle state callback to update the state of the vehicle
 *
 * @param msg  A pointer to a farol_msgs::msg::NavigationState that contains
 * information regarding the vehicle
 */
void PathFollowingNode::vehicleStateCallback(const farol_msgs::msg::NavigationState &msg) {

  /* If the algorithm is running, signal that we have received data from the
   * vehicle state */
  if (!this->timer_->is_canceled()) {
    has_received_vehicle_state = true;
  }

  /* Update the vehicle position */
  this->vehicle_state_.eta1 << msg.utm_position.northing,
                               msg.utm_position.easting,
                               msg.altimeter;

  /* Update the vehicle orientation */
  double roll = FarolUtils::wrapToPi(msg.orientation.x);
  double pitch = FarolUtils::wrapToPi(msg.orientation.y);
  double yaw = FarolUtils::wrapToPi(msg.orientation.z);
  this->vehicle_state_.eta2 << roll, pitch, yaw;

  /* Update the vehicle linear velocity */
  this->vehicle_state_.v1 << msg.body_velocity_inertial.x,
                             msg.body_velocity_inertial.y,
                             msg.body_velocity_inertial.z;

  /* Update the vehicle angular velocity */
  this->vehicle_state_.v2 << msg.orientation_rate.x,
                             msg.orientation_rate.y, 
                             msg.orientation_rate.z;
}

/**
 * @brief Initialise Timer
 */
void PathFollowingNode::initialiseTimer() {
  /* Get node frequency from parameters */
  int freq = get_parameter("control.outer_loop.path_following.node_frequency").as_int();

  /* Create timer */
  this->timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq*1000)), std::bind(&PathFollowingNode::timerIterCallback, this));

  /* Wait for the start service to start the Path Following */
  this->timer_->cancel();
}

/**
 * @brief Method where the logic is located in order to update the control law
 */
void PathFollowingNode::timerIterCallback() {
  /* If it has not received data from the path or the vehicle, do not update the
   * algorithm */
  if (!this->has_received_path_state || !this->has_received_vehicle_state)
    return;

  /* Update the values inside the PathFollowing controller */
  this->pf_algorithm_->UpdateVehicleState(this->vehicle_state_);
  this->pf_algorithm_->UpdatePathState(this->path_state_);

  /* Get the difference between previous update time and current update time */
  rclcpp::Time curr_time = clock_.now();
  rclcpp::Duration dt = curr_time - this->prev_time_;
  this->prev_time_ = curr_time;

  /* Compute the control law */
  this->pf_algorithm_->callPFController(double(dt.seconds()));

  /* Publish the results */
  this->pf_algorithm_->publish();

  /* Check if we have reached the end of the path */
  if (this->pf_algorithm_->stop()) {
    /* Ask waypoint algorithm to hold position */
    this->sendWaypoint(WP_FINISH);
    /* Reset the DR postion to the 2d state filter position */
    this->sendResetDeadReckoning();

  }
}

/**
 * @brief  Auxiliar method to send a hold position waypoint
 *
 * @param value -3 to hold the pos after the vehicle, -1 to hold position on the spot
 */
void PathFollowingNode::sendWaypoint(double value) {
  std::shared_ptr<waypoint::srv::SendWpType1::Request> req = std::make_shared<waypoint::srv::SendWpType1::Request>();
  
  req->x = value;
  req->y = value;

  wp_standard_client_->async_send_request(req);
}

/**
 * @brief  Auxiliar method to reset the DeadReckoning postion  
 * */
void PathFollowingNode::sendResetDeadReckoning() {
  std::shared_ptr<std_srvs::srv::Trigger::Request> req = std::make_shared<std_srvs::srv::Trigger::Request>();

  dr_reset_client_->async_send_request(req);
}

/**
 * @brief  Auxiliar method to stop the algorithm
 */
void PathFollowingNode::stopAlgorithm() {

  /* Inform the user that path following will stop */
  RCLCPP_INFO(get_logger(), "Path Following Node has stopped.");

  /* Stop the timer callback where the pf algorithm is invoked */
  this->timer_->cancel();

  /* Reset the Path following controller */
  this->pf_algorithm_->reset();

  /* Reset the flags used in the first iteration */
  this->has_received_vehicle_state = false;
  this->has_received_path_state = false;

  /* Call the service to reset the path */
  std::shared_ptr<paths::srv::ResetPath::Request> req = std::make_shared<paths::srv::ResetPath::Request>();
  req->reset_path = true;
  reset_path_client_->async_send_request(req);
}

/**
 * @brief  Auxiliar method to stop the algorithm
 */
void PathFollowingNode::missionStatusCallback(const std_msgs::msg::Int8 &msg) {

  /* If the flag changed and the path following was running, then stop */
  if (msg.data != FLAG_PF && !this->timer_->is_canceled()) {
    this->stopAlgorithm();
  }
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowingNode>());
  rclcpp::shutdown();
  return 0;
}
