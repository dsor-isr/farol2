#include "waypoint.hpp"

/* Constructor */
Waypoint::Waypoint() : Node("waypoint"){
  loadParams();
  initialiseSubscribers();
  initialiseServices();
  initialisePublishers();
  initialiseTimer();
}

/* Destructor */
Waypoint::~Waypoint() = default;

/**
 * @brief Load parameters
 */
void Waypoint::loadParams() {
  node_frequency_ = declare_parameter<double>("node_frequency");
  cdist_ = declare_parameter<double>("cdist");
  delta_t_ = declare_parameter<double>("delta_t");

  /* Waypoint type 1 gains */
  ku_ = declare_parameter<double>("type1.gains.ku");
  ks_ = declare_parameter<double>("type1.gains.ks");
  speed_turn_ = declare_parameter<double>("type1.gains.speed_turn");

  /* Waypoint type 2 (with heading) gains */
  k1_ = declare_parameter<double>("type2.gains.k1");
  k2_ = declare_parameter<double>("type2.gains.k2");
  k3_ = declare_parameter<double>("type2.gains.k3");
}

/**
 * @brief Initialise Subscribers
 */
void Waypoint::initialiseSubscribers() {
  mission_status_sub_ = create_subscription<std_msgs::msg::Int8>(
    declare_parameter<std::string>("topics.subscribers.mission_status"),
    rclcpp::QoS(1),
    [this](std_msgs::msg::Int8::SharedPtr msg){missionStatusCallback(msg);});
  
  state_sub_ = create_subscription<farol2_interfaces::msg::NavigationState>(
    declare_parameter<std::string>("topics.subscribers.state"),
    rclcpp::QoS(1),
    [this](farol2_interfaces::msg::NavigationState::SharedPtr msg){stateCallback(msg);});

  turn_radius_flag_sub_ = create_subscription<std_msgs::msg::Bool>(
    declare_parameter<std::string>("topics.subscribers.turn_radius_flag"),
    rclcpp::QoS(1),
    [this](std_msgs::msg::Bool::SharedPtr msg){turnRadiusFlagCallback(msg);});
}

/**
 * @brief Initialise Publishers
 */
void Waypoint::initialisePublishers() {
  yaw_ref_pub_ = create_publisher<std_msgs::msg::Float32>(
    declare_parameter<std::string>("topics.publishers.yaw_ref"),
    rclcpp::QoS(1));
  yaw_rate_ref_pub_ = create_publisher<std_msgs::msg::Float32>(
    declare_parameter<std::string>("topics.publishers.yaw_rate_ref"),
    rclcpp::QoS(1));
  u_ref_pub_ = create_publisher<std_msgs::msg::Float32>(
    declare_parameter<std::string>("topics.publishers.u_ref"),
    rclcpp::QoS(1));
  v_ref_pub_ = create_publisher<std_msgs::msg::Float32>(
    declare_parameter<std::string>("topics.publishers.v_ref"),
    rclcpp::QoS(1));
  mission_status_pub_ = create_publisher<std_msgs::msg::Int8>(
    declare_parameter<std::string>("topics.publishers.mission_status"),
    rclcpp::QoS(1));
}

/**
 * @brief Initialise Services
 */
void Waypoint::initialiseServices() {
  /* Service servers */
  wp_standard_srv_ = create_service<waypoint::srv::SendWpType1>(
    declare_parameter<std::string>("topics.services.wp_standard"),
    [this](const std::shared_ptr<waypoint::srv::SendWpType1::Request> request,
      std::shared_ptr<waypoint::srv::SendWpType1::Response> response){
      this->sendWpStandardService(request, response);
    });

  wp_loose_srv_ = create_service<waypoint::srv::SendWpType1>(
    declare_parameter<std::string>("topics.services.wp_loose"),
    [this](const std::shared_ptr<waypoint::srv::SendWpType1::Request> request,
      std::shared_ptr<waypoint::srv::SendWpType1::Response> response){
      this->sendWpLooseService(request, response);
    });
  
  wp_heading_srv_ = create_service<waypoint::srv::SendWpType1>(
    declare_parameter<std::string>("topics.services.wp_heading"),
    [this](const std::shared_ptr<waypoint::srv::SendWpType1::Request> request,
      std::shared_ptr<waypoint::srv::SendWpType1::Response> response){
      this->sendWpHeadingService(request, response);
    });
}

/**
 * @brief Initialise Timer
 */
void Waypoint::initialiseTimer() {
  auto period = std::chrono::nanoseconds( static_cast<int64_t>(1e9 / node_frequency_));
  timer_ = create_timer(period, [this]() {timerCallback();});

  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void Waypoint::timerCallback() {
  // compute waypoint controller and publish
  wp_controller_->compute(veh_state_, wp_ref_, turn_radius_flag_);
}

void Waypoint::stateCallback(farol2_interfaces::msg::NavigationState::SharedPtr msg) {
  // update vehicle state
  veh_state_.eta1[0] = msg->utm_position.northing;
  veh_state_.eta1[1] = msg->utm_position.easting;
  veh_state_.eta2[2] = msg->orientation.z;
  veh_state_.v1[0] = msg->body_velocity_inertial.x;
  veh_state_.v1[1] = msg->body_velocity_inertial.y;

  // // send message if error and stop timer
  // if (!(msg->status & msg->STATUS_ALL_OK) && !timer_->is_canceled()) {
  //   RCLCPP_ERROR(get_logger(), "The filter estimate is not good, disabling WayPoint");
  //   timer_->cancel();
  // }
}

void Waypoint::missionStatusCallback(std_msgs::msg::Int8::SharedPtr msg) {
  // stop the waypoint controller if the mission status has been changed to other value
  // than 4
  if (!timer_->is_canceled() && msg->data != 4) {
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Some process changed the mission status to %d", msg->data);
  }
}

void Waypoint::turnRadiusFlagCallback(std_msgs::msg::Bool::SharedPtr msg) {
  turn_radius_flag_ = msg->data;
}

void Waypoint::createWaypoint(WaypointController *new_wp) {
  // free memory from waypoint controller pointer and point it to new controller
  // if (wp_controller_) {
  //   free(wp_controller_);
  // }
  wp_controller_ = new_wp;
}

bool Waypoint::decodeWaypoint(double x, double y) {
  Eigen::Vector2d ref_return;
  wp_ref_.eta1[0] = x;
  wp_ref_.eta1[1] = y;
  if (x == -1 && y == -1) {               // Hold in the same position
    wp_ref_.eta1[0] = veh_state_.eta1[0]; // Actual GPS position
    wp_ref_.eta1[1] = veh_state_.eta1[1]; // Actual GPS position
  }

  else if (x == -3 && y == -3) {
    wp_ref_.eta1[0] = veh_state_.eta1[0] +
                      veh_state_.v1[0] *
                          cos(veh_state_.eta2[2] * (M_PI / 180)) *
                          delta_t_;
    wp_ref_.eta1[1] = veh_state_.eta1[1] +
                      veh_state_.v1[0] *
                          sin(veh_state_.eta2[2] * (M_PI / 180)) *
                          delta_t_;
  }
  // Verify the stop condition
  else if (x == -2 && y == -2) {
    return false;
  }
  return true;
}

void Waypoint::sendWpStandardService(const std::shared_ptr<waypoint::srv::SendWpType1::Request> req,
                                     std::shared_ptr<waypoint::srv::SendWpType1::Response> res) {

  RCLCPP_INFO(get_logger(), "Sending Waypoint");

  // create pointer to new controller
  WaypointController *aux_wp = new WpStandard(u_ref_pub_, yaw_ref_pub_);
  // set the gains
  aux_wp->setGains(std::vector<double>{cdist_, ku_, ks_});
  // substitute node pointer of the controller
  createWaypoint(aux_wp);

  // set the new waypoint reference, change the flag value and start the main
  // loop
  if (!decodeWaypoint(req->x, req->y)) {
    res->message += "Stop signal sent";

    mission_status_msg_.data = 0;
    mission_status_pub_->publish(mission_status_msg_);

    timer_->cancel();
  } else {
    res->success = true;
    res->message += "New waypoint reference: (" +
                   std::to_string(wp_ref_.eta1[0]) + "," +
                   std::to_string(wp_ref_.eta1[1]) + ")";
    
    mission_status_msg_.data = 4;
    mission_status_pub_->publish(mission_status_msg_);

    if (timer_->is_canceled()) {
      timer_->reset();
    }
  }
}

void Waypoint::sendWpLooseService(const std::shared_ptr<waypoint::srv::SendWpType1::Request> req,
                                  std::shared_ptr<waypoint::srv::SendWpType1::Response> res){
  RCLCPP_INFO(get_logger(), "Sending Waypoint");
  // create pointer to new controller
  WaypointController *aux_wp = new WpLoose(u_ref_pub_, yaw_ref_pub_);
  // set the gains
  aux_wp->setGains(std::vector<double>{cdist_, ku_, ks_, speed_turn_});
  aux_wp->setFrequency(node_frequency_);
  // substitute node pointer of the controller
  createWaypoint(aux_wp);

  // set the new waypoint reference, change the flag value and start the main
  // loop
  if (!decodeWaypoint(req->x, req->y)) {
    res->message += "Stop signal sent";
    
    mission_status_msg_.data = 0;
    mission_status_pub_->publish(mission_status_msg_);

    timer_->cancel();
  } else {
    res->success = true;
    res->message += "New waypoint reference: (" +
                   std::to_string(wp_ref_.eta1[0]) + "," +
                   std::to_string(wp_ref_.eta1[1]) + ")";
    
    mission_status_msg_.data = 4;
    mission_status_pub_->publish(mission_status_msg_);

    if (timer_->is_canceled()) {
      timer_->reset();
    }
  }
}

void Waypoint::sendWpHeadingService(const std::shared_ptr<waypoint::srv::SendWpType1::Request> req,
                                    std::shared_ptr<waypoint::srv::SendWpType1::Response> res) {
  RCLCPP_INFO(get_logger(), "Sending Waypoint");
  // create pointer to new controller
  WaypointController *aux_wp =
      new WpHeading(u_ref_pub_, v_ref_pub_, yaw_rate_ref_pub_);
  // set the gains
  aux_wp->setGains(std::vector<double>{cdist_, k1_, k2_, k3_});
  aux_wp->setFrequency(node_frequency_);
  // substitute node pointer of the controller
  createWaypoint(aux_wp);
  // set the new waypoint reference, change the flag value and start the main
  // loop
  if (!decodeWaypoint(req->x, req->y)) {
    res->message += "Stop signal sent";
    
    mission_status_msg_.data = 0;
    mission_status_pub_->publish(mission_status_msg_);

    timer_->cancel();
  } else {
    // add the yaw reference here since the wp logic (decodeWaypoint) doesnt
    // include it
    wp_ref_.eta2[2] = req->yaw;
    res->success = true;
    res->message += "New waypoint reference: (" +
                   std::to_string(wp_ref_.eta1[0]) + "," +
                   std::to_string(wp_ref_.eta1[1]) + "," + std::to_string(wp_ref_.eta2[2]) + ")";
    
    mission_status_msg_.data = 4;
    mission_status_pub_->publish(mission_status_msg_);

    if (timer_->is_canceled()) {
      timer_->reset();
    }
  }
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Waypoint>());
  rclcpp::shutdown();
  return 0;
}