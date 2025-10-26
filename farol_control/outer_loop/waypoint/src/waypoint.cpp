#include "waypoint.hpp"

/* Constructor */
Waypoint::Waypoint() : Node("waypoint", 
                                    rclcpp::NodeOptions()
                                      .allow_undeclared_parameters(true)
                                      .automatically_declare_parameters_from_overrides(true)) {
  
  loadParams();
  initialiseSubscribers();
  initialiseServices();
  initialisePublishers();
  initialiseTimer();
}

/* Destructor */
Waypoint::~Waypoint() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void Waypoint::loadParams() {
  cdist_ = get_parameter("control.outer_loop.waypoint.cdist").as_double();
  delta_t_ = get_parameter("control.outer_loop.waypoint.delta_t").as_double();

  /* Waypoint type 1 gains */
  ku_ = get_parameter("control.outer_loop.waypoint.type1.gains.ku").as_double();
  ks_ = get_parameter("control.outer_loop.waypoint.type1.gains.ks").as_double();
  speed_turn_ = get_parameter("control.outer_loop.waypoint.type1.gains.speed_turn").as_double();

  /* Waypoint type 2 (with heading) gains */
  k1_ = get_parameter("control.outer_loop.waypoint.type2.gains.k1").as_double();
  k2_ = get_parameter("control.outer_loop.waypoint.type2.gains.k2").as_double();
  k3_ = get_parameter("control.outer_loop.waypoint.type2.gains.k3").as_double();
}

/**
 * @brief Initialise Subscribers
 */
void Waypoint::initialiseSubscribers() {
  mission_status_sub_ = create_subscription<std_msgs::msg::Int8>(
                          get_parameter("control.outer_loop.waypoint.topics.subscribers.mission_status").as_string(), 
                          1, std::bind(&Waypoint::missionStatusCallback, this, std::placeholders::_1));

  state_sub_ = create_subscription<farol_msgs::msg::NavigationState>(
                get_parameter("control.outer_loop.waypoint.topics.subscribers.state").as_string(), 
                1, std::bind(&Waypoint::stateCallback, this, std::placeholders::_1));

  turn_radius_flag_sub_ = create_subscription<std_msgs::msg::Bool>(
                            get_parameter("control.outer_loop.waypoint.topics.subscribers.turn_radius_flag").as_string(), 
                            1, std::bind(&Waypoint::turnRadiusFlagCallback, this, std::placeholders::_1));
}

/**
 * @brief Initialise Publishers
 */
void Waypoint::initialisePublishers() {
  yaw_ref_pub_ = create_publisher<std_msgs::msg::Float32>(
                  get_parameter("control.outer_loop.waypoint.topics.publishers.yaw_ref").as_string(), 1);

  yaw_rate_ref_pub_ = create_publisher<std_msgs::msg::Float32>(
                        get_parameter("control.outer_loop.waypoint.topics.publishers.yaw_rate_ref").as_string(), 1);

  u_ref_pub_ = create_publisher<std_msgs::msg::Float32>(
                get_parameter("control.outer_loop.waypoint.topics.publishers.u_ref").as_string(), 1);

  v_ref_pub_ = create_publisher<std_msgs::msg::Float32>(
                get_parameter("control.outer_loop.waypoint.topics.publishers.v_ref").as_string(), 1);

  mission_status_pub_ = create_publisher<std_msgs::msg::Int8>(
                          get_parameter("control.outer_loop.waypoint.topics.publishers.mission_status").as_string(), 1);
}

/**
 * @brief Initialise Services
 */
void Waypoint::initialiseServices() {
  /* Service servers */
  /* Service to change current filter */
  wp_standard_srv_ = create_service<waypoint::srv::SendWpType1>(
                      get_parameter("control.outer_loop.waypoint.topics.services.wp_standard").as_string(),
                      std::bind(&Waypoint::sendWpStandardService, this, std::placeholders::_1, std::placeholders::_2));

  wp_loose_srv_ = create_service<waypoint::srv::SendWpType1>(
                    get_parameter("control.outer_loop.waypoint.topics.services.wp_loose").as_string(),
                    std::bind(&Waypoint::sendWpLooseService, this, std::placeholders::_1, std::placeholders::_2));

  wp_heading_srv_ = create_service<waypoint::srv::SendWpType1>(
                      get_parameter("control.outer_loop.waypoint.topics.services.wp_heading").as_string(),
                      std::bind(&Waypoint::sendWpHeadingService, this, std::placeholders::_1, std::placeholders::_2));
  
  /* Service clients */
  /*... */

  return;
}

/**
 * @brief Initialise Timer
 */
void Waypoint::initialiseTimer() {
  /* Get node frequency from parameters */
  int freq = get_parameter("control.outer_loop.waypoint.node_frequency").as_int();
  node_frequency_ = (double)freq;

  /* Create timer */
  timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq*1000)), std::bind(&Waypoint::timerCallback, this));

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

void Waypoint::stateCallback(const farol_msgs::msg::NavigationState &msg) {
  // update vehicle state
  veh_state_.eta1[0] = msg.utm_position.northing;
  veh_state_.eta1[1] = msg.utm_position.easting;
  veh_state_.eta2[2] = msg.orientation.z;
  veh_state_.v1[0] = msg.body_velocity_inertial.x;
  veh_state_.v1[1] = msg.body_velocity_inertial.y;

  // // send message if error and stop timer
  // if (!(msg.status & msg.STATUS_ALL_OK) && timer_->is_ready()) {
  //   RCLCPP_ERROR(get_logger(), "The filter estimate is not good, disabling WayPoint");
  //   timer_->cancel();
  // }
}

void Waypoint::missionStatusCallback(const std_msgs::msg::Int8 &msg) {
  // stop the waypoint controller if the mission status has been changed to other value
  // than 4
  if (timer_->is_ready() && msg.data != 4) {
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Some process changed the mission status to %d", msg.data);
  }
}

void Waypoint::turnRadiusFlagCallback(const std_msgs::msg::Bool &msg) {
  turn_radius_flag_ = msg.data;
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