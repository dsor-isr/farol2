#include <filter_handler.hpp>

/* Constructor */
FilterHandler::FilterHandler() : Node("filter_handler", 
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
FilterHandler::~FilterHandler() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Initialise Subscribers
 */
void FilterHandler::initialiseSubscribers() {
  /* If no ros topics are found for the node's subscribers in the parameters */
  if (!get_node_parameters_interface()->get_parameters_by_prefix("nav.filter_handler.topics.subscribers", subscription_topic_map_)) {
    RCLCPP_ERROR(get_logger(), "No subscribers were found.");
    rclcpp::shutdown();
  }

  /* Create subscription to each different filter state based on topic names from ros parameters */
  for (const auto & [key, param] : subscription_topic_map_) {
    /* Create subscription using param as topic */
    rclcpp::Subscription<farol_msgs::msg::NavigationState>::SharedPtr sub = 
      create_subscription<farol_msgs::msg::NavigationState>(
        param.as_string(),
        1, 
        [this, key](const farol_msgs::msg::NavigationState::SharedPtr msg) {
          nav_filter_callback(*msg, key);
        });

    /* Insert new entry to subscription_map_ with same key and ros subscription */
    subscription_map_.insert({key, sub});

    /* Initialise nav_state_map_ with default nav state messages */
    farol_msgs::msg::NavigationState msg;
    nav_state_map_.insert({key, msg});
  }

  return;
}

/**
 * @brief Load parameters
 */
void FilterHandler::loadParams() {
  /* Get list of available filters */
  filters_ = get_parameter("nav.filter_handler.filters").as_string_array();

  /* Check if any of the available filters does not have a corresponding topic with its state */
  for (std::string filter : filters_) {
    if (subscription_topic_map_.find(filter) == subscription_topic_map_.end()) {
      RCLCPP_ERROR(get_logger(), "Missing topic in ros.yaml for %s filter.", filter.c_str());
      rclcpp::shutdown();
    }
  }

  /* Get default filter name */
  current_filter_ = get_parameter("nav.filter_handler.default_filter").as_string();

  /* Check if default filter is one of the available filters */
  if (std::find(filters_.begin(), filters_.end(), current_filter_) == filters_.end()) {
    RCLCPP_ERROR(get_logger(), "Default filter set in nav.yaml is not one of the available filters.");
    rclcpp::shutdown();
  }

  return;
}

/**
 * @brief Initialise Publishers
 */
void FilterHandler::initialisePublishers() {
  state_pub_ = create_publisher<farol_msgs::msg::NavigationState>(
                get_parameter("nav.filter_handler.topics.publishers.state").as_string(), 1);
}

/**
 * @brief Initialise Services
 */
void FilterHandler::initialiseServices() {
  /* Service servers */
  /* Service to change current filter */
  change_filter_srv_ = create_service<nav_filters::srv::ChangeFilter>(
                        get_parameter("nav.filter_handler.topics.services.change_filter").as_string(),
                        std::bind(&FilterHandler::changeFilterCallback, this, std::placeholders::_1, std::placeholders::_2));

  /* service clients */
  /*... */

  return;
}

/**
 * @brief Initialise Timers
 */
void FilterHandler::initialiseTimers() {
  /* Get node frequency from parameters */
  int freq = get_parameter("nav.filter_handler.node_frequency").as_int();

  /* Create timer */
  timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq*1000)), std::bind(&FilterHandler::timerCallback, this));
}

/**
 * @brief Nav filter callback.
 *        Callback for subscriptions to each navigation filter.
 */
void FilterHandler::nav_filter_callback(const farol_msgs::msg::NavigationState &msg, std::string filter_key) {
  /* Update nav state for the correct filter */
  nav_state_map_[filter_key] = msg;

  /* Publish message if it corresponds to the current filter */
  if (filter_key != current_filter_) return;

  /* Update nav filter message with the current filter's */
  filter_state_msg_ = msg;

  /* Publish filter state message */
  state_pub_->publish(filter_state_msg_);
}

/**
 * @brief Change filter callback.
 */
void FilterHandler::changeFilterCallback(const std::shared_ptr<nav_filters::srv::ChangeFilter::Request> request,
                                         std::shared_ptr<nav_filters::srv::ChangeFilter::Response> response) {
  /* Check if requested filter is one of the available filters */
  if (std::find(filters_.begin(), filters_.end(), request->filter_name) == filters_.end()) {
    /* Get list of available filters in a string */
    std::string available_filters = "";
    for (std::size_t i = 0; i < filters_.size(); i++) {
      if (i == filters_.size() - 1) {
        available_filters += filters_[i] + ".";
      } else {
        available_filters += filters_[i] + ", ";
      }
    }

    std::string service_response = "Requested filter is not one of the available filters: " + available_filters;
    
    RCLCPP_WARN(get_logger(), service_response.c_str());
    
    response->success = false;
    response->message = service_response;
    return;
  }

  /* Change current filter to requested one */
  current_filter_ = request->filter_name;

  response->success = true;
  response->message = "Changed current filter to " + request->filter_name + ".";
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void FilterHandler::timerCallback() {
  return;
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilterHandler>());
  rclcpp::shutdown();
  return 0;
}
