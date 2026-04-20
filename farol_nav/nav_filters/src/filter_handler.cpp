#include <filter_handler.hpp>

FilterHandler::FilterHandler() : Node("filter_handler")
{
  loadParams();
  initialisePublishers();
  initialiseServices();
  initialiseSubscribers();
}

FilterHandler::~FilterHandler() = default;

void FilterHandler::loadParams()
{
  // Get list of available filters and default one 
  filters_ = declare_parameter<std::vector<std::string>>("filters");
  current_filter_ = declare_parameter<std::string>("default_filter");

  if (filters_.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter 'filters' is empty.");
    rclcpp::shutdown();
    return;
  }

  // Declare subscriber topic for each filter (must exist in YAML now)
  topic_by_filter_.clear();
  topic_by_filter_.reserve(filters_.size());

  for (const auto & filter : filters_) {
    const std::string param_name = "topics.subscribers." + filter;
    const std::string topic = declare_parameter<std::string>(param_name);

    if (topic.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "Parameter '%s' is empty (filter '%s').",
                   param_name.c_str(), filter.c_str());
      rclcpp::shutdown();
      return;
    }

    topic_by_filter_[filter] = topic;
  }

  // Validate default filter is in filters list
  if (topic_by_filter_.find(current_filter_) == topic_by_filter_.end()) {
    RCLCPP_ERROR(get_logger(),
                 "Default filter '%s' is not one of the available filters.",
                 current_filter_.c_str());
    rclcpp::shutdown();
    return;
  }
}

void FilterHandler::initialisePublishers()
{
  state_pub_ = create_publisher<farol2_interfaces::msg::NavigationState>(
      declare_parameter<std::string>("topics.publishers.state"),
      rclcpp::QoS(1));
  nav_sat_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      declare_parameter<std::string>("topics.publishers.nav_sat_fix"),
      rclcpp::QoS(1));  
}

void FilterHandler::initialiseServices()
{
  change_filter_srv_ = create_service<nav_filters::srv::ChangeFilter>(
      declare_parameter<std::string>("topics.services.change_filter"),
      [this](const std::shared_ptr<nav_filters::srv::ChangeFilter::Request> request,
             std::shared_ptr<nav_filters::srv::ChangeFilter::Response> response)
      {
        changeFilterCallback(request, response);
      });
}

void FilterHandler::initialiseSubscribers()
{
  // subscribe to default filter at startup
  (void)switchSubscription(current_filter_);
}

bool FilterHandler::switchSubscription(const std::string & filter_name)
{
  // Validate requested filter exists
  const auto it = topic_by_filter_.find(filter_name);
  if (it == topic_by_filter_.end()) {
    // Build nice list for logging
    std::string available;
    for (size_t i = 0; i < filters_.size(); ++i) {
      available += filters_[i];
      available += (i + 1 < filters_.size()) ? ", " : "";
    }

    RCLCPP_WARN(get_logger(),
                "Requested filter '%s' not available. Available: %s",
                filter_name.c_str(), available.c_str());
    return false;
  }

  // If already active nothing to do
  if (filter_name == current_filter_ && active_sub_) {
    return true;
  }

  const std::string & topic = it->second;

  // Drop old subscription and create new one
  active_sub_.reset();

  // Create new subscription with callback that republishes messages to output topic
  active_sub_ = create_subscription<farol2_interfaces::msg::NavigationState>(
      topic,
      rclcpp::QoS(1),
      [this](farol2_interfaces::msg::NavigationState::ConstSharedPtr msg){
        // relay state 
        state_pub_->publish(*msg);
        // publish global coordinates 
        nav_sat_fix_msg_.header = msg->header;
        nav_sat_fix_msg_.latitude = msg->global_position.latitude;
        nav_sat_fix_msg_.longitude = msg->global_position.longitude;
        nav_sat_fix_pub_->publish(nav_sat_fix_msg_);
      });

  // Update internal state only after successful creation
  current_filter_ = filter_name;

  RCLCPP_INFO(get_logger(),
              "Switched to filter '%s' (subscribing to '%s').",
              current_filter_.c_str(), topic.c_str());

  return true;
}

void FilterHandler::changeFilterCallback(
    const std::shared_ptr<nav_filters::srv::ChangeFilter::Request> request,
    std::shared_ptr<nav_filters::srv::ChangeFilter::Response> response)
{
  if (!switchSubscription(request->filter_name)) {
    response->success = false;
    response->message = "Requested filter '" + request->filter_name + "' is not available.";
    return;
  }

  response->success = true;
  response->message = "Changed current filter to '" + current_filter_ + "'.";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilterHandler>());
  rclcpp::shutdown();
  return 0;
}
