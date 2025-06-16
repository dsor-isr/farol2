#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "std_msgs/msg/string.hpp"
#include "farol_msgs/msg/navigation_state.hpp"
#include "nav_filters/srv/change_filter.hpp"

/**
 * @brief   Filter Handler
 * @author  Eduardo Cunha
 */
class FilterHandler : public rclcpp::Node {
  public:
    /* Constructor */
    FilterHandler();

    /* Destructor */
    ~FilterHandler();

    /* Load parameters */
    void loadParams();

    /* Initialise Subscribers */
    void initialiseSubscribers();

    /* Initialise Publishers */
    void initialisePublishers();

    /* Initialise Services */
    void initialiseServices();

    /* Initialise Timers */
    void initialiseTimers();
    
    /* Timer callback */
    void timerCallback();

  private:
    /* Timer for node's callbacks */
    rclcpp::TimerBase::SharedPtr timer_;

    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<farol_msgs::msg::NavigationState>::SharedPtr state_pub_;
    
    std::map<std::string, rclcpp::Subscription<farol_msgs::msg::NavigationState>::SharedPtr> subscription_map_;

    rclcpp::Service<nav_filters::srv::ChangeFilter>::SharedPtr change_filter_srv_;

    /* Callbacks */
    void nav_filter_callback(const farol_msgs::msg::NavigationState &msg, std::string filter_key);
    void changeFilterCallback(const std::shared_ptr<nav_filters::srv::ChangeFilter::Request> request,
                              std::shared_ptr<nav_filters::srv::ChangeFilter::Response> response);

    /* Other variables */
    farol_msgs::msg::NavigationState filter_state_msg_;
    rclcpp::Clock clock_;
    std::map<std::string, rclcpp::Parameter> subscription_topic_map_;
    std::map<std::string, farol_msgs::msg::NavigationState> nav_state_map_;
    std::vector<std::string> filters_;
    std::string current_filter_;
};