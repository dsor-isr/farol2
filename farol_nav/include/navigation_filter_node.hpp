#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief   Test node
 * @author  Eduardo Cunha
 */
class NavigationFilterNode : public rclcpp::Node {
  public:
    /* Constructor */
    NavigationFilterNode();

    /* Destructor */
    ~NavigationFilterNode();

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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    /* Other variables */
    size_t count_;
};