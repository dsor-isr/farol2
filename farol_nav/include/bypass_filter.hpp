#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "farol_msgs/msg/navigation_state.hpp"

/**
 * @brief   Test node
 * @author  Eduardo Cunha
 */
class BypassFilter : public rclcpp::Node {
  public:
    /* Constructor */
    BypassFilter();

    /* Destructor */
    ~BypassFilter();

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

    /* Other variables */
    
};