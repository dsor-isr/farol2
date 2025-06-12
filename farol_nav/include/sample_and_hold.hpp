#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "farol_msgs/msg/navigation_state.hpp"
#include "farol_msgs/msg/measurement.hpp"

/**
 * @brief   Sample and Hold navigation filter
 * @author  Eduardo Cunha
 */
class SampleAndHold : public rclcpp::Node {
  public:
    /* Constructor */
    SampleAndHold();

    /* Destructor */
    ~SampleAndHold();

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
    
    rclcpp::Subscription<farol_msgs::msg::Measurement>::SharedPtr measurement_sub_;

    /* Callbacks */
    void measurement_callback(const farol_msgs::msg::Measurement &msg);

    /* Other variables */
    farol_msgs::msg::NavigationState filter_state_msg_;
    rclcpp::Clock clock_;
};