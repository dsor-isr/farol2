#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "control_allocation/msg/body_wrench_request.hpp"

/**
 * @brief   Open Loop Control
 * @author  Eduardo Cunha
 */
class OpenLoop : public rclcpp::Node {
  public:
    /* Constructor */
    OpenLoop();

    /* Destructor */
    ~OpenLoop();

  private:
    /* Load parameters */
    void loadParams();

    /* Initialise Subscribers */
    void initialiseSubscribers();

    /* Initialise Publishers */
    void initialisePublishers();

    /* Initialise Services */
    void initialiseServices();
    
    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thrust_x_pub_;
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr surge_ref_sub_;

    /* Callbacks */
    void surgeRefCallback(const std_msgs::msg::Float32 &msg);

    /* Other variables */
    std_msgs::msg::Float32 float32_msg_;
    rclcpp::Clock clock_;
    bool surge_enabled_;
    double surge_gain_;
};