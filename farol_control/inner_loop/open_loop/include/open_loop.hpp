#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "control_allocation/msg/thruster_rpm.hpp"

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
    rclcpp::Publisher<control_allocation::msg::ThrusterRPM>::SharedPtr rpm_command_pub_;

    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr surge_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_status_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    void timerCallback();
    double surge_ref_;


    /* Callbacks */
    void surgeRefCallback(std_msgs::msg::Float32::SharedPtr msg);

    /* Other variables */
    control_allocation::msg::ThrusterRPM rpm_command_msg_;
    rclcpp::Clock clock_;
    bool surge_enabled_;
    double surge_gain_;
    int mission_status_ = 0;
};