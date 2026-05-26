#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "farol2_allocation/msg/thruster_rpm.hpp"

// Topic/service names (short form, remapped in launch file)
#define TOPIC_SUB_SURGE_REF "surge_ref"
#define TOPIC_SUB_MISSION_STATUS "mission_status"
#define TOPIC_PUB_RPM_COMMAND "rpm_command"

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
    rclcpp::Publisher<farol2_allocation::msg::ThrusterRPM>::SharedPtr rpm_command_pub_;

    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr surge_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_status_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    void timerCallback();
    double surge_ref_ = 0.0;
    rclcpp::Time last_surge_ref_time_;
    bool has_surge_ref_ = false;


    /* Callbacks */
    void surgeRefCallback(std_msgs::msg::Float32::SharedPtr msg);

    /* Other variables */
    farol2_allocation::msg::ThrusterRPM rpm_command_msg_;
    rclcpp::Clock clock_;
    bool surge_enabled_;
    double surge_gain_;
    int mission_status_ = 0;
};