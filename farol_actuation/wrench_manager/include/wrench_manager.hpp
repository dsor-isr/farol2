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
class WrenchManager : public rclcpp::Node {
  public:
    /* Constructor */
    WrenchManager();

    /* Destructor */
    ~WrenchManager();

  private:
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

    /* Timer for node's callbacks */
    rclcpp::TimerBase::SharedPtr timer_;
    
    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<control_allocation::msg::BodyWrenchRequest>::SharedPtr body_wrench_request_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thrust_x_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thrust_y_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thrust_z_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_x_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_y_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_z_sub_;

    /* Callbacks */
    void thrustXCallback(const std_msgs::msg::Float32 &msg);
    void thrustYCallback(const std_msgs::msg::Float32 &msg);
    void thrustZCallback(const std_msgs::msg::Float32 &msg);
    void torqueXCallback(const std_msgs::msg::Float32 &msg);
    void torqueYCallback(const std_msgs::msg::Float32 &msg);
    void torqueZCallback(const std_msgs::msg::Float32 &msg);

    /* Other variables */
    control_allocation::msg::BodyWrenchRequest body_wrench_request_msg_;
    rclcpp::Clock clock_;
    int freq_;
    bool surge_enabled_;
    double surge_gain_;
    std::vector<double> wrench_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<rclcpp::Time> last_received_ = {rclcpp::Time(0,1), rclcpp::Time(0,1), rclcpp::Time(0,1), 
                                                rclcpp::Time(0,1), rclcpp::Time(0,1), rclcpp::Time(0,1)};
};