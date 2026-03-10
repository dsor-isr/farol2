#ifndef CLOCK_CHECKER_HPP
#define CLOCK_CHECKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <chrono>
#include <cmath>

class ClockChecker : public rclcpp::Node {
public:
    ClockChecker();

private:
    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void initializeParameters();
    void initializeSubscribers();
    
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    std::chrono::high_resolution_clock::time_point last_clock_time_;
    double speedup_factor_;
    double real_frequency_;
    double expected_frequency_;
    double expected_interval_ms_;
};

#endif // CLOCK_CHECKER_HPP