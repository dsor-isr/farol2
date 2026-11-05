#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class SimClock : public rclcpp::Node {
public:
  SimClock();
  ~SimClock() override;

private:
  void loadParams();
  void initialisePublishers();
  void initialiseTimers();
  void timerCallback();

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int node_frequency_{10};
  double node_period_{0.1};
  double speedup_{1.0};
  uint64_t sim_time_ns_{0};
  uint64_t dt_ns_{100000000};
};
