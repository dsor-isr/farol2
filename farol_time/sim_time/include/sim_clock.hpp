#ifndef SIM_TIME__SIM_CLOCK_HPP_
#define SIM_TIME__SIM_CLOCK_HPP_

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <functional>



/**
 * @brief Simulation time authority node.
 *
 * Publishes ROS simulation time on /clock with a fixed timestep
 * and configurable speedup factor.
 *
 * Parameters:
 *  - sim_time.real_frequency (double): simulation frequency [Hz]
 *  - sim_time.speedup_factor (double): speedup factor (> 0)
 *
 * Notes:
 *  - This node MUST use wall time (use_sim_time = false)
 *  - All other nodes should use_sim_time = true
 */
class SimClock : public rclcpp::Node
{
public:
  explicit SimClock();

private:
  // =========================
  // Internal methods
  // =========================
  void tick();

  // =========================
  // Parameters (configuration)
  // =========================
  double real_frequency_;     ///< base simulation frequency [Hz]
  double speedup_;            ///< simulation speedup factor

  // =========================
  // Derived timing values
  // =========================
  uint64_t dt_ns_ = 0;              ///< simulation timestep [ns]
  uint64_t wall_period_ns_ = 0;     ///< wall-timer period [ns]

  // =========================
  // Simulation state
  // =========================
  uint64_t sim_time_ns_ = 0;        ///< current simulation time [ns]

  // =========================
  // ROS interfaces
  // =========================
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // SIM_TIME__SIM_CLOCK_HPP_
