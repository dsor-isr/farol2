#include "sim_clock.hpp"

#include <chrono>
#include <stdexcept>

SimClock::SimClock()
: Node("sim_clock")

{
  // =========================
  // Declare & read parameters
  // =========================
  this->declare_parameter<double>("sim_time.real_frequency", 50.0);
  this->declare_parameter<double>("sim_time.speedup_factor", 1.0);

  real_frequency_ = this->get_parameter("sim_time.real_frequency").as_double();
  speedup_ = this->get_parameter("sim_time.speedup_factor").as_double();

  if (real_frequency_ <= 0.0 || speedup_ <= 0.0) {
    throw std::runtime_error(
      "sim_time.real_frequency and sim_time.speedup_factor must be > 0"
    );
  }

  // =========================
  // Compute timing quantities
  // =========================
  dt_ns_ = static_cast<uint64_t>((1.0 / real_frequency_) * 1e9); // [ns]
  wall_period_ns_ = static_cast<int64_t>((static_cast<double>(dt_ns_) / speedup_));

  // =========================
  // /clock publisher (correct QoS)
  // =========================
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
               .reliable()
               .transient_local();

  clock_pub_ =
    this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);

  // =========================
  // Wall timer (never ROS time)
  // =========================
  timer_ = this->create_wall_timer(
    std::chrono::nanoseconds(wall_period_ns_),
    std::bind(&SimClock::tick, this)
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Sim clock started: freq=%.2f Hz, dt=%ld ns, speedup=%.2f, wall_period=%ld ns",
    real_frequency_, dt_ns_, speedup_, wall_period_ns_
  );
}

void SimClock::tick()
{
  //RCLCPP_INFO(this->get_logger(), "Tick (sim_time_ns_=%ld ns)", sim_time_ns_);
  
  sim_time_ns_ += dt_ns_;

  rosgraph_msgs::msg::Clock msg;
  msg.clock = rclcpp::Time(sim_time_ns_);

  clock_pub_->publish(msg);
}

// =========================
// Main
// =========================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimClock>());
  rclcpp::shutdown();
  return 0;
}
