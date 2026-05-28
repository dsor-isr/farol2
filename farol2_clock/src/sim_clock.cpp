#include "sim_clock.hpp"

SimClock::SimClock() : Node("sim_clock")
{
  loadParams();
  initialisePublishers();
  initialiseTimers();
}

SimClock::~SimClock()
{
  if (timer_) {
    timer_->cancel();
  }
}

void SimClock::loadParams()
{
  node_frequency_ = declare_parameter<int>("node_frequency");
  speedup_ = declare_parameter<double>("speedup");

  if (node_frequency_ <= 0) {
    RCLCPP_WARN(get_logger(), "node_frequency must be positive. Falling back to 10 Hz.");
    node_frequency_ = 10;
  }

  if (speedup_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "speedup must be positive. Falling back to 1.0.");
    speedup_ = 1.0;
  }

  node_period_ = 1.0 / static_cast<double>(node_frequency_);
  dt_ns_ = static_cast<uint64_t>(std::llround(node_period_ * 1e9));
}

void SimClock::initialisePublishers()
{
  const auto clock_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", clock_qos);
}

void SimClock::initialiseTimers()
{
  const auto timer_period = std::chrono::nanoseconds(
      static_cast<int64_t>(std::llround(node_period_ * 1e9 / speedup_)));
  timer_ = create_wall_timer(timer_period, std::bind(&SimClock::timerCallback, this));
}

void SimClock::timerCallback()
{
  sim_time_ns_ += dt_ns_;

  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = rclcpp::Time(sim_time_ns_);
  clock_pub_->publish(clock_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimClock>());
  rclcpp::shutdown();
  return 0;
}
