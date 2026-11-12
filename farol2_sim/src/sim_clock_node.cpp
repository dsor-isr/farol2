#include "sim_clock_node.hpp"
#include "farol2_utils/validation.hpp"
#include <stdexcept>

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
  node_frequency_ = declare_parameter<int>("node_frequency", 100);
  speedup_ = declare_parameter<double>("speedup", 1.0);

  farol2_utils::requirePositive(node_frequency_, "world node_frequency");
  farol2_utils::requirePositive(speedup_, "world speedup");

  node_period_ = 1.0 / static_cast<double>(node_frequency_);
  dt_ns_ = static_cast<uint64_t>(std::llround(node_period_ * 1e9));
  if (dt_ns_ == 0) { throw std::invalid_argument("World clock frequency is too high"); }
}

void SimClock::initialisePublishers()
{
  const auto clock_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", clock_qos);
}

void SimClock::initialiseTimers()
{
  const double period_ns = node_period_ * 1e9 / speedup_;
  if (!std::isfinite(period_ns) || period_ns < 1.0 || period_ns >= 9.22e18) {
    throw std::invalid_argument("World wall timer period is out of range");
  }
  const auto timer_period = std::chrono::nanoseconds(static_cast<int64_t>(std::llround(period_ns)));
  timer_ = create_wall_timer(timer_period, std::bind(&SimClock::timerCallback, this));
}

void SimClock::timerCallback()
{
  // Discovery is asynchronous: also detect a second clock started concurrently
  // after start_simple_world.launch.py's initial ownership check.
  if (count_publishers("/clock") > 1) {
    throw std::runtime_error("Multiple /clock publishers in this DDS domain. Use one world per ROS_DOMAIN_ID.");
  }
  sim_time_ns_ += dt_ns_;

  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = rclcpp::Time(sim_time_ns_);
  clock_pub_->publish(clock_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SimClock>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("sim_clock"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
