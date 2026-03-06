#include "clock_checker.hpp"

ClockChecker::ClockChecker() : Node("clock_checker") {
    initializeParameters();
    initializeSubscribers();
    last_clock_time_ = std::chrono::high_resolution_clock::now();
}

void ClockChecker::initializeParameters() {
    real_frequency_ = this->declare_parameter<double>("node_frequency");
    speedup_factor_ = this->declare_parameter<double>("speedup_factor");
    
    expected_frequency_ = speedup_factor_ * real_frequency_;
    expected_interval_ms_ = 1000.0 / expected_frequency_;
}

void ClockChecker::initializeSubscribers() {
    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 10,
        std::bind(&ClockChecker::clock_callback, this, std::placeholders::_1));
}

void ClockChecker::clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    auto current_time = std::chrono::high_resolution_clock::now();
    double interval_ms = std::chrono::duration<double, std::milli>(
        current_time - last_clock_time_).count();

    if(std::abs(interval_ms - expected_interval_ms_) > 0.5 * expected_interval_ms_) {
        RCLCPP_WARN(this->get_logger(),
            "Clock interval deviates more than 50%% from expected!");
    }
    
    last_clock_time_ = current_time;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClockChecker>());
    rclcpp::shutdown();
    return 0;
}
