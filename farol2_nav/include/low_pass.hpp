#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "farol2_interfaces/msg/navigation_state.hpp"
#include "farol2_interfaces/msg/measurement.hpp"
#include "farol2_utils/angles.hpp"
#include "farol2_utils/filters/low_pass_filter.hpp"

/**
 * @brief   Low pass navigation filter
 * @author  Ravi Regalo
 */
class LowPass : public rclcpp::Node {
  public:
    /* Constructor */
    LowPass();

    /* Destructor */
    ~LowPass();

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

  private:
    /* Timer for node's callbacks */
    rclcpp::TimerBase::SharedPtr timer_;

    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<farol2_interfaces::msg::NavigationState>::SharedPtr state_pub_;
    
    rclcpp::Subscription<farol2_interfaces::msg::Measurement>::SharedPtr measurement_sub_;

    /* Callbacks */
    void measurement_callback(farol2_interfaces::msg::Measurement::SharedPtr msg);


    /* Other variables */
    farol2_interfaces::msg::NavigationState filter_state_msg_;
    rclcpp::Clock clock_;
    bool neglect_current_;

    std::array<farol2_utils::LowPassFilter, 17> lpfs_;
    std::array<double, 17> last_meas_{};
    std::string utm_zone_;
    double node_frequency_;
    
};