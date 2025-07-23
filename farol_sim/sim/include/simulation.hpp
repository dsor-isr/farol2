#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

#include "sim/msg/placeholder.hpp"

/**
 * @brief   Sim
 * @author  Eduardo Cunha
 */
class Simulation : public rclcpp::Node {
  public:
    /* Constructor */
    Simulation();

    /* Destructor */
    ~Simulation();

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
    

    /* Callbacks */
    

    /* Other variables */
    std::string param1_;
    std::vector<std::string> list_of_params_;
    int param2_;
};