#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geodesic.hpp>
#include "farol_msgs/msg/utm.hpp"
#include "farol_msgs/msg/gcs.hpp"
#include "farol_msgs/msg/measurement.hpp"



/**
 * @brief   Sim
 * @author  Eduardo Cunha
 */
class SimSensors : public rclcpp::Node {
  public:
    /* Constructor */
    SimSensors();

    /* Destructor */
    ~SimSensors();

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
    rclcpp::Publisher<farol_msgs::msg::UTM>::SharedPtr utm_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pos_sub;
    rclcpp::Subscription<farol_msgs::msg::UTM>::SharedPtr utm_sub;
    /* Callbacks */
    void positionCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void utmCallback(const farol_msgs::msg::UTM::SharedPtr msg);

    double randn(double mu, double sigma);

    int freq_;
    double node_period_;

    double rcv_northing = 0.0;
    double rcv_easting = 0.0;
    int rcv_utm_zone = 29;
    bool rcv_northp = true;

    double meas_lat = 0.0;
    double meas_lon = 0.0;

    double send_northing = 0.0;
    double send_easting = 0.0;
    int send_utm_zone = 29;
    bool send_northp = true;

    double depth = 0.0;

    bool gnss_activate_;
    bool gnss_noise_;

    bool altimeter_activate_;
    bool altimeter_noise_;

    std::vector<double> gnss_bias; //Northing, Easting, Depth
    std::vector<double> gnss_variance; //Northing, Easting, Depth

};