#pragma once

#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geodesic.hpp>
#include "farol_msgs/msg/measurement.hpp"

/**
 * @brief   Sim Measurements
 * @author  André Carvalho
 */
class SimMeasurements : public rclcpp::Node {
  public:
    /* Constructor */
    SimMeasurements();

    /* Destructor */
    ~SimMeasurements();

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
    rclcpp::TimerBase::SharedPtr timer_;

    /* Publishers */
    rclcpp::Publisher<farol_msgs::msg::Measurement>::SharedPtr meas_pub_;

    /* Subscribers */
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ori_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ori_rate_sub_;

    rclcpp::Clock::SharedPtr clock_;

    /* Callbacks */
    void posCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void oriCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void velCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void orirateCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

    /* Utils */
    double randn(double mu, double sigma);

    int freq_;

    bool gnss_activate_;
    bool depth_sensor_activate_;
    bool imu_activate_;
    
    bool noise_activate_{false};

    double origin_latitude_;
    double origin_longitude_;
    int utm_zone;
    bool northp;
    double northing;
    double easting;

    std::array<double,3> pos_bias;       
    std::array<double,3>pos_variance;

    std::array<double,3> ori_bias;       
    std::array<double,3> ori_variance;

    std::array<double,3> vel_bias;       
    std::array<double,3> vel_variance;

    std::array<double,3> ori_rate_bias;    
    std::array<double,3> ori_rate_variance;


    std::array<double,3> pos;      
    std::array<double,3> ori;      
    std::array<double,3> vel;      
    std::array<double,3> ori_rate;   
};
