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
    void gnssTimerCallback();
    void depthTimerCallback();
    void imuTimerCallback();

  private:
    /* Timer for sensor's callbacks */
    rclcpp::TimerBase::SharedPtr timer_gnss_;
    rclcpp::TimerBase::SharedPtr timer_depth_sensor_;
    rclcpp::TimerBase::SharedPtr timer_imu_;

    /* Declare publishers, subscribers, services, etc. */
    rclcpp::Publisher<farol_msgs::msg::UTM>::SharedPtr utm_pub_;
    rclcpp::Publisher<farol_msgs::msg::Measurement>::SharedPtr depth_pub_;
    rclcpp::Publisher<farol_msgs::msg::Measurement>::SharedPtr ori_pub_;
    rclcpp::Publisher<farol_msgs::msg::Measurement>::SharedPtr lin_acc_pub_;
    rclcpp::Publisher<farol_msgs::msg::Measurement>::SharedPtr ang_acc_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pos_sub;
    rclcpp::Subscription<farol_msgs::msg::UTM>::SharedPtr utm_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr lin_acc_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ang_acc_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ori_sub;
    /* Callbacks */
    void depthCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void utmCallback(const farol_msgs::msg::UTM::SharedPtr msg);
    void linaccCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void angaccCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void oriCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

    double randn(double mu, double sigma);

    int freq_gnss_;
    int freq_depth_sensor_;
    int freq_imu_;
    double period_gnss_;
    double period_depth_sensor_;
    double period_imu_;

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

    std::vector<double> gnss_bias; //Northing, Easting, Depth
    std::vector<double> gnss_variance; //Northing, Easting, Depth

    bool gnss_activate_;
    bool gnss_noise_;



    bool depth_sensor_activate_;
    bool depth_sensor_noise_;
    double depth_sensor_bias = 0.0;
    double depth_sensor_variance = 0.0;
    double depth_sensor_error = 0.0;
    
    double depth_rcv = 0.0;
    double depth_send = 0.0;


    bool imu_activate_;
    bool imu_noise_;

    bool imu_acc_activate_;
    bool imu_acc_noise_;

    bool imu_gyro_activate_;
    bool imu_gyro_noise_;

    bool imu_ori_activate_;
    bool imu_ori_noise_;

    std::vector<double> imu_acc_bias; //X, Y, Z
    std::vector<double> imu_acc_variance; //X, Y, Z

    std::vector<double> imu_gyro_bias; //Roll, Pitch, Yaw
    std::vector<double> imu_gyro_variance; //Roll, Pitch, Yaw

    std::vector<double> imu_ori_bias; //Roll, Pitch, Yaw
    std::vector<double> imu_ori_variance; //Roll, Pitch, Yaw

    double rcv_lin_acc[3];
    double rcv_ang_acc[3];
    double rcv_ori[3];

    double lin_acc_send[3];
    double ang_acc_send[3];
    double ori_send[3];


    


};