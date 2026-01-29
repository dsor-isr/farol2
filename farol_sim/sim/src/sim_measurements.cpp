#include <sim_sensors.hpp>

/* Constructor */
SimMeasurements::SimMeasurements() : Node("sim_measurements", 
                  rclcpp::NodeOptions()
                    .allow_undeclared_parameters(true)
                    .automatically_declare_parameters_from_overrides(true)) {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
}

/* Destructor */
SimMeasurements::~SimMeasurements() {
  /* Stop the timer */
  timer_gnss_->cancel();
  timer_depth_sensor_->cancel();
  timer_imu_->cancel();
}

/**
 * @brief Load parameters
 */
void SimMeasurements::loadParams() {   

  pos_noise_ = get_parameter("sim.sim_sensors.sensors.gnss.noise.activate").as_bool();
  pos_bias = get_parameter("sim.sim_sensors.sensors.gnss.noise.bias").as_double_array();
  pos_variance = get_parameter("sim.sim_sensors.sensors.gnss.noise.variance").as_double_array(); 

  ori_noise_ = get_parameter("sim.sim_sensors.sensors.imu.gyroscope.noise.activate").as_bool();
  ori_bias = get_parameter("sim.sim_sensors.sensors.imu.gyroscope.noise.bias").as_double_array();
  ori_variance = get_parameter("sim.sim_sensors.sensors.imu.gyroscope.noise.variance").as_double_array(); 

  vel_noise_ = get_parameter("sim.sim_sensors.sensors.depth_sensor.noise.activate").as_bool();
  vel_bias = get_parameter("sim.sim_sensors.sensors.depth_sensor.noise.bias").as_double_array();
  vel_variance = get_parameter("sim.sim_sensors.sensors.depth_sensor.noise.variance").as_double_array(); 

  angvel_noise_ = get_parameter("sim.sim_sensors.sensors.imu.noise").as_bool();
  angvel_bias = get_parameter("sim.sim_sensors.sensors.imu.accelerometer.noise.bias").as_double_array();
  angvel_variance = get_parameter("sim.sim_sensors.sensors.imu.accelerometer.noise.variance").as_double_array(); 


  origin_Latitude = get_parameter("sim.simulation.originLat").as_double();
  origin_Longitude = get_parameter("sim.simulation.originLon").as_double();

  GeographicLib::UTMUPS::Forward(origin_Latitude, origin_Longitude, utm_zone, northp, easting, northing);

}

/**
 * @brief Initialise Subscribers
 */
void SimMeasurements::initialiseSubscribers() {

  pos_sub  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.simulation.topics.publishers.position").as_string(), 
                          1, std::bind(&SimMeasurements::posCallback, this, std::placeholders::_1));
  ori_sub  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.sim_sensors.topics.subscribers.orientation").as_string(), 
                          1, std::bind(&SimMeasurements::oriCallback, this, std::placeholders::_1));                    
  vel_sub  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.simulation.topics.publishers.velocity").as_string(), 
                          1, std::bind(&SimMeasurements::velCallback, this, std::placeholders::_1));
  ang_vel_sub  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.simulation.topics.publishers.angular_velocity").as_string(), 
                          1, std::bind(&SimMeasurements::angvelCallback, this, std::placeholders::_1));  

  
  return;
}


/**
 * @brief Initialise Publishers
 */
void SimMeasurements::initialisePublishers() {
  

  pos_pub_ = create_publisher<farol_msgs::msg::Measurement>(
      get_parameter("sim.sim_sensors.topics.publishers.depth_sensor").as_string(), 1);
  ori_pub_ = create_publisher<farol_msgs::msg::Measurement>( 
      get_parameter("sim.sim_sensors.topics.publishers.imu.orientation").as_string(), 1);
  vel_pub_ = create_publisher<farol_msgs::msg::Measurement>(
      get_parameter("sim.sim_sensors.topics.publishers.imu.linear_acceleration").as_string(), 1);
  angvel_pub_ = create_publisher<farol_msgs::msg ::Measurement>(
      get_parameter("sim.sim_sensors.topics.publishers.imu.angular_acceleration").as_string(), 1);  
  
      
  return;
}

/**
 * @brief Initialise Services
 */
void SimMeasurements::initialiseServices() {
  /* Service servers */
  /* ... */

  /* service clients */
  /* ... */

  return;
}

/**
 * @brief Initialise Timers
 */
void SimMeasurements::initialiseTimers() {

    int gnss_period_ms = static_cast<int>(1000.0 / static_cast<double>(freq_gnss_));
    timer_gnss_ = this->create_wall_timer(
          std::chrono::milliseconds(gnss_period_ms),
          std::bind(&SimMeasurements::gnssTimerCallback, this));


}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */

void SimMeasurements::posCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  pos = msg->x + northing; //Add origin UTM
  pos = msg->y + easting; //Add origin UTM 
  pos = msg->z;

}

void SimSensSimMeasurements::oriCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  ori[0] = msg->x;
  ori[1] = msg->y;
  ori[2] = msg->z;
  
}

void SimMeasurements::velCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  vel[0] = msg->x;
  vel[1] = msg->y;
  vel[2] = msg->z;
  
}

void SimMeasurements::angvelCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  angvel[0] = msg->x;
  angvel[1] = msg->y;
  angvel[2] = msg->z;
  
}





void SimMeasurements::TimerCallback() {

    farol_msgs::msg::Measurement pos_msg, depth_msg, ori_msg, vel_msg, angvel_msg;

    pos_msg.type = farol_msgs::msg::Measurement::MEAS_UTM_POSITION;
    pos_msg.value = {pos + (pos_noise_ ? randn(pos_bias[0], pos_variance[0]) : 0.0),
                     pos + (pos_noise_ ? randn(pos_bias[1], pos_variance[1]) : 0.0),
                     utm_zone};
    pos_pub_->publish(pos_msg);


    depth_msg.type = farol_msgs::msg::Measurement::MEAS_DEPTH;
    depth_msg.value = {pos + (pos_noise_ ? randn(pos_bias[2], pos_variance[2]) : 0.0)};
    depth_pub_->publish(depth_msg);


    ori_msg.type = farol_msgs::msg::Measurement::MEAS_ORIENTATION;
    ori_msg.value = {ori[0] + (ori_noise_ ? randn(ori_bias[0], ori_variance[0]) : 0.0),
                     ori[1] + (ori_noise_ ? randn(ori_bias[1], ori_variance[1]) : 0.0),
                     ori[2] + (ori_noise_ ? randn(ori_bias[2], ori_variance[2]) : 0.0)};
    ori_pub_->publish(ori_msg);


    vel_msg.type = farol_msgs::msg::Measurement::MEAS_BODY_VELOCITY_INERTIAL;
    vel_msg.value = {vel[0] + (vel_noise_ ? randn(angvel_bias[0], angvel_variance[0]) : 0.0),
                     vel[1] + (vel_noise_ ? randn(angvel_bias[1], angvel_variance[1]) : 0.0),
                     vel[2] + (vel_noise_ ? randn(angvel_bias[2], angvel_variance[2 ]) : 0.0)};
    vel_pub_->publish(vel_msg);


    angvel_msg.type = farol_msgs::msg::Measurement::MEAS_ORIENTATION_RATE;
    angvel_msg.value = {angvel[0] + (angvel_noise_ ? randn(angvel_bias[0], angvel_variance[0]) : 0.0),
                        angvel[1] + (angvel_noise_ ? randn(angvel_bias[1], angvel_variance[1]) : 0.0),
                        angvel[2] + (angvel_noise_ ? randn(angvel_bias[2], angvel_variance[2]) : 0.0)};
    angvel_pub_->publish(angvel_msg);
    

    return;

}


    

// from http://phoxis.org/2013/05/04/generating-random-numbers-from-normal-distribution-in-c/
double SimMeasurements::randn(double mu, double sigma){
  double U1, U2, W, mult;
  static double X1, X2;
  static int call = 0;
  if (call)
  {
    call = !call;
    return (mu + sigma * (double)X2);
  }
  do
  {
    U1 = -1 + ((double)rand() / RAND_MAX) * 2;
    U2 = -1 + ((double)rand() / RAND_MAX) * 2;
    W = pow(U1, 2) + pow(U2, 2);
  } while (W >= 1 || W == 0);

  mult = sqrt((-2 * log(W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;
  call = !call;
  return (mu + sigma * (double)X1);
}

/**
 * @brief Main function
 */

int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimMeasurements>());
  rclcpp::shutdown();
  return 0;
}
