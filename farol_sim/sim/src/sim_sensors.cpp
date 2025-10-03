#include <sim_sensors.hpp>

/* Constructor */
SimSensors::SimSensors() : Node("sim_sensors", 
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
SimSensors::~SimSensors() {
  /* Stop the timer */
  timer_gnss_->cancel();
  timer_depth_sensor_->cancel();
  timer_imu_->cancel();
}

/**
 * @brief Load parameters
 */
void SimSensors::loadParams() {
  
  gnss_activate_ = get_parameter("sim.sim_sensors.sensors.gnss.activate").as_bool();
  gnss_noise_ = get_parameter("sim.sim_sensors.sensors.gnss.noise.activate").as_bool();
  freq_gnss_ = get_parameter("sim.sim_sensors.sensors.gnss.frequency").as_int();
  gnss_bias = get_parameter("sim.sim_sensors.sensors.gnss.noise.bias").as_double_array();
  gnss_variance = get_parameter("sim.sim_sensors.sensors.gnss.noise.variance").as_double_array(); 


  depth_sensor_activate_ = get_parameter("sim.sim_sensors.sensors.depth_sensor.activate").as_bool();
  depth_sensor_noise_ = get_parameter("sim.sim_sensors.sensors.depth_sensor.noise.activate").as_bool();
  freq_depth_sensor_ = get_parameter("sim.sim_sensors.sensors.depth_sensor.frequency").as_int();
  depth_sensor_bias = get_parameter("sim.sim_sensors.sensors.depth_sensor.noise.bias").as_double();
  depth_sensor_variance = get_parameter("sim.sim_sensors.sensors.depth_sensor.noise.variance").as_double(); 

  imu_activate_ = get_parameter("sim.sim_sensors.sensors.imu.activate").as_bool();
  imu_noise_ = get_parameter("sim.sim_sensors.sensors.imu.noise").as_bool();
  freq_imu_ = get_parameter("sim.sim_sensors.sensors.imu.frequency").as_int();

  imu_acc_activate_ = get_parameter("sim.sim_sensors.sensors.imu.accelerometer.activate").as_bool();
  imu_acc_noise_ = get_parameter("sim.sim_sensors.sensors.imu.accelerometer.noise.activate").as_bool();
  imu_acc_bias = get_parameter("sim.sim_sensors.sensors.imu.accelerometer.noise.bias").as_double_array();
  imu_acc_variance = get_parameter("sim.sim_sensors.sensors.imu.accelerometer.noise.variance").as_double_array(); 

  imu_gyro_activate_ = get_parameter("sim.sim_sensors.sensors.imu.gyroscope.activate").as_bool();
  imu_gyro_noise_ = get_parameter("sim.sim_sensors.sensors.imu.gyroscope.noise.activate").as_bool();
  imu_gyro_bias = get_parameter("sim.sim_sensors.sensors.imu.gyroscope.noise.bias").as_double_array();
  imu_gyro_variance = get_parameter("sim.sim_sensors.sensors.imu.gyroscope.noise.variance").as_double_array(); 

  imu_ori_activate_ = get_parameter("sim.sim_sensors.sensors.imu.orientation.activate").as_bool();
  imu_ori_noise_ = get_parameter("sim.sim_sensors.sensors.imu.orientation.noise.activate").as_bool();
  imu_ori_bias = get_parameter("sim.sim_sensors.sensors.imu.orientation.noise.bias").as_double_array();
  imu_ori_variance = get_parameter("sim.sim_sensors.sensors.imu.orientation.noise.variance").as_double_array();  

}

/**
 * @brief Initialise Subscribers
 */
void SimSensors::initialiseSubscribers() {

  pos_sub  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.sim_sensors.topics.subscribers.position").as_string(), 
                          1, std::bind(&SimSensors::depthCallback, this, std::placeholders::_1));
  lin_acc_sub  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.sim_sensors.topics.subscribers.linear_acceleration").as_string(), 
                          1, std::bind(&SimSensors::linaccCallback, this, std::placeholders::_1));
  ang_acc_sub  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.sim_sensors.topics.subscribers.angular_acceleration").as_string(), 
                          1, std::bind(&SimSensors::angaccCallback, this, std::placeholders::_1));
  ori_sub  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.sim_sensors.topics.subscribers.orientation").as_string(), 
                          1, std::bind(&SimSensors::oriCallback, this, std::placeholders::_1));                    
  utm_sub = create_subscription<farol_msgs::msg::UTM>(
                          get_parameter("sim.sim_sensors.topics.subscribers.utm").as_string(), 
                          1, std::bind(&SimSensors::utmCallback, this, std::placeholders::_1));
  
  return;
}


/**
 * @brief Initialise Publishers
 */
void SimSensors::initialisePublishers() {

  utm_pub_ = create_publisher<farol_msgs::msg::UTM>(
      get_parameter("sim.sim_sensors.topics.publishers.gnss").as_string(), 1);
  depth_pub_ = create_publisher<farol_msgs::msg::Measurement>(
      get_parameter("sim.sim_sensors.topics.publishers.depth_sensor").as_string(), 1);
  ori_pub_ = create_publisher<farol_msgs::msg::Measurement>( 
      get_parameter("sim.sim_sensors.topics.publishers.imu.orientation").as_string(), 1);
  lin_acc_pub_ = create_publisher<farol_msgs::msg::Measurement>(
      get_parameter("sim.sim_sensors.topics.publishers.imu.linear_acceleration").as_string(), 1);
  ang_acc_pub_ = create_publisher<farol_msgs::msg ::Measurement>(
      get_parameter("sim.sim_sensors.topics.publishers.imu.angular_acceleration").as_string(), 1);  
  
      
  return;
}

/**
 * @brief Initialise Services
 */
void SimSensors::initialiseServices() {
  /* Service servers */
  /* ... */

  /* service clients */
  /* ... */

  return;
}

/**
 * @brief Initialise Timers
 */
void SimSensors::initialiseTimers() {
  /* Get node frequency from parameters */

  if(gnss_activate_){
    period_gnss_ = 1.0/freq_gnss_;
    timer_gnss_ = create_wall_timer(std::chrono::duration<double>(period_gnss_),
                             std::bind(&SimSensors::gnssTimerCallback, this));
  }

  if(depth_sensor_activate_){
    period_depth_sensor_ = 1.0/freq_depth_sensor_;
    timer_depth_sensor_ = create_wall_timer(std::chrono::duration<double>(period_depth_sensor_),
                             std::bind(&SimSensors::depthTimerCallback, this));
  }

  if(imu_activate_){
    period_imu_ = 1.0/freq_imu_;
    timer_imu_ = create_wall_timer(std::chrono::duration<double>(period_imu_),
                             std::bind(&SimSensors::imuTimerCallback, this));   
  }
                          
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */

void SimSensors::depthCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  depth_rcv = msg->z;

}

void SimSensors::linaccCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  rcv_lin_acc[0] = msg->x;
  rcv_lin_acc[1] = msg->y;
  rcv_lin_acc[2] = msg->z;
  
}

void SimSensors::angaccCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  rcv_ang_acc[0] = msg->x;
  rcv_ang_acc[1] = msg->y;
  rcv_ang_acc[2] = msg->z;
  
}

void SimSensors::oriCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  rcv_ori[0] = msg->x;
  rcv_ori[1] = msg->y;
  rcv_ori[2] = msg->z;
  
}

void SimSensors::utmCallback(const farol_msgs::msg::UTM::SharedPtr msg) {

  rcv_northing = msg->northing;
  rcv_easting = msg->easting;
  rcv_utm_zone = msg->utm_zone;
  rcv_northp = msg->northp;
  
}


void SimSensors::gnssTimerCallback() {

    farol_msgs::msg::UTM utm_msg;

    if(gnss_noise_) {
      send_northing = rcv_northing + randn(gnss_bias[0], gnss_variance[0]);
      send_easting = rcv_easting + randn(gnss_bias[1], gnss_variance[1]);
    
    }else{
      send_northing = rcv_northing;
      send_easting = rcv_easting;
    }
    //To simulate the true sensor who ouputs lat lon
    GeographicLib::UTMUPS::Reverse(rcv_utm_zone, rcv_northp,send_easting , send_northing, meas_lat, meas_lon);
    GeographicLib::UTMUPS::Forward(meas_lat, meas_lon, send_utm_zone, send_northp, send_easting, send_northing);
    
    utm_msg.northing = send_northing;
    utm_msg.easting = send_easting;
    utm_msg.utm_zone = send_utm_zone;
    utm_msg.northp = send_northp;
    
    utm_pub_->publish(utm_msg);
    
    return;

}

void SimSensors::depthTimerCallback() {

    farol_msgs::msg::Measurement depth_msg;

    if(depth_sensor_noise_){
      depth_send = depth_rcv + randn(depth_sensor_bias, depth_sensor_variance);
    }else{
      depth_send = depth_rcv;
    } 

    depth_msg.type = depth_msg.MEAS_DEPTH;
    depth_msg.value = {depth_send};           
    depth_msg.noise = {depth_sensor_variance};   
    depth_msg.data = "";                      
    depth_pub_->publish(depth_msg);  

    return;
}


void SimSensors::imuTimerCallback() {


  if(imu_ori_activate_){
      farol_msgs::msg::Measurement ori_msg;
      if(imu_ori_noise_ && imu_noise_){
        ori_send[0] = rcv_ori[0] + randn(imu_ori_bias[0], imu_ori_variance[0]);
        ori_send[1] = rcv_ori[1] + randn(imu_ori_bias[1], imu_ori_variance[1]);
        ori_send[2] = rcv_ori[2] + randn(imu_ori_bias[2], imu_ori_variance[2]);
      }else{
        ori_send[0] = rcv_ori[0];
        ori_send[1] = rcv_ori[1];
        ori_send[2] = rcv_ori[2];
      }
      ori_msg.type = ori_msg.MEAS_ORIENTATION;
      ori_msg.value = {ori_send[0], ori_send[1], ori_send[2]};
      if(imu_ori_noise_){
        ori_msg.noise = {imu_ori_variance[0], imu_ori_variance[1], imu_ori_variance[2]};
      }
      ori_msg.data = "";
      ori_pub_->publish(ori_msg);
      
    }

    if(imu_acc_activate_){
      farol_msgs::msg::Measurement lin_acc_msg;
      if(imu_acc_noise_ && imu_noise_){
        lin_acc_send[0] = rcv_lin_acc[0] + randn(imu_acc_bias[0], imu_acc_variance[0]);
        lin_acc_send[1] = rcv_lin_acc[1] + randn(imu_acc_bias[1], imu_acc_variance[1]);
        lin_acc_send[2] = rcv_lin_acc[2] + randn(imu_acc_bias[2], imu_acc_variance[2]);
      }else{
        lin_acc_send[0] = rcv_lin_acc[0];
        lin_acc_send[1] = rcv_lin_acc[1];
        lin_acc_send[2] = rcv_lin_acc[2];
      }
      lin_acc_msg.type = lin_acc_msg.MEAS_BODY_ACCEL_INERTIAL;
      lin_acc_msg.value = {lin_acc_send[0], lin_acc_send[1], lin_acc_send[2]};
      if(imu_acc_noise_){
        lin_acc_msg.noise = {imu_acc_variance[0], imu_acc_variance[1], imu_acc_variance[2]};
      }
      lin_acc_msg.data = "";
      lin_acc_pub_->publish(lin_acc_msg);
      
    }

    if(imu_gyro_activate_){
      farol_msgs::msg::Measurement ang_acc_msg;
      if(imu_gyro_noise_ && imu_noise_){
        ang_acc_send[0] = rcv_ang_acc[0] + randn(imu_gyro_bias[0], imu_gyro_variance[0]);
        ang_acc_send[1] = rcv_ang_acc[1] + randn(imu_gyro_bias[1], imu_gyro_variance[1]);
        ang_acc_send[2] = rcv_ang_acc[2] + randn(imu_gyro_bias[2], imu_gyro_variance[2]);
      }else{
        ang_acc_send[0] = rcv_ang_acc[0];
        ang_acc_send[1] = rcv_ang_acc[1];
        ang_acc_send[2] = rcv_ang_acc[2];
      }
      ang_acc_msg.type = ang_acc_msg.MEAS_ORIENTATION_ACCEL;
      ang_acc_msg.value = {ang_acc_send[0], ang_acc_send[1], ang_acc_send[2]};
      if(imu_gyro_noise_){
        ang_acc_msg.noise = {imu_gyro_variance[0], imu_gyro_variance[1], imu_gyro_variance[2]};
      }
      ang_acc_msg.data = "";
      ang_acc_pub_->publish(ang_acc_msg);
      
    }

    return;
}     

// from http://phoxis.org/2013/05/04/generating-random-numbers-from-normal-distribution-in-c/
double SimSensors::randn(double mu, double sigma){
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
  rclcpp::spin(std::make_shared<SimSensors>());
  rclcpp::shutdown();
  return 0;
}
