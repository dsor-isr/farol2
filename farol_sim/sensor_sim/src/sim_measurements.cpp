#include <sim_measurements.hpp>

/* Constructor */
SimMeasurements::SimMeasurements() : Node("sim_measurements", 
                  rclcpp::NodeOptions()
                    .allow_undeclared_parameters(true)
                    .automatically_declare_parameters_from_overrides(true)) {
  
  
  clock_ = this->get_clock();
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
}

/* Destructor */
SimMeasurements::~SimMeasurements() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void SimMeasurements::loadParams() {   

  freq_ = get_parameter("sim.sensor_sim.sim_measurements.node_frequency").as_int();

  gnss_activate_ = get_parameter("sim.sensor.gnss").as_bool();
  depth_sensor_activate_ = get_parameter("sim.sensor.depth_sensor").as_bool();
  imu_activate_ = get_parameter("sim.sensor.imu").as_bool();

  noise_activate_ = get_parameter("sim.sensor.noise.activate").as_bool();

  auto pos_bias_param = get_parameter("sim.sensor.noise.position.bias").as_double_array();
  pos_bias[0] = pos_bias_param[0];
  pos_bias[1] = pos_bias_param[1];
  pos_bias[2] = pos_bias_param[2];
  
  auto pos_variance_param = get_parameter("sim.sensor.noise.position.variance").as_double_array(); 
  pos_variance[0] = pos_variance_param[0];
  pos_variance[1] = pos_variance_param[1];
  pos_variance[2] = pos_variance_param[2];


  auto ori_bias_param = get_parameter("sim.sensor.noise.orientation.bias").as_double_array();
  ori_bias[0] = ori_bias_param[0];
  ori_bias[1] = ori_bias_param[1];
  ori_bias[2] = ori_bias_param[2];

  auto ori_variance_param = get_parameter("sim.sensor.noise.orientation.variance").as_double_array(); 
  ori_variance[0] = ori_variance_param[0];
  ori_variance[1] = ori_variance_param[1];
  ori_variance[2] = ori_variance_param[2];

  auto vel_bias_param = get_parameter("sim.sensor.noise.body_velocity.bias").as_double_array();
  vel_bias[0] = vel_bias_param[0];
  vel_bias[1] = vel_bias_param[1];
  vel_bias[2] = vel_bias_param[2];

  auto vel_variance_param = get_parameter("sim.sensor.noise.body_velocity.variance").as_double_array(); 
  vel_variance[0] = vel_variance_param[0];
  vel_variance[1] = vel_variance_param[1];
  vel_variance[2] = vel_variance_param[2];

  auto ori_rate_bias_param = get_parameter("sim.sensor.noise.orientation_rate.bias").as_double_array();
  ori_rate_bias[0] = ori_rate_bias_param[0];
  ori_rate_bias[1] = ori_rate_bias_param[1];
  ori_rate_bias[2] = ori_rate_bias_param[2];

  auto ori_rate_variance_param = get_parameter("sim.sensor.noise.orientation_rate.variance").as_double_array(); 
  ori_rate_variance[0] = ori_rate_variance_param[0];
  ori_rate_variance[1] = ori_rate_variance_param[1];
  ori_rate_variance[2] = ori_rate_variance_param[2];


  auto pos = this->get_parameter("sim.initial_state.position").as_double_array(); 
  origin_latitude_ = pos[0];
  origin_longitude_ = pos[1];

  ori[0] = 0.0;
  ori[1] = 0.0;
  ori[2] = 0.0;

  vel[0] = 0.0;
  vel[1] = 0.0;
  vel[2] = 0.0;

  ori_rate[0] = 0.0;
  ori_rate[1] = 0.0;
  ori_rate[2] = 0.0;
  

  GeographicLib::UTMUPS::Forward(origin_latitude_, origin_longitude_, utm_zone, northp, easting, northing);

}

/**
 * @brief Initialise Subscribers
 */
void SimMeasurements::initialiseSubscribers() {

  pos_sub_  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.sensor_sim.sim_measurements.topics.subscribers.position").as_string(), 
                          1, std::bind(&SimMeasurements::posCallback, this, std::placeholders::_1));
  ori_sub_  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.sensor_sim.sim_measurements.topics.subscribers.orientation").as_string(), 
                          1, std::bind(&SimMeasurements::oriCallback, this, std::placeholders::_1));                    
  vel_sub_  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.sensor_sim.sim_measurements.topics.subscribers.body_velocity").as_string(), 
                          1, std::bind(&SimMeasurements::velCallback, this, std::placeholders::_1));
  ori_rate_sub_  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.sensor_sim.sim_measurements.topics.subscribers.orientation_rate").as_string(), 
                          1, std::bind(&SimMeasurements::orirateCallback, this, std::placeholders::_1));  

  
  return;
}


/**
 * @brief Initialise Publishers
 */
void SimMeasurements::initialisePublishers() {
  

  meas_pub_ = create_publisher<farol_msgs::msg::Measurement>(
      get_parameter("sim.sensor_sim.sim_measurements.topics.publishers.measurement").as_string(), 1);

      
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

    int node_period_ = static_cast<double>(1000.0 / static_cast<double>(freq_));
    timer_ = create_timer(std::chrono::milliseconds(int(node_period_)), std::bind(&SimMeasurements::timerCallback, this));


}


void SimMeasurements::posCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  pos[0] = msg->x + northing; //Add origin UTM
  pos[1] = msg->y + easting; //Add origin UTM 
  pos[2] = msg->z;

}

void SimMeasurements::oriCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) { 

  //RCLCPP_INFO(get_logger(), "Received orientation: x: %f, y: %f, z: %f", msg->x, msg->y, msg->z);
  ori[0] = msg->x;
  ori[1] = msg->y;
  ori[2] = msg->z;
  
}

void SimMeasurements::velCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  vel[0] = msg->x;
  vel[1] = msg->y;
  vel[2] = msg->z;
  
}

void SimMeasurements::orirateCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  ori_rate[0] = msg->x;
  ori_rate[1] = msg->y;
  ori_rate[2] = msg->z;
  
}

void SimMeasurements::timerCallback() {

    //RCLCPP_INFO(get_logger(), "gnss_activate_: %d, depth_sensor_activate_: %d, imu_activate_: %d", gnss_activate_, depth_sensor_activate_, imu_activate_);
    //RCLCPP_INFO(get_logger(), "Orientation: x: %f, y: %f, z: %f", ori[0], ori[1], ori[2]);

    if(gnss_activate_){

      farol_msgs::msg::Measurement pos_msg, vel_msg;

      pos_msg.type = farol_msgs::msg::Measurement::MEAS_UTM_POSITION;
      pos_msg.value = {pos[0] + (noise_activate_ ? randn(pos_bias[0], pos_variance[0]) : 0.0),
                      pos[1] + (noise_activate_ ? randn(pos_bias[1], pos_variance[1]) : 0.0),
                      (double)utm_zone};
      meas_pub_->publish(pos_msg);


      vel_msg.type = farol_msgs::msg::Measurement::MEAS_BODY_VELOCITY_INERTIAL;
      vel_msg.value = {vel[0] + (noise_activate_ ? randn(ori_rate_bias[0], ori_rate_variance[0]) : 0.0),
                      vel[1] + (noise_activate_ ? randn(ori_rate_bias[1], ori_rate_variance[1]) : 0.0),
                      vel[2] + (noise_activate_ ? randn(ori_rate_bias[2], ori_rate_variance[2 ]) : 0.0)};
      meas_pub_->publish(vel_msg);
    }

    if(depth_sensor_activate_){

      farol_msgs::msg::Measurement depth_msg;

      depth_msg.type = farol_msgs::msg::Measurement::MEAS_DEPTH;
      depth_msg.value = {pos[2] + (noise_activate_ ? randn(pos_bias[2], pos_variance[2]) : 0.0)};
      meas_pub_->publish(depth_msg);
    }

    if(imu_activate_){

      farol_msgs::msg::Measurement  ori_msg, ori_rate_msg;

      ori_msg.type = farol_msgs::msg::Measurement::MEAS_ORIENTATION;
      ori_msg.value = {ori[0] + (noise_activate_ ? randn(ori_bias[0], ori_variance[0]) : 0.0),
                      ori[1] + (noise_activate_ ? randn(ori_bias[1], ori_variance[1]) : 0.0),
                      ori[2] + (noise_activate_ ? randn(ori_bias[2], ori_variance[2]) : 0.0)};
      meas_pub_->publish(ori_msg);



      ori_rate_msg.type = farol_msgs::msg::Measurement::MEAS_ORIENTATION_RATE;
      ori_rate_msg.value = {ori_rate[0] + (noise_activate_ ? randn(ori_rate_bias[0], ori_rate_variance[0]) : 0.0),
                          ori_rate[1] + (noise_activate_ ? randn(ori_rate_bias[1], ori_rate_variance[1]) : 0.0),
                          ori_rate[2] + (noise_activate_ ? randn(ori_rate_bias[2], ori_rate_variance[2]) : 0.0)};
      meas_pub_->publish(ori_rate_msg);
    }

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
