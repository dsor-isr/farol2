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
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void SimSensors::loadParams() {
  
  freq_ = get_parameter("sim.simulation.node_frequency").as_int(); 
  node_period_ = 1.0/freq_;

  gnss_activate_ = get_parameter("sim.sim_sensors.sensors.gnss.activate").as_bool();
  gnss_noise_ = get_parameter("sim.sim_sensors.sensors.gnss.noise.activate").as_bool();
  gnss_bias = get_parameter("sim.sim_sensors.sensors.gnss.noise.bias").as_double_array();
  gnss_variance = get_parameter("sim.sim_sensors.sensors.gnss.noise.variance").as_double_array(); 

}

/**
 * @brief Initialise Subscribers
 */
void SimSensors::initialiseSubscribers() {

  pos_sub  = create_subscription<geometry_msgs::msg::Vector3>(
                          get_parameter("sim.sim_sensors.topics.subscribers.position").as_string(), 
                          1, std::bind(&SimSensors::positionCallback, this, std::placeholders::_1));
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
  node_period_ = 1.0/freq_;

  /* Create timer */
  timer_ = create_wall_timer(std::chrono::duration<double>(node_period_),
                             std::bind(&SimSensors::timerCallback, this));
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */

void SimSensors::positionCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {

  depth = msg->z;

}

void SimSensors::utmCallback(const farol_msgs::msg::UTM::SharedPtr msg) {

  rcv_northing = msg->northing;
  rcv_easting = msg->easting;
  rcv_utm_zone = msg->utm_zone;
  rcv_northp = msg->northp;
  
}



void SimSensors::timerCallback() {

  if(gnss_activate_) {
    farol_msgs::msg::UTM utm_msg;

    if(gnss_noise_) {
      send_northing = rcv_northing + randn(gnss_bias[0], gnss_variance[0]);
      send_easting = rcv_easting + randn(gnss_bias[1], gnss_variance[1]);
    
    }else{
      send_northing = rcv_northing;
      send_easting = rcv_easting;
    }
    GeographicLib::UTMUPS::Reverse(rcv_utm_zone, rcv_northp,send_easting , send_northing, meas_lat, meas_lon);
    GeographicLib::UTMUPS::Forward(meas_lat, meas_lon, send_utm_zone, send_northp, send_easting, send_northing);
    
    utm_msg.northing = send_northing;
    utm_msg.easting = send_easting;
    utm_msg.utm_zone = send_utm_zone;
    utm_msg.northp = send_northp;
    
    utm_pub_->publish(utm_msg);
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
