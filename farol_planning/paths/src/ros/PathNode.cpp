#include "PathNode.h"

/**
 * @brief  PathNode constructor. Initializes the subscribers, publishers, 
 * timers, parameters, etc...
 */
PathNode::PathNode() : Node("paths", 
                            rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true)) {

  /* Instantiate the ROS subscribers, publishers, etc */
  this->loadParams();
  this->initializeSubscribers();
  this->initializePublishers();
  this->initializeServices();
  this->initializeTimer();

  /* Allocate memory to store the Path object */
  this->path_ = new Path();
}

/**
 * @brief  Class destructor. Called when deleting the class object.
 */
PathNode::~PathNode() {
  /* Stop the timer */
  this->timer_->cancel();

  /* Free the memory used by the path */
  delete this->path_;
}

void PathNode::loadParams() {
  this->frame_id_ = get_parameter("planning.paths.frame_id").as_string();
}

/**
 * @brief  A method for initializing all the subscribers. This method is called
 * by the constructor of the PathNode class upon object creation
 */
void PathNode::initializeSubscribers() {
  this->gamma_sub_ = create_subscription<std_msgs::msg::Float64>(
                      get_parameter("planning.paths.topics.subscribers.gamma").as_string(), 
                      1, std::bind(&PathNode::gammaCallback, this, std::placeholders::_1));

  this->vehicle_sub_ = create_subscription<farol_msgs::msg::NavigationState>(
                        get_parameter("planning.paths.topics.subscribers.vehicle_state").as_string(), 
                        1, std::bind(&PathNode::vehicleStateCallback, this, std::placeholders::_1));

}

/**
 * @brief  A method for initializing all the subscribers. This method is called by
 * the constructor of the PathNode class upon object creation
 */
void PathNode::initializePublishers() {	  
  this->path_pub_ = create_publisher<paths::msg::PathData>(
                      get_parameter("planning.paths.topics.publishers.path_data").as_string(), 1);

  this->virtual_target_pub_ = create_publisher<farol_msgs::msg::StateConsole>(
                                get_parameter("planning.paths.topics.publishers.virtual_target_state").as_string(), 1);
}

/**
 * @brief  A method for initializing all the timers. This method is called by the
 * constructor of the PathNode class upon object creation
 */
void PathNode::initializeTimer() {
  /* Get node frequency from parameters */
  int freq = get_parameter("planning.paths.node_frequency").as_int();

  /* Create timer */
  timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq*1000)), std::bind(&PathNode::timerCallback, this));
}

void PathNode::timerCallback() {

  /* Declare the variables to store the values of the path */
  std::optional<Eigen::Vector3d> pd;
  std::optional<Eigen::Vector3d> d_pd;
  std::optional<Eigen::Vector3d> dd_pd;
  std::optional<double> tangent = 0;
  std::optional<double> curvature = 0;
  std::optional<double> derivative_norm = 0;
  std::optional<double> vd = 0;
  std::optional<double> d_vd = 0;
  double vehicle_speed = 0;
  std::pair<double, double> min_max_gamma_path;

  /* If the mode is to use the closest point to the vehicle, than get the gamma of that point */
  if(this->closer_point_mode_) {
    /* Then just override the gamma with the one corresponding to the closest point on the path */
    this->gamma_ = this->path_->getClosestGamma(this->vehicle_pos_);
  }

  /* Construct the message to send with the path information and with the virtual target current state */
  paths::msg::PathData msg; 
  farol_msgs::msg::StateConsole vt_state_msg;

  if(!this->path_->isEmpty() && this->gamma_.has_value()) {

    /* Get the position, first and second derivatives */
    pd = this->path_->eq_pd(this->gamma_.value());
    d_pd = this->path_->eq_d_pd(this->gamma_.value());
    dd_pd = this->path_->eq_dd_pd(this->gamma_.value());

    /* Get the values of the tangent angle, curvature and derivative_norm */
    tangent = this->path_->tangent(this->gamma_.value());
    curvature = this->path_->curvature(this->gamma_.value());
    derivative_norm = this->path_->derivative_norm(this->gamma_.value());
    vd = this->path_->eq_vd(this->gamma_.value());
    d_vd = this->path_->eq_d_vd(this->gamma_.value());
    
    /* Compute the vehicle speed */
    vehicle_speed = vd.value_or(0.0) * derivative_norm.value_or(0.0);
  
    /* Get the values for the minimum and maximum values allowed for the path */
    min_max_gamma_path = this->path_->getMinMaxGamma();

    /* Check if we have all the data */
    if(pd && d_pd && dd_pd && tangent && curvature && derivative_norm) {

      /* Header for the message */
      msg.header.stamp = this->clock_.now();
      msg.header.frame_id = this->frame_id_;

      /* The value of gamma used to make the computations */
      msg.gamma = this->gamma_.value();

      /* The data that we got from the path */
      for(int i = 0; i < 3; i++) {
        msg.pd[i] = pd.value()[i];
        msg.d_pd[i] = d_pd.value()[i];
        msg.dd_pd[i] = dd_pd.value()[i]; 
      }

      msg.curvature = curvature.value();
      msg.tangent = tangent.value();
      msg.derivative_norm = derivative_norm.value();

      msg.vd = vd.value();
      msg.d_vd = d_vd.value();
      msg.vehicle_speed = vehicle_speed;

      msg.gamma_min = min_max_gamma_path.first;
      msg.gamma_max = min_max_gamma_path.second;

      /* Publish the message with path info */
      this->path_pub_->publish(msg);

      /* Create a message with the state of the virtual target and 
       * publish it (to be seen as a virtual vehicle)
       */
      vt_state_msg.x = pd.value()[0];
      vt_state_msg.y = pd.value()[1];
      vt_state_msg.z = pd.value()[2];
      vt_state_msg.yaw = tangent.value() * 180.0 / M_PI; 

      /* Publish the message with the state of the virtual target */
      this->virtual_target_pub_->publish(vt_state_msg);
    }
  }
}

/**
 * @brief  Callback to update the current gamma of the path
 *
 * @param msg  A Float64/Double with the current value of gamma
 */
void PathNode::gammaCallback(const std_msgs::msg::Float64 &msg) {

  /* Update the current gamma value */
  this->gamma_ = msg.data;
}

/**
 * @brief  Callback to update the current vehicle position
 *
 * @param msg  A auv_msgs/NavigationStatus messages with the current state of the vehicle
 */
void PathNode::vehicleStateCallback(const farol_msgs::msg::NavigationState &msg) {
  
  /* Update the vehicle position */
  this->vehicle_pos_ <<  msg.utm_position.northing, msg.utm_position.easting, msg.altimeter;
}

/**
 * @brief  Main method. The entry point of the PathNode program
 *
 * @param argc  The number of arguments passed to the program
 * @param argv  The vector with the arguments passed to the program
 *
 * @return  An int symbolizing success or failure
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathNode>());
  rclcpp::shutdown();
  return 0;
}

