#include <magic_electric_sim.hpp>

/* Constructor */
MagicElectricSim::MagicElectricSim() : Node("magic_electric_sim"){
  clock_ = this->get_clock();
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
}

/* Destructor */
MagicElectricSim::~MagicElectricSim() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void MagicElectricSim::loadParams() {

  //////////////////////////////////////////////////////////////////////////////////////


  node_frequency_ = declare_parameter<double>("node_frequency");
  node_period_ = (1.0 / static_cast<double>(node_frequency_));

  // ==========================
  // INITIAL STATE
  // ==========================

  auto vel_param = declare_parameter<std::vector<double>>("initial_state.body_velocity");
  body_velocity_[0] = vel_param[0];
  body_velocity_[1] = vel_param[1];
  body_velocity_[2] = vel_param[2];

  auto orient_param = declare_parameter<std::vector<double>>("initial_state.orientation");
  orientation_[0] = orient_param[0];
  orientation_[1] = orient_param[1];
  orientation_[2] = orient_param[2];

  auto orient_rate_param = declare_parameter<std::vector<double>>("initial_state.orientation_rate");
  orientation_rate_[0] = orient_rate_param[0];
  orientation_rate_[1] = orient_rate_param[1];
  orientation_rate_[2] = orient_rate_param[2];

  body_acceleration_[0] = 0.0;
  body_acceleration_[1] = 0.0;
  body_acceleration_[2] = 0.0;

  angular_acceleration_[0] = 0.0;
  angular_acceleration_[1] = 0.0;
  angular_acceleration_[2] = 0.0;

  position_dot_[0] = 0.0;
  position_dot_[1] = 0.0;
  position_dot_[2] = 0.0;

  rudder_velocity_[0] = 0.0;
  rudder_velocity_[1] = 0.0;
  rudder_velocity_[2] = 0.0;

  // ==========================
  // ENVIRONMENT
  // ==========================

  fluid_density = declare_parameter<double>("environment.fluid_density");
  auto current_vec = declare_parameter<std::vector<double>>("environment.current");
  curr_vel_ = current_vec[0];
  curr_dir_ = current_vec[1];


  // ==========================
  // VEHICLE PARAMETERS
  // ==========================

  mu   = declare_parameter<double>("vehicle.mass");
  zg   = declare_parameter<double>("vehicle.zg");
  m_u  = declare_parameter<double>("vehicle.m_u");
  X_u  = declare_parameter<double>("vehicle.X_u");
  X_uu = declare_parameter<double>("vehicle.X_uu");
  m_uv = declare_parameter<double>("vehicle.m_uv");
  m_v = m_u - m_uv;
  N_r  = declare_parameter<double>("vehicle.N_r");
  N_rr = declare_parameter<double>("vehicle.N_rr");
  K_L  = declare_parameter<double>("vehicle.K_L");
  K_D0 = declare_parameter<double>("vehicle.K_D0");
  K_D1 = declare_parameter<double>("vehicle.K_D1");
  m_r  = declare_parameter<double>("vehicle.m_r");
  Y_v  = declare_parameter<double>("vehicle.Y_v");
  Y_vv = declare_parameter<double>("vehicle.Y_vv");


  // ==========================
  // RUDDER
  // ==========================

  rudder_actuation_sim_ = declare_parameter<bool>("vehicle.actuation.rudder.sim_rudder");
  max_rudder_angle_ = declare_parameter<double>("vehicle.actuation.rudder.max_angle_deg");
  min_rudder_angle_ = declare_parameter<double>("vehicle.actuation.rudder.min_angle_deg");
  max_rudder_angle_ = deg_to_rad(max_rudder_angle_);
  min_rudder_angle_ = deg_to_rad(min_rudder_angle_);
  rudder_ang_vel_ = deg_to_rad(declare_parameter<double>("vehicle.actuation.rudder.ang_vel"));




  // ==========================
  // PROPULSION
  // ==========================

  fixed_rpm_ = declare_parameter<bool>("vehicle.actuation.propulsion.fixed_rpm");
  fixed_rpm_value_ = declare_parameter<double>("vehicle.actuation.propulsion.fixed_rpm_value");
  max_rpm_ = declare_parameter<double>("vehicle.actuation.propulsion.max_rpm");
  min_rpm_ = declare_parameter<double>("vehicle.actuation.propulsion.min_rpm");
  K_T_BP = declare_parameter<double>("vehicle.actuation.propulsion.K_T_BP");
  prop_pitch = declare_parameter<double>("vehicle.actuation.propulsion.prop_pitch");
  D = declare_parameter<double>("vehicle.actuation.propulsion.D");
  l_ = declare_parameter<double>("vehicle.actuation.propulsion.l_");
  deadzone_propeller_pos_ = declare_parameter<double>("vehicle.actuation.propulsion.deadzone_pos");
  deadzone_propeller_neg_ = declare_parameter<double>("vehicle.actuation.propulsion.deadzone_neg");
  rpm_[0] = 0.0;
 
}

/**
 * @brief Initialise Subscribers
 */
void MagicElectricSim::initialiseSubscribers() {


  rpm_sub_  = create_subscription<control_allocation::msg::ThrusterRPM>(
                          declare_parameter<std::string>("topics.subscribers.rpm_command"), 
                          1, std::bind(&MagicElectricSim::rpmCallback, this, std::placeholders::_1));

  if(rudder_actuation_sim_){
    rudder_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
                          declare_parameter<std::string>("topics.subscribers.rudder_cmd"), 
                          1, std::bind(&MagicElectricSim::rudderAngleCallback, this, std::placeholders::_1));

  }else{
    rudder_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                          declare_parameter<std::string>("topics.subscribers.rudder_ref"), 
                          1, std::bind(&MagicElectricSim::rudderAngleCallback, this, std::placeholders::_1));
  }

  
  return;
}


/**
 * @brief Initialise Publishers
 */
void MagicElectricSim::initialisePublishers() {

  position_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      declare_parameter<std::string>("topics.publishers.position"), 1);
  velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      declare_parameter<std::string>("topics.publishers.body_velocity"), 1);
  orientation_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      declare_parameter<std::string>("topics.publishers.orientation"), 1);
  oreintation_rate_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      declare_parameter<std::string>("topics.publishers.orientation_rate"), 1);
  body_acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      declare_parameter<std::string>("topics.publishers.body_acceleration"), 1);
  angular_acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      declare_parameter<std::string>("topics.publishers.angular_acceleration"), 1);

  if(rudder_actuation_sim_){
    rudder_pub_ = create_publisher<std_msgs::msg::Float32>(
      declare_parameter<std::string>("topics.publishers.rudder_angle"), 1);
  }
      
  return;
}

/**
 * @brief Initialise Services
 */
void MagicElectricSim::initialiseServices() {
  /* Service servers */
  /* ... */

  /* service clients */
  /* ... */

  return;
}

/**
 * @brief Initialise Timers
 */
void MagicElectricSim::initialiseTimers() {

  /* Create timer */
  timer_ = create_timer(std::chrono::milliseconds(int(node_period_*1000)), std::bind(&MagicElectricSim::timerCallback, this));
}


/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */

void MagicElectricSim::rudderAngleCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!rudder_actuation_sim_) {
    rudder_angle_ = msg->data;  
    return;
  }

  const rclcpp::Time now = this->get_clock()->now();

  double dt = 0.0;
  if (!have_last_rudder_time_) {
    have_last_rudder_time_ = true;
    last_rudder_time_ = now;
    return; 
  }

  dt = (now - last_rudder_time_).seconds();
  last_rudder_time_ = now;

  updateRudder(msg->data, dt);
}


void MagicElectricSim::rpmCallback(const control_allocation::msg::ThrusterRPM::SharedPtr msg){

  rpm_[0] = msg->rpm[0];
  
  rpm_[0] = std::max(std::min(rpm_[0], max_rpm_), min_rpm_);
  

}

void MagicElectricSim::timerCallback() {


    updateState();

    geometry_msgs::msg::Vector3 pos_msg, body_vel_msg, ori_msg, ori_rate_msg, body_acc_msg, ang_acc_msg;

    pos_msg.x = position_[0];
    pos_msg.y = position_[1];
    pos_msg.z = position_[2];

    body_vel_msg.x = body_velocity_[0];
    body_vel_msg.y = body_velocity_[1];
    body_vel_msg.z = body_velocity_[2];

    ori_msg.x = orientation_[0];
    ori_msg.y = orientation_[1];
    ori_msg.z = orientation_[2];

    ori_rate_msg.x = orientation_rate_[0];
    ori_rate_msg.y = orientation_rate_[1];
    ori_rate_msg.z = orientation_rate_[2];

    body_acc_msg.x = body_acceleration_[0];
    body_acc_msg.y = body_acceleration_[1];
    body_acc_msg.z = body_acceleration_[2];

    ang_acc_msg.x = angular_acceleration_[0];
    ang_acc_msg.y = angular_acceleration_[1];
    ang_acc_msg.z = angular_acceleration_[2];

    position_pub_->publish(pos_msg);
    velocity_pub_->publish(body_vel_msg);
    orientation_pub_->publish(ori_msg);
    oreintation_rate_pub_->publish(ori_rate_msg);
    body_acceleration_pub_->publish(body_acc_msg);
    angular_acceleration_pub_->publish(ang_acc_msg);

    if(rudder_actuation_sim_){
        std_msgs::msg::Float32 rudder_msg;
        rudder_msg.data = rudder_angle_;
        rudder_pub_->publish(rudder_msg);
    }
  
  return;
}

void MagicElectricSim::updateRudder(double command, double dt)
{
  
  if (command > 0.5) {
    rudder_angle_ += rudder_ang_vel_ * dt;
  } else if (command < -0.5) {
    rudder_angle_ -= rudder_ang_vel_ * dt;
  }

  rudder_angle_ = std::clamp(rudder_angle_, min_rudder_angle_, max_rudder_angle_);
}

void MagicElectricSim::updateState(){

    //DYNAMICS

    if(fixed_rpm_){
        rpm_[0] = fixed_rpm_value_;
    }

    rpm_[0] = std::max(std::min(rpm_[0], max_rpm_), min_rpm_);

    if(rpm_[0] > deadzone_propeller_neg_ && rpm_[0] < deadzone_propeller_pos_){
        rpm_[0] = 0.0;
    }

    
    rps_ = rpm_[0]/60;

    if(abs(rps_*prop_pitch)<1e-6){
        tau_u_ = 0;
    }else{
        tau_u_ = 2*fluid_density*pow(rps_, 2)*pow(D, 4)*K_T_BP*(1 - body_velocity_[0]/(rps_*prop_pitch));
    }

    if(rps_ < 0){
        tau_u_ = -tau_u_;
    }

    if(std::abs(body_velocity_[0]) < 1e-6){
      RCLCPP_WARN(get_logger(), "Surge velocity is very low check if vehicle is moving");
    }else{
      sideslip_angle_ = atan2(body_velocity_[1],body_velocity_[0]);
    }

    
    course_angle_ = orientation_[2] + sideslip_angle_;
    speed_ = sqrt(pow(body_velocity_[0], 2) + pow(body_velocity_[1], 2));

    rudder_velocity_[0] = speed_*std::cos(course_angle_) + orientation_rate_[2]*l_*std::sin(orientation_[2]);
    rudder_velocity_[1] = speed_*std::sin(course_angle_) - orientation_rate_[2]*l_*std::cos(orientation_[2]);

    gamma_ = atan2(rudder_velocity_[1], rudder_velocity_[0]) - orientation_[2];

    gamma_ = wrapToPi(gamma_);

    rudder_angle_ = std::max(std::min(rudder_angle_, max_rudder_angle_), min_rudder_angle_);

    alpha_ = rudder_angle_ + gamma_;
    alpha_ = wrapToPi(alpha_);

    mod_ruder_velocity_ = sqrt(pow(rudder_velocity_[0], 2) + pow(rudder_velocity_[1], 2));
    lift_ = K_L*alpha_*pow(mod_ruder_velocity_, 2);
    drag_ = (K_D0 + K_D1*pow(alpha_, 2))*pow(mod_ruder_velocity_, 2);
    
    body_acceleration_[0] = (1/m_u)*(tau_u_ + m_v*body_velocity_[1]*orientation_rate_[2] + X_u*body_velocity_[0] +X_uu*std::abs(body_velocity_[0])*body_velocity_[0] - drag_*std::cos(gamma_) + lift_*std::sin(gamma_));
    tau_r_ = l_*(lift_*std::cos(gamma_) + drag_*std::sin(gamma_));
    angular_acceleration_[2] = (1/m_r)*((m_u-m_v)*body_velocity_[0]*body_velocity_[1] + N_r*orientation_rate_[2] + N_rr*std::abs(orientation_rate_[2])*orientation_rate_[2] + tau_r_);

    
    body_velocity_[0] += body_acceleration_[0]*node_period_; //TODO: Change to a dt in node time
    body_velocity_[1] += body_acceleration_[1]*node_period_;
    orientation_rate_[2] += angular_acceleration_[2]*node_period_;


    //KINEMATICS

    position_dot_[0] = body_velocity_[0]*std::cos(orientation_[2]) - body_velocity_[1]*std::sin(orientation_[2]) + curr_vel_*std::cos(curr_dir_);
    position_dot_[1] = body_velocity_[0]*std::sin(orientation_[2]) + body_velocity_[1]*std::cos(orientation_[2]) + curr_vel_*std::sin(curr_dir_);

    position_[0] += position_dot_[0]*node_period_;
    position_[1] += position_dot_[1]*node_period_;

    body_velocity_[0] = position_dot_[0]*std::cos(orientation_[2]) + position_dot_[1]*std::sin(orientation_[2]);
    body_velocity_[1] = -position_dot_[0]*std::sin(orientation_[2]) + position_dot_[1]*std::cos(orientation_[2]);

    orientation_[2] += orientation_rate_[2]*node_period_;
    orientation_[2] = wrapToPi(orientation_[2]);



}


int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MagicElectricSim>());
  rclcpp::shutdown();
  return 0;
}
