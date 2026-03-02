#include <magic_electric_sim.hpp>

/* Constructor */
MagicElectricSim::MagicElectricSim() : Node("magic_electric_sim", 
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
MagicElectricSim::~MagicElectricSim() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void MagicElectricSim::loadParams() {

  //////////////////////////////////////////////////////////////////////////////////////


  node_frequency_ = get_parameter("sim.vehicle_sim.magic_electric_sim.node_frequency").as_int();
  node_period_ = (1.0 / static_cast<double>(node_frequency_));

  // ==========================
  // INITIAL STATE
  // ==========================

  auto vel_param = this->get_parameter("sim.initial_state.body_velocity").as_double_array();
  body_velocity_[0] = vel_param[0];
  body_velocity_[1] = vel_param[1];
  body_velocity_[2] = vel_param[2];

  auto orient_param = this->get_parameter("sim.initial_state.orientation").as_double_array();
  orientation_[0] = orient_param[0];
  orientation_[1] = orient_param[1];
  orientation_[2] = orient_param[2];

  auto orient_rate_param = this->get_parameter("sim.initial_state.orientation_rate").as_double_array();
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

  fluid_density = this->get_parameter("sim.environment.fluid_density").as_double();
  auto current_vec = this->get_parameter("sim.environment.current").as_double_array();
  curr_vel_ = current_vec[0];
  curr_dir_ = current_vec[1];


  // ==========================
  // VEHICLE PARAMETERS
  // ==========================

  mu   = this->get_parameter("sim.vehicle.mass").as_double();
  zg   = this->get_parameter("sim.vehicle.zg").as_double();
  m_u  = this->get_parameter("sim.vehicle.m_u").as_double();
  X_u  = this->get_parameter("sim.vehicle.X_u").as_double();
  X_uu = this->get_parameter("sim.vehicle.X_uu").as_double();
  m_uv = this->get_parameter("sim.vehicle.m_uv").as_double();
  m_v = m_u - m_uv;
  N_r  = this->get_parameter("sim.vehicle.N_r").as_double();
  N_rr = this->get_parameter("sim.vehicle.N_rr").as_double();
  K_L  = this->get_parameter("sim.vehicle.K_L").as_double();
  K_D0 = this->get_parameter("sim.vehicle.K_D0").as_double();
  K_D1 = this->get_parameter("sim.vehicle.K_D1").as_double();
  m_r  = this->get_parameter("sim.vehicle.m_r").as_double();
  Y_v  = this->get_parameter("sim.vehicle.Y_v").as_double();
  Y_vv = this->get_parameter("sim.vehicle.Y_vv").as_double();


  // ==========================
  // RUDDER
  // ==========================

  rudder_actuation_sim_ = this->get_parameter("sim.vehicle.actuation.rudder.sim_rudder").as_bool();
  max_rudder_angle_ = this->get_parameter("sim.vehicle.actuation.rudder.max_angle_deg").as_double();
  min_rudder_angle_ = this->get_parameter("sim.vehicle.actuation.rudder.min_angle_deg").as_double();
  max_rudder_angle_ = deg_to_rad(max_rudder_angle_);
  min_rudder_angle_ = deg_to_rad(min_rudder_angle_);
  rudder_ang_vel_ = deg_to_rad(this->get_parameter("sim.vehicle.actuation.rudder.ang_vel").as_double());




  // ==========================
  // PROPULSION
  // ==========================

  fixed_rpm_ = this->get_parameter("sim.vehicle.actuation.propulsion.fixed_rpm").as_bool();
  fixed_rpm_value_ = this->get_parameter("sim.vehicle.actuation.propulsion.fixed_rpm_value").as_double();
  max_rpm_ = this->get_parameter("sim.vehicle.actuation.propulsion.max_rpm").as_double();
  min_rpm_ = this->get_parameter("sim.vehicle.actuation.propulsion.min_rpm").as_double();
  K_T_BP = this->get_parameter("sim.vehicle.actuation.propulsion.K_T_BP").as_double();
  prop_pitch = this->get_parameter("sim.vehicle.actuation.propulsion.prop_pitch").as_double();
  D = this->get_parameter("sim.vehicle.actuation.propulsion.D").as_double();
  l_ = this->get_parameter("sim.vehicle.actuation.propulsion.l_").as_double();
  deadzone_propeller_pos_ = this->get_parameter("sim.vehicle.actuation.propulsion.deadzone_pos").as_double();
  deadzone_propeller_neg_ = this->get_parameter("sim.vehicle.actuation.propulsion.deadzone_neg").as_double();
  rpm_[0] = 0.0;
 
}

/**
 * @brief Initialise Subscribers
 */
void MagicElectricSim::initialiseSubscribers() {


  rpm_sub_  = create_subscription<control_allocation::msg::ThrusterRPM>(
                          get_parameter("sim.vehicle_sim.magic_electric_sim.topics.subscribers.rpm_command").as_string(), 
                          1, std::bind(&MagicElectricSim::rpmCallback, this, std::placeholders::_1));

  if(rudder_actuation_sim_){
    rudder_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
                          get_parameter("sim.vehicle_sim.magic_electric_sim.topics.subscribers.rudder_cmd").as_string(), 
                          1, std::bind(&MagicElectricSim::rudderAngleCallback, this, std::placeholders::_1));

  }else{
    rudder_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                          get_parameter("sim.vehicle_sim.magic_electric_sim.topics.subscribers.rudder_ref").as_string(), 
                          1, std::bind(&MagicElectricSim::rudderAngleCallback, this, std::placeholders::_1));
  }

  
  return;
}


/**
 * @brief Initialise Publishers
 */
void MagicElectricSim::initialisePublishers() {

  position_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      get_parameter("sim.vehicle_sim.magic_electric_sim.topics.publishers.position").as_string(), 1);
  velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      get_parameter("sim.vehicle_sim.magic_electric_sim.topics.publishers.body_velocity").as_string(), 1);
  orientation_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      get_parameter("sim.vehicle_sim.magic_electric_sim.topics.publishers.orientation").as_string(), 1);
  oreintation_rate_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      get_parameter("sim.vehicle_sim.magic_electric_sim.topics.publishers.orientation_rate").as_string(), 1);
  body_acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      get_parameter("sim.vehicle_sim.magic_electric_sim.topics.publishers.body_acceleration").as_string(), 1);
  angular_acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      get_parameter("sim.vehicle_sim.magic_electric_sim.topics.publishers.angular_acceleration").as_string(), 1);

  if(rudder_actuation_sim_){
    RCLCPP_INFO(get_logger(), "rudder_pub active: %f", rudder_ang_vel_);
    rudder_pub_ = create_publisher<std_msgs::msg::Float32>(
      get_parameter("sim.vehicle_sim.magic_electric_sim.topics.publishers.rudder_angle").as_string(), 1);
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
