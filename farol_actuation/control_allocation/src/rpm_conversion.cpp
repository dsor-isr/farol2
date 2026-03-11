#include <rpm_conversion.hpp>

/* Constructor */
RPMConversion::RPMConversion() : Node("rpm_conversion") {
  loadParams();
  initialisePublishers();
  initialiseSubscribers();
  // initialiseServices();
}

/* Destructor */
RPMConversion::~RPMConversion() = default;

/**
 * @brief Initialise Subscribers
 */
void RPMConversion::initialiseSubscribers() {
  thruster_force_sub_ = create_subscription<control_allocation::msg::ThrusterForce>(
    declare_parameter<std::string>("topics.subscribers.thruster_force"),
    rclcpp::QoS(1),
    [this](control_allocation::msg::ThrusterForce::SharedPtr msg){thrusterForceCallback(msg);});

  nav_state_sub_ = create_subscription<farol_interfaces::msg::NavigationState>(
    declare_parameter<std::string>("topics.subscribers.nav_state"),
    rclcpp::QoS(1),
    [this](farol_interfaces::msg::NavigationState::SharedPtr msg){surge_ = msg->body_velocity_fluid.x;});
}

/**
 * @brief Load parameters
 */
void RPMConversion::loadParams() {
  mode_ = declare_parameter<int>("mode");
  if (mode_ != 0 && mode_ != 1) {
    mode_ = 1;
  }

  /* Thruster parameters */
  rho_ = declare_parameter<double>("mode0.rho");
  K_T_BP_ = declare_parameter<double>("mode0.K_T_BP");
  prop_pitch_ = declare_parameter<double>("mode0.prop_pitch");
  D_ = declare_parameter<double>("mode0.D");
  
  /* Thruster coefficients */
  coef_fwd_ = declare_parameter<std::vector<double>>("mode1.coef_fwd");
  coef_bwd_ = declare_parameter<std::vector<double>>("mode1.coef_bwd");

  /* Minimum and maximum RPM allowed */
  max_rpm_ = declare_parameter<double>("max_rpm");
  min_rpm_ = declare_parameter<double>("min_rpm");
}

/**
 * @brief Initialise Publishers
 */
void RPMConversion::initialisePublishers() {
  rpm_command_pub_ = create_publisher<control_allocation::msg::ThrusterRPM>(
    declare_parameter<std::string>("topics.publishers.rpm_command"),
    rclcpp::QoS(1));
}

/**
 * @brief Initialise Services
 */
void RPMConversion::initialiseServices() {}

/**
 * @brief Compute force for each thruster based on body wrench (force and torque) request.
 */
void RPMConversion::thrusterForceCallback(control_allocation::msg::ThrusterForce::SharedPtr msg) {
  /* Create message to publish thruster RPM */
  rpm_command_msg_.header.stamp = msg->header.stamp;
  rpm_command_msg_.rpm = {};
  
  /* Convert force to RPM */
  for (int i = 0; i < (int)msg->force.size(); i++) {
    // uses thrstcurve parameters that are constant and specified in yaml file
    if (mode_ == 1) {
      if (msg->force[i] == 0.0) {
      rpm_command_msg_.rpm.push_back(0.0);
      } else if (msg->force[i] > 0.0) {
      rpm_value_ = (-coef_fwd_[1] + sqrt(coef_fwd_[1] * coef_fwd_[1] - 4 * coef_fwd_[0] * (coef_fwd_[2] - msg->force[i]))) / (2 * coef_fwd_[0]);

      /* Saturate */
      rpm_value_ = (rpm_value_ > max_rpm_) ? max_rpm_ : rpm_value_;

      rpm_command_msg_.rpm.push_back(rpm_value_);
      } else {
      rpm_value_ = (-coef_bwd_[1] + sqrt(coef_bwd_[1] * coef_bwd_[1] - 4 * coef_bwd_[0] * (coef_bwd_[2] - msg->force[i]))) / (2 * coef_bwd_[0]);

      /* Saturate */
      rpm_value_ = (rpm_value_ < min_rpm_) ? min_rpm_ : rpm_value_;

        rpm_command_msg_.rpm.push_back(rpm_value_);
      }
    
    // computes thrstcurve parameters that depend on current surge velocity
    } else if (mode_ == 0) {
      if (msg->force[i] == 0.0) {
        rpm_command_msg_.rpm.push_back(0.0);
      } else {
        double a = rho_*pow(D_,4)*K_T_BP_;
        double b = -rho_*pow(D_,4)*K_T_BP_/prop_pitch_*surge_;
        double c = - msg->force[i];

        if(c >0){
        rpm_value_ = ((-b + sqrt(pow(b,2) + 4*a*c)) / (2*a)) * 60; /* RPM = RPS x 60 */
        rpm_value_ = -rpm_value_;
        rpm_value_ = (rpm_value_ < min_rpm_) ? min_rpm_ : rpm_value_;
        }else{

          rpm_value_ = ((-b + sqrt(pow(b,2) - 4*a*c)) / (2*a)) * 60; /* RPM = RPS x 60 */
          /* Saturate */
          rpm_value_ = (rpm_value_ > max_rpm_) ? max_rpm_ : rpm_value_;

        }
        rpm_command_msg_.rpm.push_back(rpm_value_);
      }
    }

  }
  /* Publish */
  //RCLCPP_INFO(this->get_logger(), "Publishing RPM command");
  rpm_command_pub_->publish(rpm_command_msg_);
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RPMConversion>());
  rclcpp::shutdown();
  return 0;
}
