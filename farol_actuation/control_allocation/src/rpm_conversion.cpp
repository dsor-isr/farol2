#include <rpm_conversion.hpp>

/* Constructor */
RPMConversion::RPMConversion() : Node("rpm_conversion", 
                                      rclcpp::NodeOptions()
                                        .allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true)) {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
}

/* Destructor */
RPMConversion::~RPMConversion() {

}

/**
 * @brief Initialise Subscribers
 */
void RPMConversion::initialiseSubscribers() {
  thruster_force_sub_ = create_subscription<control_allocation::msg::ThrusterForce>(
                          get_parameter("actuation.rpm_conversion.topics.subscribers.thruster_force").as_string(), 
                          1, std::bind(&RPMConversion::thrusterForceCallback, this, std::placeholders::_1));

  nav_state_sub_ = create_subscription<farol_msgs::msg::NavigationState>(
                    get_parameter("actuation.rpm_conversion.topics.subscribers.nav_state").as_string(), 
                    1, std::bind(&RPMConversion::navStateCallback, this, std::placeholders::_1));

  return;
}

/**
 * @brief Load parameters
 * Thruster configuration parameters are loaded under some assumptions.
 * In the future, if ROS2 enables native parameter loading for dicts and 
 * other complex types, this method should be adapted for further robustness.
 */
void RPMConversion::loadParams() {
  mode_ = get_parameter("actuation.thrusters.rpm_conversion.mode").as_int();

  if (mode_ != 0 && mode_ != 1) {
    mode_ = 1;
  }

  /* Thruster parameters */
  rho_ = get_parameter("actuation.thrusters.rpm_conversion.mode0.rho").as_double();
  K_T_BP_ = get_parameter("actuation.thrusters.rpm_conversion.mode0.K_T_BP").as_double();
  prop_pitch_ = get_parameter("actuation.thrusters.rpm_conversion.mode0.prop_pitch").as_double();
  D_ = get_parameter("actuation.thrusters.rpm_conversion.mode0.D").as_double();

  /* Thruster coefficients */
  coef_fwd_ = get_parameter("actuation.thrusters.rpm_conversion.mode1.coef_fwd").as_double_array();
  coef_bwd_ = get_parameter("actuation.thrusters.rpm_conversion.mode1.coef_bwd").as_double_array();

  /* Minimum and maximum RPM allowed */
  max_rpm_ = get_parameter("actuation.thrusters.rpm_conversion.max_rpm").as_double();
  min_rpm_ = get_parameter("actuation.thrusters.rpm_conversion.min_rpm").as_double();
}

/**
 * @brief Initialise Publishers
 */
void RPMConversion::initialisePublishers() {
  rpm_command_pub_ = create_publisher<control_allocation::msg::ThrusterRPM>(
                        get_parameter("actuation.rpm_conversion.topics.publishers.rpm_command").as_string(), 1);
}

/**
 * @brief Initialise Services
 */
void RPMConversion::initialiseServices() {
  /* Service servers */
  /* ... */

  /* service clients */
  /* ... */

  return;
}

/**
 * @brief Compute force for each thruster based on body wrench (force and torque) request.
 */
void RPMConversion::thrusterForceCallback(const control_allocation::msg::ThrusterForce &msg) {
  /* Create message to publish thruster RPM */
  rpm_command_msg_.header.stamp = msg.header.stamp;
  rpm_command_msg_.rpm = {};
  
  /* Convert force to RPM */
  for (int i = 0; i < (int)msg.force.size(); i++) {
    if (mode_ == 1) {
      if (msg.force[i] == 0.0) {
        rpm_command_msg_.rpm.push_back(0.0);
      } else if (msg.force[i] > 0.0) {
        rpm_value_ = (-coef_fwd_[1] + sqrt(coef_fwd_[1] * coef_fwd_[1] - 4 * coef_fwd_[0] * (coef_fwd_[2] - msg.force[i]))) / (2 * coef_fwd_[0]);

        /* Saturate */
        rpm_value_ = (rpm_value_ > max_rpm_) ? max_rpm_ : rpm_value_;

        rpm_command_msg_.rpm.push_back(rpm_value_);
      } else {
        rpm_value_ = (-coef_bwd_[1] + sqrt(coef_bwd_[1] * coef_bwd_[1] - 4 * coef_bwd_[0] * (coef_bwd_[2] - msg.force[i]))) / (2 * coef_bwd_[0]);

        /* Saturate */
        rpm_value_ = (rpm_value_ < min_rpm_) ? min_rpm_ : rpm_value_;

        rpm_command_msg_.rpm.push_back(rpm_value_);
      }
    } else if (mode_ == 0) {
      if (msg.force[i] == 0.0) {
        rpm_command_msg_.rpm.push_back(0.0);
      } else {
        double a = rho_*pow(D_,4)*K_T_BP_;
        double b = -rho_*pow(D_,4)*K_T_BP_/prop_pitch_*surge_;
        double c = - msg.force[i];

        rpm_value_ = ((-b + sqrt(pow(b,2) - 4*a*c)) / (2*a)) * 60; /* RPM = RPS x 60 */

        /* Saturate */
        rpm_value_ = (rpm_value_ > max_rpm_) ? max_rpm_ : rpm_value_;

        rpm_command_msg_.rpm.push_back(rpm_value_);
      }
    }

  /* Publish */
  rpm_command_pub_->publish(rpm_command_msg_);
  }
}

/**
 * @brief Callback for navigation state.
 */
void RPMConversion::navStateCallback(const farol_msgs::msg::NavigationState &msg) {
  surge_ = msg.body_velocity_fluid.x;
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
