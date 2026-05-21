#include "filter_node.hpp"

#include <farol2_nav/filters/pass_through.hpp>
#include <farol2_nav/filters/position_current_ekf.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

FilterNode::FilterNode()
  : Node("filter_node")
{
  load_params();
  initialise_publishers();
  initialise_subscribers();
  build_pipeline();
  initialise_timer();
}

void FilterNode::load_params()
{
  node_frequency_ = declare_parameter<double>("node_frequency", 10.0);
  publish_all_steps_ = declare_parameter<bool>("publish_all_steps", true);
  filters_ = declare_parameter<std::vector<std::string>>("filters", std::vector<std::string>{});

  imu_timeout_s_ = declare_parameter<double>("timeouts.imu", 1.0);
  navsat_timeout_s_ = declare_parameter<double>("timeouts.navsat", 2.0);
  utm_timeout_s_ = declare_parameter<double>("timeouts.utm", 2.0);
  rpm_timeout_s_ = declare_parameter<double>("timeouts.rpm", 1.0);
}

void FilterNode::initialise_publishers()
{
  // Final state publisher
  final_state_pub_ = create_publisher<farol2_interfaces::msg::NavigationState>(TOPIC_PUB_STATE, rclcpp::QoS(10));
}

void FilterNode::initialise_subscribers()
{
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    TOPIC_SUB_IMU, rclcpp::QoS(10),
    [this](sensor_msgs::msg::Imu::SharedPtr msg) {
      snapshot_.imu = std::move(msg);
      snapshot_.imu_stamp = now();
    });

  gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    TOPIC_SUB_GNSS, rclcpp::QoS(10),
    [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      snapshot_.gnss = std::move(msg);
      snapshot_.gnss_stamp = now();
    });

  utm_ned_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    TOPIC_SUB_UTM_NED, rclcpp::QoS(10),
    [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
      snapshot_.utm_ned = std::move(msg);
      snapshot_.utm_ned_stamp = now();
    });

  velocity_over_ground_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    TOPIC_SUB_VELOCITY_OVER_GROUND, rclcpp::QoS(10),
    [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
      snapshot_.velocity_over_ground = std::move(msg);
      snapshot_.velocity_over_ground_stamp = now();
    });

  velocity_through_water_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    TOPIC_SUB_VELOCITY_THROUGH_WATER, rclcpp::QoS(10),
    [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
      snapshot_.velocity_through_water = std::move(msg);
      snapshot_.velocity_through_water_stamp = now();
    });

  depth_sub_ = create_subscription<std_msgs::msg::Float32>(
    TOPIC_SUB_DEPTH, rclcpp::QoS(10),
    [this](std_msgs::msg::Float32::SharedPtr msg) {
      snapshot_.depth = std::move(msg);
      snapshot_.depth_stamp = now();
    });

  altimeter_sub_ = create_subscription<std_msgs::msg::Float32>(
    TOPIC_SUB_ALTIMETER, rclcpp::QoS(10),
    [this](std_msgs::msg::Float32::SharedPtr msg) {
      snapshot_.altimeter = std::move(msg);
      snapshot_.altimeter_stamp = now();
    });

  rudder_sub_ = create_subscription<std_msgs::msg::Float32>(
    TOPIC_SUB_RUDDER_ANGLE, rclcpp::QoS(10),
    [this](std_msgs::msg::Float32::SharedPtr msg) {
      snapshot_.rudder_angle = std::move(msg);
      snapshot_.rudder_stamp = now();
    });

  rpm_sub_ = create_subscription<farol2_allocation::msg::ThrusterRPM>(
    TOPIC_SUB_RPM_COMMAND, rclcpp::QoS(10),
    [this](farol2_allocation::msg::ThrusterRPM::SharedPtr msg) {
      snapshot_.rpm_command = std::move(msg);
      snapshot_.rpm_stamp = now();
    });
}

void FilterNode::build_pipeline()
{
  pipeline_.clear();

  auto pass_through = std::make_unique<farol2_nav::filters::PassThroughFilter>();
  pass_through->configure(*this);
  pipeline_.push_back(std::move(pass_through));

  for (const auto & key : filters_) {
    std::unique_ptr<farol2_nav::filters::BaseFilter> filter;
    if (key == "position_current_ekf") {
      filter = std::make_unique<farol2_nav::filters::PositionCurrentEkfFilter>();
    // Here add additional filters with else if blocks, following the pattern above. For example:
    // } else if (key == "your_filter_name") {
    //   filter = std::make_unique<farol2_nav::filters::YourFilter>();
    } else if (key == "") {
        continue; // here so warning is not printing always when filters list is empty (check comment on nav.yaml)
    }  else {
      RCLCPP_WARN(get_logger(), "Unknown filter key '%s', skipping", key.c_str());
      continue;
    }
    filter->configure(*this);
    pipeline_.push_back(std::move(filter));
  }

  stage_pubs_.clear();
  for (size_t i = 0; i + 1U < pipeline_.size(); ++i) {
    const auto & filter_name = pipeline_[i]->name();
    stage_pubs_.push_back(
      create_publisher<farol2_interfaces::msg::NavigationState>(filter_name + "/state", rclcpp::QoS(10)));
  }
}

void FilterNode::initialise_timer()
{
  const auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / std::max(0.1, node_frequency_)));
  timer_ = create_wall_timer(period_ns, [this]() { on_timer(); });
}

bool FilterNode::is_fresh(const rclcpp::Time & stamp, double timeout_s) const
{
  if (stamp.nanoseconds() == 0) {
    return false;
  }
  return (now() - stamp).seconds() <= timeout_s;
}

void FilterNode::on_timer()
{
  const auto tick_stamp = now();
  const double now_s = tick_stamp.seconds();
  const double dt_s = (last_tick_s_ > 0.0) ? (now_s - last_tick_s_) : (1.0 / std::max(0.1, node_frequency_));
  last_tick_s_ = now_s;

  if (!is_fresh(snapshot_.imu_stamp, imu_timeout_s_)) {
    snapshot_.imu.reset();
  }
  if (!is_fresh(snapshot_.gnss_stamp, navsat_timeout_s_)) {
    snapshot_.gnss.reset();
  }
  if (!is_fresh(snapshot_.utm_ned_stamp, utm_timeout_s_)) {
    snapshot_.utm_ned.reset();
  }
  if (!is_fresh(snapshot_.rpm_stamp, rpm_timeout_s_)) {
    snapshot_.rpm_command.reset();
  }

  // Every cycle starts from a clean state. sample_and_hold runs first and repopulates it.
  state_ = farol2_nav::filters::State{};

  // run filter pipeline 
  for (size_t i = 0; i < pipeline_.size(); ++i) {
    auto & filter = pipeline_[i];
    filter->update(dt_s, snapshot_, state_);

    // Only intermediate stages use stage publishers.
    if (publish_all_steps_ && (i + 1U < pipeline_.size())) {
      if (i < stage_pubs_.size()) {
        fill_state_msg(tick_stamp);
        stage_pubs_[i]->publish(msg_);
      }
    }
  }

  // Final output is always published after the full pipeline is applied.
  fill_state_msg(tick_stamp);
  final_state_pub_->publish(msg_);
}

void FilterNode::fill_state_msg(const rclcpp::Time & stamp)
{
  // Reuse a single message instance to avoid per-tick temporary allocations.
  msg_.header.stamp = stamp;
  msg_.header.frame_id = "";
  msg_.global_position.latitude = state_.latitude_deg;
  msg_.global_position.longitude = state_.longitude_deg;

  msg_.utm_position.northing = state_.northing_m;
  msg_.utm_position.easting = state_.easting_m;
  msg_.utm_position.utm_zone = static_cast<uint32_t>(std::max(0, state_.utm_zone));
  msg_.utm_position.northp = (state_.latitude_deg >= 0.0);

  msg_.depth = state_.depth_m;
  msg_.altimeter = state_.altimeter_m;
  msg_.altitude_ellipsoidal = state_.altitude_ellipsoidal_m;
  msg_.local_datum.altitude = state_.local_datum_altitude_m;

  msg_.velocity_over_ground_body.x = state_.velocity_over_ground_body_mps(0);
  msg_.velocity_over_ground_body.y = state_.velocity_over_ground_body_mps(1);
  msg_.velocity_over_ground_body.z = state_.velocity_over_ground_body_mps(2);
  msg_.velocity_through_water_body.x = state_.velocity_through_water_body_mps(0);
  msg_.velocity_through_water_body.y = state_.velocity_through_water_body_mps(1);
  msg_.velocity_through_water_body.z = state_.velocity_through_water_body_mps(2);
  msg_.velocity_over_ground_ned.x = state_.velocity_over_ground_ned_mps(0);
  msg_.velocity_over_ground_ned.y = state_.velocity_over_ground_ned_mps(1);
  msg_.velocity_over_ground_ned.z = state_.velocity_over_ground_ned_mps(2);
  msg_.velocity_through_water_ned.x = state_.velocity_through_water_ned_mps(0);
  msg_.velocity_through_water_ned.y = state_.velocity_through_water_ned_mps(1);
  msg_.velocity_through_water_ned.z = state_.velocity_through_water_ned_mps(2);
  msg_.course_over_ground = state_.course_over_ground_deg;
  msg_.current_velocity_inertial.x = state_.current_velocity_inertial_mps(0);
  msg_.current_velocity_inertial.y = state_.current_velocity_inertial_mps(1);
  msg_.current_velocity_inertial.z = state_.current_velocity_inertial_mps(2);

  msg_.attitude.x = state_.attitude_deg(0);
  msg_.attitude.y = state_.attitude_deg(1);
  msg_.attitude.z = state_.attitude_deg(2);
  msg_.angular_velocity.x = state_.angular_velocity_dps(0);
  msg_.angular_velocity.y = state_.angular_velocity_dps(1);
  msg_.angular_velocity.z = state_.angular_velocity_dps(2);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilterNode>());
  rclcpp::shutdown();
  return 0;
}
