#include "filter_node.hpp"

#include <farol2_nav/filters/sample_and_hold.hpp>
#include <farol2_nav/filters/position_current_ekf.hpp>
#include <farol2_nav/filters/yaw_rate_ekf.hpp>
#include <farol2_nav/filters/asv_dynamics_model.hpp>
#include <farol2_utils/angles.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
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
  measurements_ = declare_parameter<std::vector<std::string>>("measurements", std::vector<std::string>{"gnss", "imu"});
}

void FilterNode::initialise_publishers()
{
  // Final state publisher
  final_state_pub_ = create_publisher<farol2_interfaces::msg::NavigationState>(TOPIC_PUB_STATE, rclcpp::QoS(10));
}

void FilterNode::initialise_subscribers()
{
  const auto is_active = [this](const std::string & measurement) {
      return std::find(measurements_.begin(), measurements_.end(), measurement) !=
        measurements_.end();
    };

  if (is_active("imu")) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      TOPIC_SUB_IMU, rclcpp::QoS(10),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        snapshot_.imu = std::move(msg);
      });
  }

  if (is_active("gnss")) {
    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      TOPIC_SUB_GNSS, rclcpp::QoS(10),
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        snapshot_.gnss = std::move(msg);
      });
  }

  if (is_active("utm_ned")) {
    utm_ned_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      TOPIC_SUB_UTM_NED, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        snapshot_.utm_ned = std::move(msg);
      });
  }

  if (is_active("velocity_over_ground")) {
    velocity_over_ground_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      TOPIC_SUB_VELOCITY_OVER_GROUND, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        snapshot_.velocity_over_ground = std::move(msg);
      });
  }

  if (is_active("velocity_through_water")) {
    velocity_through_water_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      TOPIC_SUB_VELOCITY_THROUGH_WATER, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        snapshot_.velocity_through_water = std::move(msg);
      });
  }

  if (is_active("depth")) {
    depth_sub_ = create_subscription<farol2_interfaces::msg::Depth>(
      TOPIC_SUB_DEPTH, rclcpp::QoS(10),
      [this](farol2_interfaces::msg::Depth::SharedPtr msg) {
        snapshot_.depth = std::move(msg);
      });
  }

  if (is_active("altimeter")) {
    altimeter_sub_ = create_subscription<sensor_msgs::msg::Range>(
      TOPIC_SUB_ALTIMETER, rclcpp::QoS(10),
      [this](sensor_msgs::msg::Range::SharedPtr msg) {
        snapshot_.altimeter = std::move(msg);
      });
  }

  if (is_active("control_surface_deflection")) {
    control_surface_deflection_sub_ =
      create_subscription<farol2_interfaces::msg::ControlSurfaceDeflection>(
      TOPIC_SUB_CONTROL_SURFACE_DEFLECTION, rclcpp::QoS(10),
      [this](farol2_interfaces::msg::ControlSurfaceDeflection::SharedPtr msg) {
        snapshot_.control_surface_deflection = std::move(msg);
      });
  }

  if (is_active("thruster_rpm")) {
    thruster_rpm_sub_ = create_subscription<farol2_interfaces::msg::ThrusterRPM>(
      TOPIC_SUB_THRUSTER_RPM, rclcpp::QoS(10),
      [this](farol2_interfaces::msg::ThrusterRPM::SharedPtr msg) {
        snapshot_.thruster_rpm = std::move(msg);
      });
  }
}

void FilterNode::build_pipeline()
{
  pipeline_.clear();

  auto sample_and_hold = std::make_unique<farol2_nav::filters::SampleAndHoldFilter>();
  sample_and_hold->configure(*this);
  pipeline_.push_back(std::move(sample_and_hold));

  for (const auto & key : filters_) {
    std::unique_ptr<farol2_nav::filters::BaseFilter> filter;
    if (key == "position_current_ekf") {
      filter = std::make_unique<farol2_nav::filters::PositionCurrentEkfFilter>();
    } else if (key == "yaw_rate_ekf") {
      filter = std::make_unique<farol2_nav::filters::YawRateEkfFilter>();
    } else if (key == "asv_dynamics_model") {
      filter = std::make_unique<farol2_nav::filters::AsvDynamicsModelFilter>();
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

void FilterNode::on_timer()
{
  const auto tick_stamp = now();
  const double now_s = tick_stamp.seconds();
  const double dt_s = (last_tick_s_ > 0.0) ? (now_s - last_tick_s_) : (1.0 / std::max(0.1, node_frequency_));
  last_tick_s_ = now_s;

  // Every cycle starts from a clean state. sample_and_hold runs first and repopulates it.
  state_ = farol2_nav::filters::State{};

  // add here the measurement pre processing -> convert to base_link

  // run filter pipeline 
  for (size_t i = 0; i < pipeline_.size(); ++i) {
    auto & filter = pipeline_[i];
    filter->compute(dt_s, snapshot_, state_);

    // sample_and_hold is the first filter and gates startup publication.
    if (i == 0U && !filter->initialized()) {
      snapshot_ = farol2_nav::filters::MeasurementSnapshot{};
      return;
    }

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

  // Flush the consumed measurements so the next tick only sees new input.
  snapshot_ = farol2_nav::filters::MeasurementSnapshot{};

}

void FilterNode::fill_state_msg(const rclcpp::Time & stamp)
{
  // Reuse a single message instance to avoid per-tick temporary allocations.
  msg_.header.stamp = stamp;
  msg_.header.frame_id = "";
  msg_.global_position.latitude = state_.latitude;
  msg_.global_position.longitude = state_.longitude;

  msg_.utm_position.northing = state_.northing;
  msg_.utm_position.easting = state_.easting;
  msg_.utm_position.utm_zone = static_cast<uint32_t>(std::max(0, state_.utm_zone));
  msg_.utm_position.northp = (state_.latitude >= 0.0);

  msg_.depth = state_.depth;
  msg_.altimeter = state_.altimeter;
  msg_.altitude_wgs84 = state_.altitude_wgs84;
  msg_.altitude_local_datum.altitude = state_.altitude_local_datum_altitude;

  msg_.velocity_over_ground_body.x = state_.velocity_over_ground_body(0);
  msg_.velocity_over_ground_body.y = state_.velocity_over_ground_body(1);
  msg_.velocity_over_ground_body.z = state_.velocity_over_ground_body(2);
  msg_.velocity_through_water_body.x = state_.velocity_through_water_body(0);
  msg_.velocity_through_water_body.y = state_.velocity_through_water_body(1);
  msg_.velocity_through_water_body.z = state_.velocity_through_water_body(2);
  msg_.velocity_over_ground_ned.x = state_.velocity_over_ground_ned(0);
  msg_.velocity_over_ground_ned.y = state_.velocity_over_ground_ned(1);
  msg_.velocity_over_ground_ned.z = state_.velocity_over_ground_ned(2);
  msg_.velocity_through_water_ned.x = state_.velocity_through_water_ned(0);
  msg_.velocity_through_water_ned.y = state_.velocity_through_water_ned(1);
  msg_.velocity_through_water_ned.z = state_.velocity_through_water_ned(2);
  msg_.course_over_ground = state_.course_over_ground;
  msg_.current_velocity_ned.x = state_.current_velocity_ned(0);
  msg_.current_velocity_ned.y = state_.current_velocity_ned(1);
  msg_.current_velocity_ned.z = state_.current_velocity_ned(2);
  msg_.current_speed = state_.current_velocity_ned.head<2>().norm();
  msg_.current_direction = farol2_utils::rad2deg(farol2_utils::wrapTo2Pi(std::atan2(state_.current_velocity_ned(1), state_.current_velocity_ned(0))));

  msg_.attitude.roll = state_.attitude(0);
  msg_.attitude.pitch = state_.attitude(1);
  msg_.attitude.yaw = farol2_utils::rad2deg(farol2_utils::wrapTo2Pi(farol2_utils::deg2rad( state_.attitude(2))));
  msg_.angular_velocity.x = state_.angular_velocity(0);
  msg_.angular_velocity.y = state_.angular_velocity(1);
  msg_.angular_velocity.z = state_.angular_velocity(2);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilterNode>());
  rclcpp::shutdown();
  return 0;
}
