#include <chrono>
#include <cmath>
#include <stdexcept>

#include <allocation_node.hpp>

#include <farol2_utils/angles.hpp>

using namespace std::chrono_literals;

AllocationNode::AllocationNode() : Node("allocation_node") {
	clock_ = this->get_clock();
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
		*tf_buffer_,
		get_node_base_interface(),
		get_node_logging_interface(),
		get_node_parameters_interface(),
		get_node_topics_interface());

	loadParams();
	initialiseSubscribers();
	initialisePublishers();
	initialiseTimers();
}

AllocationNode::~AllocationNode() = default;

void AllocationNode::loadParams() {
	node_frequency_ = declare_parameter<double>("node_frequency");

	const auto allocation_type = declare_parameter<std::string>("allocation.type");
	allocation_type_ = parseAllocationType(allocation_type);

	frame_prefix_ = declare_parameter<std::string>("frame_prefix", "");
	const auto apply_frame_prefix = [this](const std::string & frame) {
		if (frame_prefix_.empty() || frame.rfind(frame_prefix_, 0) == 0) {
			return frame;
		}
		return frame_prefix_ + frame;
	};

	base_frame_ = declare_parameter<std::string>("allocation.thrusters.base_frame");
	if (base_frame_.empty()) {
		throw std::runtime_error("allocation.thrusters.base_frame cannot be empty");
	}
	base_frame_ = apply_frame_prefix(base_frame_);

	thruster_frames_ = declare_parameter<std::vector<std::string>>("allocation.thrusters.frames");
	if (thruster_frames_.empty()) {
		throw std::runtime_error("allocation.thrusters.frames must contain at least one TF frame");
	}
	for (const auto & frame : thruster_frames_) {
		if (frame.empty()) {
			throw std::runtime_error("allocation.thrusters.frames cannot contain empty frame names");
		}
	}
	for (auto & frame : thruster_frames_) {
		frame = apply_frame_prefix(frame);
	}

	static_thruster_allocator_ = std::make_unique<StaticThrusterAllocator>(
		base_frame_,
		thruster_frames_);

	if (allocation_type_ == AllocationType::THRUST_RUDDER) {
		rpm_converter_ = std::make_unique<ThrusterRpmConverter>(ThrusterRpmConverter::thrusterRudder(
			0,
			{0.00000177778, 0.0, 0.0},
			{-0.00000177778, 0.0, 0.0},
			1025.0,
			0.061461915,
			0.381,
			0.4318,
			2200.0,
			-2200.0));
	} else {
		rpm_converter_ = std::make_unique<ThrusterRpmConverter>(ThrusterRpmConverter::staticCurve(
			{0.00000177778, 0.0, 0.0},
			{-0.00000177778, 0.0, 0.0},
			5000.0,
			-5000.0));
	}

	if (allocation_type_ == AllocationType::THRUST_RUDDER) {
		rudder_angle_min_ = declare_parameter<double>("allocation.rudder.limits.min") / 180.0 * M_PI;
		rudder_angle_max_ = declare_parameter<double>("allocation.rudder.limits.max") / 180.0 * M_PI;
		rudder_cm_distance_ = declare_parameter<double>("allocation.rudder.cm_distance");

		K_s_ = declare_parameter<double>("allocation.model.K_s");
		K_L_ = declare_parameter<double>("allocation.model.K_L");
		K_D0_ = declare_parameter<double>("allocation.model.K_D0");
		K_D1_ = declare_parameter<double>("allocation.model.K_D1");

		rudder_allocator_ = std::make_unique<RudderAllocator>(
			rudder_angle_min_,
			rudder_angle_max_,
			rudder_cm_distance_,
			K_s_,
			K_L_,
			K_D0_,
			K_D1_);

		open_loop_ = declare_parameter<bool>("allocation.thrusters.open_loop");
	}
}

void AllocationNode::initialiseSubscribers() {
	thrust_x_sub_ = create_subscription<std_msgs::msg::Float32>(
		TOPIC_SUB_THRUST_X,
		rclcpp::QoS(1),
		[this](std_msgs::msg::Float32::SharedPtr msg) {
			wrench_input_[0] = msg->data;
			last_received_[0] = clock_->now();
		});

	thrust_y_sub_ = create_subscription<std_msgs::msg::Float32>(
		TOPIC_SUB_THRUST_Y,
		rclcpp::QoS(1),
		[this](std_msgs::msg::Float32::SharedPtr msg) {
			wrench_input_[1] = msg->data;
			last_received_[1] = clock_->now();
		});

	thrust_z_sub_ = create_subscription<std_msgs::msg::Float32>(
		TOPIC_SUB_THRUST_Z,
		rclcpp::QoS(1),
		[this](std_msgs::msg::Float32::SharedPtr msg) {
			wrench_input_[2] = msg->data;
			last_received_[2] = clock_->now();
		});

	torque_x_sub_ = create_subscription<std_msgs::msg::Float32>(
		TOPIC_SUB_TORQUE_X,
		rclcpp::QoS(1),
		[this](std_msgs::msg::Float32::SharedPtr msg) {
			wrench_input_[3] = msg->data;
			last_received_[3] = clock_->now();
		});

	torque_y_sub_ = create_subscription<std_msgs::msg::Float32>(
		TOPIC_SUB_TORQUE_Y,
		rclcpp::QoS(1),
		[this](std_msgs::msg::Float32::SharedPtr msg) {
			wrench_input_[4] = msg->data;
			last_received_[4] = clock_->now();
		});

	torque_z_sub_ = create_subscription<std_msgs::msg::Float32>(
		TOPIC_SUB_TORQUE_Z,
		rclcpp::QoS(1),
		[this](std_msgs::msg::Float32::SharedPtr msg) {
			wrench_input_[5] = msg->data;
			last_received_[5] = clock_->now();
		});

	if (allocation_type_ == AllocationType::THRUST_RUDDER) {
		nav_state_sub_ = create_subscription<farol2_interfaces::msg::NavigationState>(
			TOPIC_SUB_NAV_STATE,
			rclcpp::QoS(1),
			[this](farol2_interfaces::msg::NavigationState::SharedPtr msg){nav_state_ = *msg;});

		mission_status_sub_ = create_subscription<std_msgs::msg::Int8>(
			TOPIC_SUB_MISSION_STATUS,
			rclcpp::QoS(1),
			[this](std_msgs::msg::Int8::SharedPtr msg){mission_status_ = msg->data;});
	}
}

void AllocationNode::initialisePublishers() {
	rpm_command_pub_ = create_publisher<farol2_interfaces::msg::ThrusterRPM>(
		"rpm_command",
		rclcpp::QoS(1));

	if (allocation_type_ == AllocationType::THRUST_RUDDER) {
		control_surface_angle_pub_ =
			create_publisher<farol2_interfaces::msg::ControlSurfaceAngle>(
				TOPIC_PUB_CONTROL_SURFACE_ANGLE,
				rclcpp::QoS(1));
	}
}

void AllocationNode::initialiseTimers() {
	tf_initialisation_timer_ = create_wall_timer(
		500ms,
		[this](){initialiseAllocationFromTF();});

	auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / node_frequency_));
	allocation_timer_ = create_wall_timer(period, [this]() { allocationTimerCallback(); });
}

AllocationNode::AllocationType AllocationNode::parseAllocationType(
	const std::string & allocation_type) const {
	if (allocation_type == "thrust_rudder" || allocation_type == "thruster_rudder" ||
			allocation_type == "thruster_rudder_allocation") {
		return AllocationType::THRUST_RUDDER;
	}

	if (allocation_type == "thrust" || allocation_type == "static_thruster_allocation") {
		return AllocationType::THRUST;
	}

	throw std::runtime_error("Unknown allocation.type '" + allocation_type + "'");
}

void AllocationNode::initialiseAllocationFromTF() {
	if (allocation_ready_ || static_thruster_allocator_ == nullptr) {
		return;
	}

	if (!static_thruster_allocator_->initialize(*tf_buffer_, *clock_, get_logger())) {
		return;
	}

	nr_thrusters_ = static_thruster_allocator_->thrusterCount();
	forces_.resize(nr_thrusters_);

	allocation_ready_ = true;
	tf_initialisation_timer_->cancel();
}

void AllocationNode::allocationTimerCallback() {
	if (!allocation_ready_) {
		return;
	}

	const auto now = clock_->now();
	const double freshness_timeout = 2.0 / static_cast<double>(node_frequency_);

	const bool thrust_x_recent = last_received_[0].has_value() && (now - *last_received_[0]).seconds() < freshness_timeout;
	const bool thrust_y_recent = last_received_[1].has_value() && (now - *last_received_[1]).seconds() < freshness_timeout;
	const bool thrust_z_recent = last_received_[2].has_value() && (now - *last_received_[2]).seconds() < freshness_timeout;
	const bool torque_x_recent = last_received_[3].has_value() && (now - *last_received_[3]).seconds() < freshness_timeout;
	const bool torque_y_recent = last_received_[4].has_value() && (now - *last_received_[4]).seconds() < freshness_timeout;
	const bool torque_z_recent = last_received_[5].has_value() && (now - *last_received_[5]).seconds() < freshness_timeout;

	if (!(thrust_x_recent || thrust_y_recent || thrust_z_recent || torque_x_recent || torque_y_recent || torque_z_recent)) {
		return;
	}

	geometry_msgs::msg::WrenchStamped body_wrench_request_msg;
	body_wrench_request_msg.header.stamp = now;
	body_wrench_request_msg.wrench.force.x = thrust_x_recent ? wrench_input_[0] : 0.0;
	body_wrench_request_msg.wrench.force.y = thrust_y_recent ? wrench_input_[1] : 0.0;
	body_wrench_request_msg.wrench.force.z = thrust_z_recent ? wrench_input_[2] : 0.0;
	body_wrench_request_msg.wrench.torque.x = torque_x_recent ? wrench_input_[3] : 0.0;
	body_wrench_request_msg.wrench.torque.y = torque_y_recent ? wrench_input_[4] : 0.0;
	body_wrench_request_msg.wrench.torque.z = torque_z_recent ? wrench_input_[5] : 0.0;

	processBodyWrenchRequest(body_wrench_request_msg);
}

void AllocationNode::processBodyWrenchRequest(const geometry_msgs::msg::WrenchStamped & msg) {
	if (!allocation_ready_) {
		RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000, "Ignoring wrench request while waiting for TF allocation.");
		return;
	}

	if (allocation_type_ == AllocationType::THRUST_RUDDER && mission_status_ == 0) {
		return;
	}

	tau_ << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
					msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;

	if (allocation_type_ == AllocationType::THRUST_RUDDER) {
		const auto rudder_result = rudder_allocator_->compute(nav_state_, tau_[5]);
		rudder_angle_ = rudder_result.rudder_angle_rad;
		rudder_x_body_drag_ = rudder_result.rudder_x_body_drag;
		tau_common_mode_ << tau_[0], 0.0, 0.0,
												0.0, 0.0, 0.0;
		forces_ = static_thruster_allocator_->allocate(tau_common_mode_);

		if (!open_loop_) {
			const auto stamp = msg.header.stamp;
			std::vector<double> forces_vec(forces_.data(), forces_.data() + forces_.size());

			rpm_converter_->setSurge(nav_state_.velocity_through_water.x);
			rpm_command_pub_->publish(rpm_converter_->convert(forces_vec, stamp));
		}

		control_surface_angle_msg_.header.stamp = msg.header.stamp;
		control_surface_angle_msg_.angle.clear();
		control_surface_angle_msg_.angle.push_back(farol2_utils::rad2deg(rudder_angle_));
		control_surface_angle_pub_->publish(control_surface_angle_msg_);
		return;
	}

	forces_ = static_thruster_allocator_->allocate(tau_);
	const auto stamp = msg.header.stamp;
	std::vector<double> forces_vec(forces_.data(), forces_.data() + forces_.size());

	rpm_converter_->setSurge(nav_state_.velocity_through_water.x);
	rpm_command_pub_->publish(rpm_converter_->convert(forces_vec, stamp));
}

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AllocationNode>());
	rclcpp::shutdown();
	return 0;
}
