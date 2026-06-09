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

	base_frame_ = declare_parameter<std::string>("allocation.thrusters.base_frame");
	if (base_frame_.empty()) {
		throw std::runtime_error("allocation.thrusters.base_frame cannot be empty");
	}

	const auto thrust_axis = declare_parameter<std::vector<double>>("allocation.thrusters.thrust_axis");
	if (thrust_axis.size() != 3) {
		throw std::runtime_error("allocation.thrusters.thrust_axis must have exactly 3 values");
	}
	thrust_axis_ << thrust_axis[0], thrust_axis[1], thrust_axis[2];
	if (thrust_axis_.norm() <= 1e-9) {
		throw std::runtime_error("allocation.thrusters.thrust_axis cannot have near-zero norm");
	}
	thrust_axis_.normalize();

	thruster_frames_ = declare_parameter<std::vector<std::string>>("allocation.thrusters.frames");
	if (thruster_frames_.empty()) {
		throw std::runtime_error("allocation.thrusters.frames must contain at least one TF frame");
	}
	for (const auto & frame : thruster_frames_) {
		if (frame.empty()) {
			throw std::runtime_error("allocation.thrusters.frames cannot contain empty frame names");
		}
	}

	static_thruster_allocator_ = std::make_unique<StaticThrusterAllocator>(
		base_frame_,
		thrust_axis_,
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
	body_wrench_request_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
		TOPIC_SUB_BODY_WRENCH_REQUEST,
		rclcpp::QoS(1),
		[this](geometry_msgs::msg::WrenchStamped::SharedPtr msg){bodyWrenchRequestCallback(msg);});

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
		rudder_command_pub_ = create_publisher<std_msgs::msg::Float32>(
			TOPIC_PUB_RUDDER_COMMAND,
			rclcpp::QoS(1));
	}
}

void AllocationNode::initialiseTimers() {
	tf_initialisation_timer_ = create_wall_timer(
		500ms,
		[this](){initialiseAllocationFromTF();});
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

void AllocationNode::bodyWrenchRequestCallback(geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
	if (!allocation_ready_) {
		RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000, "Ignoring wrench request while waiting for TF allocation.");
		return;
	}

	if (allocation_type_ == AllocationType::THRUST_RUDDER && mission_status_ == 0) {
		return;
	}

	tau_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
					msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

	if (allocation_type_ == AllocationType::THRUST_RUDDER) {
		const auto rudder_result = rudder_allocator_->compute(nav_state_, tau_[5]);
		rudder_angle_ = rudder_result.rudder_angle_rad;
		rudder_x_body_drag_ = rudder_result.rudder_x_body_drag;
		tau_common_mode_ << tau_[0], 0.0, 0.0,
												0.0, 0.0, 0.0;
		forces_ = static_thruster_allocator_->allocate(tau_common_mode_);

		if (!open_loop_) {
			const auto stamp = clock_->now();
			std::vector<double> forces_vec(forces_.data(), forces_.data() + forces_.size());

			rpm_converter_->setSurge(nav_state_.velocity_through_water_body.x);
			rpm_command_pub_->publish(rpm_converter_->convert(forces_vec, stamp));
		}

		rudder_command_msg_.data = farol2_utils::rad2deg(rudder_angle_);
		rudder_command_pub_->publish(rudder_command_msg_);
		return;
	}

	forces_ = static_thruster_allocator_->allocate(tau_);
	const auto stamp = clock_->now();
	std::vector<double> forces_vec(forces_.data(), forces_.data() + forces_.size());

	rpm_converter_->setSurge(nav_state_.velocity_through_water_body.x);
	rpm_command_pub_->publish(rpm_converter_->convert(forces_vec, stamp));
}

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AllocationNode>());
	rclcpp::shutdown();
	return 0;
}