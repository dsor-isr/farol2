#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <stdlib.h>
#include <vector>
#include <optional>

/* ROS specific includes */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

/* Include the message for publishing the path information */
#include "farol2_planning/msg/path_data.hpp"
#include "farol2_interfaces/msg/navigation_state.hpp"
#include "farol2_interfaces/msg/state_console.hpp"

/* Farol gimicks library for reading configuration paramters */
// #include <farol_gimmicks_library/FarolGimmicks.h>

/* Include the algorithms library for the Paths */
#include "Path.h"
#include "PathSection.h"
#include "Arc2D.h"
#include "Bernoulli.h"
#include "Bezier.h"
#include "Circle2D.h"
#include "Line.h"
#include "ConstRabbitSpeed.h"
#include "ConstVehicleSpeed.h"
#include "BezierVehicleSpeed.h"

/* Include the generated services for the farol2_planning */
#include "farol2_planning/srv/reset_path.hpp"
#include "farol2_planning/srv/set_mode.hpp"
#include "farol2_planning/srv/spawn_arc2_d.hpp"
#include "farol2_planning/srv/spawn_bernoulli.hpp"
#include "farol2_planning/srv/spawn_circle2_d.hpp"
#include "farol2_planning/srv/spawn_line.hpp"
#include "farol2_planning/srv/spawn_bezier.hpp"

/* Services for setting the speed profile of the vehicle*/
#include "farol2_planning/srv/set_const_speed.hpp"
#include "farol2_planning/srv/set_bezier_speed.hpp"

// Topic/service names (short form, remapped in launch file)
static constexpr char TOPIC_SUB_GAMMA[] = "gamma";
static constexpr char TOPIC_SUB_VEHICLE_STATE[] = "vehicle_state";
static constexpr char TOPIC_PUB_PATH_DATA[] = "path_data";
static constexpr char TOPIC_PUB_VIRTUAL_TARGET_STATE[] = "virtual_target_state";
static constexpr char SERVICE_RESET_PATH[] = "reset_path";
static constexpr char SERVICE_SET_MODE[] = "set_mode";
static constexpr char SERVICE_ARC2D_PATH[] = "arc2d_path";
static constexpr char SERVICE_BERNOULLI_PATH[] = "bernoulli_path";
static constexpr char SERVICE_CIRCLE2D_PATH[] = "circle2d_path";
static constexpr char SERVICE_LINE_PATH[] = "line_path";
static constexpr char SERVICE_CONST_RABBIT_SPEED[] = "speed_const_rabbit_speed";
static constexpr char SERVICE_CONST_VEHICLE_SPEED[] = "speed_const_vehicle_speed";
static constexpr char SERVICE_BEZIER_VEHICLE_SPEED[] = "speed_bezier_vehicle_speed";
static constexpr char SERVICE_BEZIER_PATH[] = "bezier_path";
/** 
 *  @brief     Implementation of the PathNode. Creates a Path, adds elements 
 *             to the path and publishes the path data when listening to the 
 *             path parameter gamma
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class PathNode : public rclcpp::Node {
  public:
    
    /**
     * @brief  Constructor of the PathNode class
     */
    PathNode();

    /**
     * @brief  Destructor of the PathNode class
     */
    ~PathNode();

  private:

    /** 
     * @brief Path structure that will have all the logic 
     */
    std::optional<double> gamma_; // The current gamma being published
    Path * path_{NULL}; // A pointer to the path object

    double Tf_val_{0.0}; // Additional variable needed for Bezier Implementation

    /**
     * @brief Frame_id for messages 
     */
    std::string frame_id_;


    /**
     * @brief Auxiliar variables to store the current vehicle position - useful
     * if we want the closest point to the path 
     */
    Eigen::Vector3d vehicle_pos_;
    bool closer_point_mode_{false};

    /**
     * @brief ROS Subscribers
     */ 
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gamma_sub_;
    rclcpp::Subscription<farol2_interfaces::msg::NavigationState>::SharedPtr vehicle_sub_;

    /** 
     * @brief ROS Publishers
     */
    rclcpp::Publisher<farol2_planning::msg::PathData>::SharedPtr path_pub_;
    rclcpp::Publisher<farol2_interfaces::msg::StateConsole>::SharedPtr virtual_target_pub_;

    /** 
     * @brief ROS Services 
     */
    rclcpp::Service<farol2_planning::srv::ResetPath>::SharedPtr reset_path_srv_;
    rclcpp::Service<farol2_planning::srv::SetMode>::SharedPtr set_mode_srv_;
    rclcpp::Service<farol2_planning::srv::SpawnArc2D>::SharedPtr arc2d_srv_;
    rclcpp::Service<farol2_planning::srv::SpawnBernoulli>::SharedPtr bernoulli_srv_;
    rclcpp::Service<farol2_planning::srv::SpawnBezier>::SharedPtr bezier_srv_;
    rclcpp::Service<farol2_planning::srv::SpawnCircle2D>::SharedPtr circle2D_srv_;
    rclcpp::Service<farol2_planning::srv::SpawnLine>::SharedPtr line_srv_;
    rclcpp::Service<farol2_planning::srv::SetConstSpeed>::SharedPtr vehicle_const_speed_srv_;
    rclcpp::Service<farol2_planning::srv::SetConstSpeed>::SharedPtr rabbit_const_speed_srv_;
    rclcpp::Service<farol2_planning::srv::SetBezierSpeed>::SharedPtr vehicle_bezier_speed_srv_;

    /**
     * @brief ROS Timer
     */
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock clock_;
    double node_frequency_;

    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();
    void initializeTimer();
    void loadParams();

    /**
     * @brief Method where is the logic for publishing the path information 
     */
    void timerCallback(); 

    /** 
     * @brief Callbacks 
     */
    void gammaCallback(const std_msgs::msg::Float32 &msg);
    void vehicleStateCallback(const farol2_interfaces::msg::NavigationState &msg);

    /** 
     * @brief Services Callbacks 
     */
    bool ResetPathService(const std::shared_ptr<farol2_planning::srv::ResetPath::Request> req, std::shared_ptr<farol2_planning::srv::ResetPath::Response> res);
    bool SetModeService(const std::shared_ptr<farol2_planning::srv::SetMode::Request> req, std::shared_ptr<farol2_planning::srv::SetMode::Response> res);
    bool Arc2DService(const std::shared_ptr<farol2_planning::srv::SpawnArc2D::Request> req, std::shared_ptr<farol2_planning::srv::SpawnArc2D::Response> res); 
    bool BernoulliService(const std::shared_ptr<farol2_planning::srv::SpawnBernoulli::Request> req, std::shared_ptr<farol2_planning::srv::SpawnBernoulli::Response> res);
    bool Circle2DService(const std::shared_ptr<farol2_planning::srv::SpawnCircle2D::Request> req, std::shared_ptr<farol2_planning::srv::SpawnCircle2D::Response> res);
    bool LineService(const std::shared_ptr<farol2_planning::srv::SpawnLine::Request> req, std::shared_ptr<farol2_planning::srv::SpawnLine::Response> res);
    bool RabbitConstSpeedService(const std::shared_ptr<farol2_planning::srv::SetConstSpeed::Request> req, std::shared_ptr<farol2_planning::srv::SetConstSpeed::Response> res);
    bool VehicleConstSpeedService(const std::shared_ptr<farol2_planning::srv::SetConstSpeed::Request> req, std::shared_ptr<farol2_planning::srv::SetConstSpeed::Response> res);
    bool VehicleBezierSpeedService(const std::shared_ptr<farol2_planning::srv::SetBezierSpeed::Request> req, std::shared_ptr<farol2_planning::srv::SetBezierSpeed::Response> res); 
    bool BezierService(const std::shared_ptr<farol2_planning::srv::SpawnBezier::Request> req, std::shared_ptr<farol2_planning::srv::SpawnBezier::Response> res);

    /** 
     * @brief Auxiliar method to be called inside the callbacks
     */
    bool loadSectionIntoPath(PathSection * section);
    bool loadSpeedIntoPath(Speed * speed); 
};
