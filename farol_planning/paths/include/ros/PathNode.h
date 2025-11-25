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
#include "paths/msg/path_data.hpp"
#include "farol_msgs/msg/navigation_state.hpp"
#include "farol_msgs/msg/state_console.hpp"

/* Farol gimicks library for reading configuration paramters */
// #include <farol_gimmicks_library/FarolGimmicks.h>

/* Include the algorithms library for the Paths */
#include "Path.h"
#include "PathSection.h"
#include "Arc2D.h"
#include "Bernoulli.h"
#include "Circle2D.h"
#include "Line.h"
#include "ConstRabbitSpeed.h"
#include "ConstVehicleSpeed.h"

/* Include the generated services for the paths */
#include "paths/srv/reset_path.hpp"
#include "paths/srv/set_mode.hpp"
#include "paths/srv/spawn_arc2_d.hpp"
#include "paths/srv/spawn_bernoulli.hpp"
#include "paths/srv/spawn_circle2_d.hpp"
#include "paths/srv/spawn_line.hpp"

/* Services for setting the speed profile of the vehicle*/
#include "paths/srv/set_const_speed.hpp"


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
    rclcpp::Subscription<farol_msgs::msg::NavigationState>::SharedPtr vehicle_sub_;

    /** 
     * @brief ROS Publishers
     */
    rclcpp::Publisher<paths::msg::PathData>::SharedPtr path_pub_;
    rclcpp::Publisher<farol_msgs::msg::StateConsole>::SharedPtr virtual_target_pub_;

    /** 
     * @brief ROS Services 
     */
    rclcpp::Service<paths::srv::ResetPath>::SharedPtr reset_path_srv_;
    rclcpp::Service<paths::srv::SetMode>::SharedPtr set_mode_srv_;
    rclcpp::Service<paths::srv::SpawnArc2D>::SharedPtr arc2d_srv_;
    rclcpp::Service<paths::srv::SpawnBernoulli>::SharedPtr bernoulli_srv_;
    rclcpp::Service<paths::srv::SpawnCircle2D>::SharedPtr circle2D_srv_;
    rclcpp::Service<paths::srv::SpawnLine>::SharedPtr line_srv_;
    rclcpp::Service<paths::srv::SetConstSpeed>::SharedPtr vehicle_const_speed_srv_;
    rclcpp::Service<paths::srv::SetConstSpeed>::SharedPtr rabbit_const_speed_srv_;

    /**
     * @brief ROS Timer
     */
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Clock clock_;

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
    void vehicleStateCallback(const farol_msgs::msg::NavigationState &msg);

    /** 
     * @brief Services Callbacks 
     */
    bool ResetPathService(const std::shared_ptr<paths::srv::ResetPath::Request> req, std::shared_ptr<paths::srv::ResetPath::Response> res);
    bool SetModeService(const std::shared_ptr<paths::srv::SetMode::Request> req, std::shared_ptr<paths::srv::SetMode::Response> res);
    bool Arc2DService(const std::shared_ptr<paths::srv::SpawnArc2D::Request> req, std::shared_ptr<paths::srv::SpawnArc2D::Response> res); 
    bool BernoulliService(const std::shared_ptr<paths::srv::SpawnBernoulli::Request> req, std::shared_ptr<paths::srv::SpawnBernoulli::Response> res);
    bool Circle2DService(const std::shared_ptr<paths::srv::SpawnCircle2D::Request> req, std::shared_ptr<paths::srv::SpawnCircle2D::Response> res);
    bool LineService(const std::shared_ptr<paths::srv::SpawnLine::Request> req, std::shared_ptr<paths::srv::SpawnLine::Response> res);
    bool RabbitConstSpeedService(const std::shared_ptr<paths::srv::SetConstSpeed::Request> req, std::shared_ptr<paths::srv::SetConstSpeed::Response> res);
    bool VehicleConstSpeedService(const std::shared_ptr<paths::srv::SetConstSpeed::Request> req, std::shared_ptr<paths::srv::SetConstSpeed::Response> res);

    /** 
     * @brief Auxiliar method to be called inside the callbacks
     */
    bool loadSectionIntoPath(PathSection * section);
    bool loadSpeedIntoPath(Speed * speed); 
};
