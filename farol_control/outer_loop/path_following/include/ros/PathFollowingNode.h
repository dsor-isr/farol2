#pragma once

#include <math.h>
#include <stdlib.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

/* Include the messages used by publishers */
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"

/* Messages used to receive data from the vehicle and from the path */
#include "farol_msgs/msg/navigation_state.hpp"
#include "paths/msg/path_data.hpp"
#include "farol_msgs/msg/pf_debug.hpp"

/* Include the Control libraries */
#include "RelativeHeading.h"
#include "Marcelo.h"
#include "Aguiar.h"
#include "Breivik.h"
#include "Fossen.h"
#include "Romulo.h"
#include "Lapierre.h"
#include "Pramod.h"
#include "Samson.h"
#include "PathFollowing.h"
#include "States.h"

/* Include the generated services for the path following */
#include "path_following/srv/reset_vt.hpp"
#include "path_following/srv/set_pf.hpp"
#include "path_following/srv/start_pf.hpp"
#include "path_following/srv/stop_pf.hpp"
#include "path_following/srv/update_gains_pf.hpp"

/* Include the waypoint service to use when a mission finishes */
#include "waypoint/srv/send_wp_type1.hpp"
#include "std_srvs/srv/trigger.hpp"

/** 
 * Include the service to set the path mode to closest point (needed for
 * some algorithms)
 */
#include "paths/srv/reset_path.hpp"
#include "paths/srv/set_mode.hpp"

/* Define Constants used when starting and stoping the path following */
#define WP_FINISH -3
#define FLAG_IDLE 0
#define FLAG_PF 6

/**
 * @brief     Path Following Node, where the magic happens 
 * @author    Marcelo Jacinto
 * @author    Joao Quintas
 * @author    Joao Cruz
 * @author    Hung Tuan
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
class PathFollowingNode : public rclcpp::Node {

  public:

    /**
     * @brief  The constructor of the path following node
     */
    PathFollowingNode();

    /**
     * @brief The destructor of the path following node
     */
    ~PathFollowingNode();

  private:

    /**
     * @brief Pointer to the Path Following algorithm to use 
     */
    PathFollowing *pf_algorithm_{nullptr};

    /**
     * @brief Auxiliar variables to store the current vehicle state and path values 
     */
    VehicleState vehicle_state_;
    PathState path_state_;

    /**
     * @brief Auxiliary variables to check whether we have received the first information
     * for intial path position and vehicle state
     */
    bool has_received_vehicle_state{false};
    bool has_received_path_state{false};

    /**
     * @brief Use to store the time-elapsed to be used by the controller
     */
    rclcpp::Time prev_time_;

    /**
     * @brief ROS Services for path following 
     */
    rclcpp::Service<path_following::srv::StartPF>::SharedPtr pf_start_srv_;
    rclcpp::Service<path_following::srv::StopPF>::SharedPtr pf_stop_srv_;
    rclcpp::Service<path_following::srv::UpdateGainsPF>::SharedPtr pf_update_gains_srv_;

    rclcpp::Service<path_following::srv::SetPF>::SharedPtr pf_aguiar_srv_;
    rclcpp::Service<path_following::srv::SetPF>::SharedPtr pf_breivik_srv_;
    rclcpp::Service<path_following::srv::SetPF>::SharedPtr pf_fossen_srv_;
    rclcpp::Service<path_following::srv::SetPF>::SharedPtr pf_romulo_srv_;
    rclcpp::Service<path_following::srv::SetPF>::SharedPtr pf_lapierre_srv_;
    rclcpp::Service<path_following::srv::SetPF>::SharedPtr pf_pramod_srv_;
    rclcpp::Service<path_following::srv::SetPF>::SharedPtr pf_samson_srv_;
    rclcpp::Service<path_following::srv::SetPF>::SharedPtr pf_marcelo_srv_;
    rclcpp::Service<path_following::srv::SetPF>::SharedPtr pf_relative_heading_srv_;

    /* Service to reset the virtual target position */
    rclcpp::Service<path_following::srv::ResetVT>::SharedPtr pf_reset_vt_srv_;

    /**
     * @brief ROS Services auxiliary to the path following 
     */
    rclcpp::Client<paths::srv::SetMode>::SharedPtr set_path_mode_client_;
    rclcpp::Client<waypoint::srv::SendWpType1>::SharedPtr wp_standard_client_;
    rclcpp::Client<paths::srv::ResetPath>::SharedPtr reset_path_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr dr_reset_client_;         

    /**
     * @brief ROS publishers attributes
     */
    std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_status_pub_;

    /**
     * @brief ROS subscribers attributes
     */
    rclcpp::Subscription<farol_msgs::msg::NavigationState>::SharedPtr state_sub_;
    rclcpp::Subscription<paths::msg::PathData>::SharedPtr path_data_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vc_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_status_sub_;

    /* Timer for node's callbacks */
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Clock clock_;

    /**
     * @brief Method to allocate memory for a default PF controller class 
     */
    PathFollowing *getDefaultControllerLapierre();
    PathFollowing *getDefaultControllerBreivik();
    PathFollowing *getDefaultControllerAguiar();

    /**
     * @brief Method to delete the current controller being used
     */
    void deleteCurrentController();

    /**
     * @brief Method to stop the current path_following 
     */
    void stopAlgorithm();

    /**
     * @brief Method to initialise the ROS part
     */
    void initialiseSubscribers();
    void initialisePublishers();
    void initialiseServices();
    void initialiseTimer();

    /**
     * @brief Method where the logic is located in order to update the control law
     *
     * @param event  A TimerEvent from ros
     */
    void timerIterCallback();

    /**
     * @brief Callbacks
     */
    void vcCallback(const std_msgs::msg::Float32 &msg);
    void pathStateCallback(const paths::msg::PathData &msg);
    void vehicleStateCallback(const farol_msgs::msg::NavigationState &msg);
    void missionStatusCallback(const std_msgs::msg::Int8 &msg);

    /**
     * @brief Services callbacks 
     */

    void StartPFService(const std::shared_ptr<path_following::srv::StartPF::Request> req,
                        std::shared_ptr<path_following::srv::StartPF::Response> res);

    void StopPFService(const std::shared_ptr<path_following::srv::StopPF::Request> req,
                        std::shared_ptr<path_following::srv::StopPF::Response> res);

    void ResetVirtualTargetService(const std::shared_ptr<path_following::srv::ResetVT::Request> req,
                                   std::shared_ptr<path_following::srv::ResetVT::Response> res);
    
    void UpdateGainsPFService(const std::shared_ptr<path_following::srv::UpdateGainsPF::Request> req,
                              std::shared_ptr<path_following::srv::UpdateGainsPF::Response> res);

    
    void SetRelativeHeadingService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                                   std::shared_ptr<path_following::srv::SetPF::Response> res);

    void SetMarceloService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                           std::shared_ptr<path_following::srv::SetPF::Response> res);

    void SetAguiarService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                          std::shared_ptr<path_following::srv::SetPF::Response> res);

    void SetBreivikService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                          std::shared_ptr<path_following::srv::SetPF::Response> res);

    void SetFossenService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                          std::shared_ptr<path_following::srv::SetPF::Response> res);

    void SetRomuloService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                          std::shared_ptr<path_following::srv::SetPF::Response> res);

    void SetLapierreService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                            std::shared_ptr<path_following::srv::SetPF::Response> res);

    void SetPramodService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                          std::shared_ptr<path_following::srv::SetPF::Response> res);

    void SetSamsonService(const std::shared_ptr<path_following::srv::SetPF::Request> req,
                          std::shared_ptr<path_following::srv::SetPF::Response> res);

    /**
     * @brief Service to send a waypoint when the path following stops
     */
    void sendWaypoint(double value);
    
    /**
     * @brief Service to reset the DeadReckoning position when the path following stops
     */
    void sendResetDeadReckoning();
};
