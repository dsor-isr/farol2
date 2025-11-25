#pragma once

// some generically useful stuff to include...
#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <list>

/* Farol Utils */
#include "utils.hpp"

// ROS Fundamentals
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

// Package Related
#include <section.h>
#include <formation.h>

/* Required to call the path services*/
#include "paths/srv/reset_path.hpp"
#include "paths/srv/spawn_arc2_d.hpp"
#include "paths/srv/spawn_line.hpp"
#include "paths/srv/set_const_speed.hpp"

#include "path_following/srv/start_pf.hpp"
#include "path_following/srv/stop_pf.hpp"

// ROS messages and stuff
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
// #include "geometry_msgs/msg/vector3_stamped.hpp"
#include "farol_msgs/msg/navigation_state.hpp"
#include "farol_msgs/msg/formation.hpp"
#include "farol_msgs/msg/multi_section.hpp"
#include "farol_msgs/msg/section.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

#define BUF_SIZE_TIME 80

/**
 * @brief Class Responsible for parsing a mission from the console Yebisu to farol_vx stack format
 * 
 */
class ConsoleParser : public rclcpp::Node {
  public:
    /**
     * @brief Construct a new Console Parser object
     */
    ConsoleParser();

    /**
     * @brief Destroy the Console Parser object
     * 
     */
    ~ConsoleParser();

    // +.+ Mission and Formation list
    std::list<Section> mission;
    std::list<Formation> formation;

    // +.+ Actual Section of the mission
    std::list<Section>::iterator act_section;

    // +.+ Without this variable gamma will be a huge value. Pfs will work, but not ideal
    farol_msgs::msg::Section section_copy; 	

    rclcpp::Time depth_end;

    // +.+ ID of the vehicle passed as parameter
    int own_id{0}; 

    // +.+ Section Variables
    double xrefpoint = 0;
    double yrefpoint = 0;
    double gamma_s = 0;
    double gamma_e = 0;
    double x_act = 0;
    double y_act = 0;
    double gamma = 0;
    double gamma_old = 0;
    double u_est = 0;

    // +.+ Formation Variables
    double x_forma = 0;
    double y_forma = 0;

    // +.+ Aux class variables
    float DesiredDepth = 0.0;
    bool wpOrient;
    bool ENABLE = false;
    bool formation_mode = false;
    bool biased_formation_mode = false;
    float node_frequency;
    std::string path_folder;

  private:
    /* Timer for node's callbacks */
    rclcpp::TimerBase::SharedPtr timer_;

    // +.+ Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr missionstring_sub_;
    rclcpp::Subscription<farol_msgs::msg::NavigationState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_status_sub_;

    // +.+ Publishers
    rclcpp::Publisher<farol_msgs::msg::Section>::SharedPtr section_pub_;
    rclcpp::Publisher<farol_msgs::msg::Formation>::SharedPtr formation_pub_;
    rclcpp::Publisher<farol_msgs::msg::Formation>::SharedPtr biased_formation_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr wpref_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;
    rclcpp::Publisher<farol_msgs::msg::MultiSection>::SharedPtr fullpath_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr flag_pub_;

    // +.+ Parameters
    bool p_console_new_{false};

    /* Path Service clients - used to construct the path to follow */
    rclcpp::Client<paths::srv::ResetPath>::SharedPtr reset_path_client_;
    rclcpp::Client<paths::srv::SpawnArc2D>::SharedPtr spawn_arc_client_;
    rclcpp::Client<paths::srv::SpawnLine>::SharedPtr spawn_line_client_;
    rclcpp::Client<paths::srv::SetConstSpeed>::SharedPtr set_path_speed_client_;

    /* Path Following clients - to start and stop the path following algorithm */
    rclcpp::Client<path_following::srv::StartPF>::SharedPtr start_pf_client_;
    rclcpp::Client<path_following::srv::StopPF>::SharedPtr stop_pf_client_;

    /* ROS Clock */
    rclcpp::Clock clock_;

    // +.+ Reference position
    std_msgs::msg::Float32 aux;
    geometry_msgs::msg::PointStamped Reference_;

    // #######################################################################################
    // @.@ Encapsulation the gory details of initializing subscribers, publishers and services
    // #######################################################################################

    /**
     * @brief Load parameters from ROS parameter server
     * 
     */
    void loadParams();

    /**
     * @brief Initialize all node subscribers
     * 
     */
    void initializeSubscribers();

    /**
     * @brief Initialize all node publishers
     * 
     */
    void initializePublishers();

    /**
     * @brief Initialize all service publishers
     * 
     */
    void initializeServices();

    /**
     * @brief Initialize node timer
     * 
     */
    void initializeTimer();

    // #######################################################################################
    // @.@ Member helper functions of the class
    // #######################################################################################s

    /**
     * @brief Parsing string mission to the vehicle format of sections, used in missionStringCallback 
     * 
     * @param is 
     */
    void parseMission(std::istream &is);

    /**
     * @brief For cooperative pf, used in missionStringCallback
     * 
     */
    void missionFormation();

    /**
     * @brief Manage switch between sections of the path from console, and what to do in the end
     * 
     */
    void startNewSection();

    /**
     * @brief Method to make calls to the path service
     * 
     */
    void requestPath();

    // #######################################################################################
    // @.@ Callbacks declaration
    // #######################################################################################

    /**
     * @brief 
     */
    void depthCallback();

    /**
     * @brief Accepts string from external source (Yebisu console) and tries to parse mission from it 
     * 
     * @param msg Mission String message 
     */
    void missionStringCallback(const std_msgs::msg::String &msg);

    /**
     * @brief Check if we are doing a mission flag = 6, otherwise cancel 
     * 
     * @param msg Flag message 
     */
    void missionStatusCallback(const std_msgs::msg::Int8 &msg);

    /**
     * @brief 
     * 
     * @param msg  
     */
    void stateCallback(const farol_msgs::msg::NavigationState &msg);
};

