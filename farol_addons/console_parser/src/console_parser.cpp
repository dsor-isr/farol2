#include "console_parser.h"

/**
 * @brief  Console Path Parser node constructor.
 */
ConsoleParser::ConsoleParser() : Node("console_parser",
                                      rclcpp::NodeOptions()
                                        .allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true)) {
    loadParams();
    initializeSubscribers();
    initializePublishers();
    initializeServices();
    initializeTimer();

    depth_end = rclcpp::Time(0,1);
}

/**
 * @brief  Console Path Parser node destructor
 */
ConsoleParser::~ConsoleParser() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief  Method to initialize all the subscribers
 */
void ConsoleParser::initializeSubscribers() {
  missionstring_sub_ = create_subscription<std_msgs::msg::String>(
                        get_parameter("addons.console_parser.topics.subscribers.Mission_String").as_string(), 
                        1, std::bind(&ConsoleParser::missionStringCallback, this, std::placeholders::_1));
  
  state_sub_ = create_subscription<farol_msgs::msg::NavigationState>(
                get_parameter("addons.console_parser.topics.subscribers.state").as_string(), 
                1, std::bind(&ConsoleParser::stateCallback, this, std::placeholders::_1));

  mission_status_sub_ = create_subscription<std_msgs::msg::Int8>(
                          get_parameter("addons.console_parser.topics.subscribers.mission_status").as_string(), 
                          1, std::bind(&ConsoleParser::missionStatusCallback, this, std::placeholders::_1));
}


/**
 * @brief  Method to initialize all the publishers 
 */
void ConsoleParser::initializePublishers() {
  section_pub_ = create_publisher<farol_msgs::msg::Section>(
                  get_parameter("addons.console_parser.topics.publishers.Path_Section").as_string(), 1);

  formation_pub_ = create_publisher<farol_msgs::msg::Formation>(
                    get_parameter("addons.console_parser.topics.publishers.Formation").as_string(), 1);

  biased_formation_pub_ = create_publisher<farol_msgs::msg::Formation>(
                            get_parameter("addons.console_parser.topics.publishers.biased_formation").as_string(), 1);

  wpref_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
                get_parameter("addons.console_parser.topics.publishers.WPRef").as_string(), 1);

  altitude_pub_ = create_publisher<std_msgs::msg::Float64>(
                    get_parameter("addons.console_parser.topics.publishers.DepthRef").as_string(), 1);

  depth_pub_ = create_publisher<std_msgs::msg::Float64>(
                get_parameter("addons.console_parser.topics.publishers.AltRef").as_string(), 1);

  fullpath_pub_ = create_publisher<farol_msgs::msg::MultiSection>(
                    get_parameter("addons.console_parser.topics.publishers.FullMission").as_string(), 1);
}

/**
 * @brief  Method to initialized all the services
 */
void ConsoleParser::initializeServices() {
  /* Service clients */
  reset_path_client_ = create_client<paths::srv::ResetPath>(
                        get_parameter("addons.console_parser.topics.services.reset_path").as_string());

  spawn_arc_client_ = create_client<paths::srv::SpawnArc2D>(
                        get_parameter("addons.console_parser.topics.services.arc2d_path").as_string());

  spawn_line_client_ = create_client<paths::srv::SpawnLine>(
                        get_parameter("addons.console_parser.topics.services.line_path").as_string());

  set_path_speed_client_ = create_client<paths::srv::SetConstSpeed>(
                            get_parameter("addons.console_parser.topics.services.pf_start").as_string());

  // start_pf_client_ = create_client<path_following::srv::StartPF>(
  //                     get_parameter("addons.console_parser.topics.services.pf_stop").as_string());

  // stop_pf_client_ = create_client<path_following::srv::StopPF>(
  //                     get_parameter("addons.console_parser.topics.services.set_speed").as_string());
}

/**
 * @brief  Method to load all the parameters 
 */
void ConsoleParser::loadParams() {
  path_folder = get_parameter("addons.console_parser.path_folder").as_string();
  // TODO: probably not being used, legacy
  own_id = get_parameter("addons.console_parser.vehicle_id").as_int();
  p_console_new_ = get_parameter("addons.console_parser.console_new").as_bool();
}

/**
 * @brief  Method to initialize the timer
 */
void ConsoleParser::initializeTimer() {
  /* Get node frequency from parameters */
  int freq = get_parameter("addons.console_parser.node_frequency").as_int();

  /* Create timer */
  timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq*1000)), std::bind(&ConsoleParser::depthCallback, this));
}

/* 
 * @brief  Mission String Callback. Accepts string and tries to parse mission from it
 *
 * @param msg  A reference to a string message that contains a mission
 */
void ConsoleParser::missionStringCallback(const std_msgs::msg::String &msg) {
  RCLCPP_INFO(get_logger(), "New Mission String");
  std::istringstream is(msg.data);
  parseMission(is);

  /* Call the method to request the service to generate the path */
  requestPath();

  /* get pretty name for the mission */
  time_t rawtime;
  struct tm *timeinfo;
  char buffer[BUF_SIZE_TIME];
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, BUF_SIZE_TIME, "/%Y-%m-%d_%H-%M-%S.txt", timeinfo);
  RCLCPP_INFO_STREAM(get_logger(), "PATH: " << path_folder);
  std::string str(path_folder + buffer);
  RCLCPP_INFO(get_logger(), "Saving mission in: [%s]", str.c_str());
  std::ofstream out(str.c_str());
  out << msg.data;
  out.close();
}


/**
 * @brief  Method used to request the path stored in the mission to the path service
 */
void ConsoleParser::requestPath() {

  std::list<Section>::iterator it;
  
  /* Auxiliary variable to check wether there was at least 1 valid section to follow */
  int run = 0;

  /* Call the service to reset the path */
  std::shared_ptr<paths::srv::ResetPath::Request> req = std::make_shared<paths::srv::ResetPath::Request>();
  req->reset_path = true;
  reset_path_client_->async_send_request(req);

  if (p_console_new_){
    xrefpoint = 0;
    yrefpoint = 0;
  }

  /* Iterate over the entire mission that was requested */
  for(it = mission.begin(); it != mission.end(); ++it) {
    /* Check which type of mission we have: 2=Line, 3=Arc */
    if(it->type == 2) {

      /* Make sure the line received is valid */
      if(!(it->xi == it->xe && it->yi == it->ye)) {
        /* Call the service to spawn a line in the path */
        std::shared_ptr<paths::srv::SpawnLine::Request> req = std::make_shared<paths::srv::SpawnLine::Request>();
        req->start_point[1] = xrefpoint + it->xi;
        req->start_point[0] = yrefpoint + it->yi;
        req->start_point[2] = 0.0;

        req->end_point[1] = xrefpoint + it->xe;
        req->end_point[0] = yrefpoint + it->ye;
        req->end_point[2] = 0.0;
      
        req->ref_point = {0.0, 0.0, 0.0}; 
        spawn_line_client_->async_send_request(req);

        /* Call the service to specify the section desired speed for this section */
        std::shared_ptr<paths::srv::SetConstSpeed::Request> speed_req = std::make_shared<paths::srv::SetConstSpeed::Request>();
        speed_req->speed = it->velocity;
        speed_req->default_speed = it->velocity;
        set_path_speed_client_->async_send_request(speed_req);
      
        run++;  /* Increment the number of valid sent sections */
      }
    
    } else if(it->type == 3) {

      /* Make sure the arc received is valid */
      if(!((it->xi == it->xe && it->yi == it->ye) || 
           (it->xi == it->xc && it->yi == it->yc) ||
           (it->xc == it->xe && it->yc == it->ye))) {

        /* Call the service to spawn an arc in the path */
        std::shared_ptr<paths::srv::SpawnArc2D::Request> req = std::make_shared<paths::srv::SpawnArc2D::Request>();
        req->start_point[1] = xrefpoint + it->xi;
        req->start_point[0] = yrefpoint + it->yi;
      
        req->end_point[1] = xrefpoint + it->xe;
        req->end_point[0] = yrefpoint + it->ye;

        req->center_point[1] = xrefpoint + it->xc;
        req->center_point[0] = yrefpoint + it->yc;

        req->direction = it->adirection;
        req->z = 0.0;
        spawn_arc_client_->async_send_request(req);
      
        /* Call the service to specify the section desired speed for this section */
        std::shared_ptr<paths::srv::SetConstSpeed::Request> speed_req = std::make_shared<paths::srv::SetConstSpeed::Request>();
        speed_req->speed = it->velocity;
        speed_req->default_speed = it->velocity;
        set_path_speed_client_->async_send_request(speed_req);

        /* Increment the number of valid sent sections */
        run++;
      }
    }
  }

  /* If there was at least one valid section to follow, invoke the path following algorithm */
  if(run >= 1) {
      /* Call the path following service */
      // path_following::srv::StartPF::Request req;
      // start_pf_client_->async_send_request(req);
  }
}

/*
 * @brief  Function used in MissionString_Callback. The goal is to parse the 
 * string mission to the vehicle format of sections
 *
 * @param is  A reference to a stringstream
 */
void ConsoleParser::parseMission(std::istream &is) {
  int version = 0;
  int lines = 0;
  double Gamma0 = 0;
  // +.+. Which vehicles will cooperate -> LEGACY probably
  std::list<int> IDs_present; 

  // +.+ Initialize formation mode to false
  formation_mode = false; 
  biased_formation_mode = false;
  std::string line;

  farol_msgs::msg::MultiSection FullSection;

  // +.+ Delete the previous mission
  mission.clear();   
  
  // +.+ Delete the previous formation
  formation.clear(); 

  // +.+ Read all the lines from the string
  while (getline(is, line)) {
    RCLCPP_INFO(get_logger(), "parseMission new line: [%s]", line.c_str());
    Section newSection;
    int res = 0;
    if (line[0] == '#') // Discard comments
      continue;
    lines++;          // Number of useful lines
    if (lines == 1) { // Version
      sscanf(line.c_str(), "%d", &version);
      if (version != 3) {
        RCLCPP_ERROR(get_logger(), "Mission version [%d] not supported. Supported versions: 3",
            version);
        return;
      }
      continue;
    }

    // +.+ Reference Points
    if (lines == 2) { 
      sscanf(line.c_str(), "%lf %lf %*s", &xrefpoint, &yrefpoint);
      if (p_console_new_){
        FullSection.ref_point.x = 0;
        FullSection.ref_point.y = 0;
      }
      FullSection.ref_point.x = xrefpoint;
      FullSection.ref_point.y = yrefpoint;
    }

    // +.+ Formation info
    if (line.compare(0, 9, "FORMATION") == 0) {
      std::vector<std::string> formation_str;
      boost::split(formation_str, line, boost::is_any_of("\t "));

      std::vector<std::string>::iterator fstrs;
      Formation aux;
      for (fstrs = formation_str.begin() + 1; fstrs != formation_str.end();
           fstrs = fstrs + 3) {
        if (strlen((*fstrs).c_str()) == 0)
          break;
        // +.+ Build the object
        aux.id = atoi((*fstrs).c_str());
        aux.x = atof((*(fstrs + 1)).c_str());
        aux.y = atof((*(fstrs + 2)).c_str());
        formation.push_back(aux); // add a new vehicle

        // +.+ For changing the path and see if this vehicle will act in this mission
        if (atoi((*fstrs).c_str()) == own_id) {
          formation_mode = true;
          x_forma = atof((*(fstrs + 1)).c_str());
          y_forma = atof((*(fstrs + 2)).c_str());
          RCLCPP_INFO(get_logger(), "FORMATION x=%lf y=%lf", x_forma, y_forma);
        }
      }

      if (!formation_mode) { 
        RCLCPP_WARN(get_logger(), "This mission is not for this vehicle, no id=%d in the "
                 "FORMATION line",
                 own_id);
        return;
      }
      // +.+ Don't add anything to the mission
      continue;
    } else if (line.compare(0, 4, "BIAS") == 0) {
      std::vector<std::string> formation_str;
      boost::split(formation_str, line, boost::is_any_of("\t "));

      std::vector<std::string>::iterator fstrs;
      Formation aux;
      for (fstrs = formation_str.begin() + 1; fstrs != formation_str.end();
           fstrs = fstrs + 3) {
        if (strlen((*fstrs).c_str()) == 0)
          break;
        // +.+ Build the object
        aux.id = atoi((*fstrs).c_str());
        aux.x = atof((*(fstrs + 1)).c_str());
        aux.y = atof((*(fstrs + 2)).c_str());
        formation.push_back(aux); // add a new vehicle

        // +.+ For changing the path and see if this vehicle will act in this mission
        if (atoi((*fstrs).c_str()) == own_id) {
          biased_formation_mode = true;
          x_forma = atof((*(fstrs + 1)).c_str());
          y_forma = atof((*(fstrs + 2)).c_str());
          RCLCPP_INFO(get_logger(), "BIASED FORMATION x=%lf y=%lf", x_forma, y_forma);
        }
      }

      if (!biased_formation_mode) { 
        RCLCPP_WARN(get_logger(), "This mission is not for this vehicle, no id=%d in the "
                 "FORMATION line",
                 own_id);
        return;
      }
      // Don't add anything to the mission
      continue; 
    } 
    // +.+ Point
    else if (line.compare(0, 5, "POINT") == 0) {
      RCLCPP_INFO(get_logger(), "POINT %s", line.c_str());
      newSection.type = 1;
      if ((res = sscanf(line.c_str(), "POINT %lf %lf %f %f %f %f %d",
              &newSection.xe, &newSection.ye, &newSection.radius,
              &newSection.velocity, &newSection.heading,
              &newSection.time, &newSection.nVehicle)) ==
          6) { // Optional value
        newSection.nVehicle = -1;
        Gamma0 = 0;
        newSection.gamma_s = 0;
        newSection.gamma_e = 0;
      } else if (res >= 7) {
        Gamma0 = 0;
        newSection.gamma_s = 0;
        newSection.gamma_e = 0;
      } else {
        continue;
      }
    } 
    // +.+ Line
    else if (line.compare(0, 4, "LINE") == 0) {
      newSection.type = 2;
      if ((res = sscanf(line.c_str(), "LINE %lf %lf %lf %lf %f %d %lf",
                        &newSection.xi, &newSection.yi, &newSection.xe,
                        &newSection.ye, &newSection.velocity,
                        &newSection.nVehicle, &newSection.gamma_e)) == 5) { 
        newSection.nVehicle = -1;
      }
      if (res >= 5 && res <= 6) {
        Gamma0 = (*--mission.end()).gamma_e;
        newSection.gamma_s = Gamma0;
        newSection.gamma_e = Gamma0 + sqrt(pow(newSection.xe - newSection.xi, 2) + pow(newSection.ye - newSection.yi, 2));
      } else {
        continue;
      }
    } 
    // +.+ Arc
    else if (line.compare(0, 3, "ARC") == 0) {
      newSection.type = 3;
      newSection.gamma_e = -1;
      
      if ((res = sscanf(
               line.c_str(), "ARC %lf %lf %lf %lf %lf %lf %f %d %f %d %lf",
               &newSection.xi, &newSection.yi, &newSection.xc, &newSection.yc,
               &newSection.xe, &newSection.ye, &newSection.velocity,
               &newSection.adirection, &newSection.radius, &newSection.nVehicle,
               &newSection.gamma_e)) == 9) { // Optional value
        newSection.nVehicle = -1;
      }

      if (res >= 9 && res <= 10) {
        Gamma0 = (*--mission.end()).gamma_e;
        double psis = atan2(newSection.yi - newSection.yc, newSection.xi - newSection.xc);
        double psie = atan2(newSection.ye - newSection.yc, newSection.xe - newSection.xc);
        newSection.gamma_s = Gamma0;
        
        // +.+  Check direction
        // +.+ Clockwise
        if (newSection.adirection == -1)
          newSection.gamma_e = Gamma0 + FarolUtils::wrapTo2Pi(psis - psie) * newSection.radius;
        // +.+ Counter clockwise
        else
          newSection.gamma_e = Gamma0 + (2 * M_PI - FarolUtils::wrapTo2Pi(psis - psie)) * newSection.radius;
      } else {
        continue;
      }
    } 
    // +.+ Depth
    else if (line.compare(0, 5, "DEPTH") == 0) {
      if ((res = sscanf(line.c_str(), "DEPTH %f %f %d", &newSection.depth, &newSection.time, &newSection.nVehicle)) == 3) {
        newSection.type = 4;
      } else
        RCLCPP_ERROR(get_logger(), "Erroneous construction of DEPTH Primitive.");
    } 
    // +.+ Alt
    else if (line.compare(0, 3, "ALT") == 0) {
      if ((res = sscanf(line.c_str(), "ALT %f %f %d", &newSection.depth, &newSection.time, &newSection.nVehicle)) == 3) {
        newSection.type = 5;
      } else
        RCLCPP_ERROR(get_logger(), "Erroneous construction of Altitude Primitive.");
    } 
    // +.+ Unknown
    else
      continue;

    // +.+ Create variables for RViz Visualization -> LEGACY
    geometry_msgs::msg::Point start_point;
    geometry_msgs::msg::Point center_point;
    geometry_msgs::msg::Point end_point;
    start_point.x = newSection.xi;
    start_point.y = newSection.yi;
    start_point.z = 0.0;
    center_point.x = newSection.xc;
    center_point.y = newSection.yc;
    center_point.z = 0.0;
    end_point.x = newSection.xe;
    end_point.y = newSection.ye;
    end_point.z = 0.0;

    FullSection.type.push_back(newSection.type);
    FullSection.adirection.push_back(newSection.adirection);
    FullSection.start_point.push_back(start_point);
    FullSection.center_point.push_back(center_point);
    FullSection.end_point.push_back(end_point);
    FullSection.gamma_s.push_back(newSection.gamma_s);
    FullSection.gamma_e.push_back(newSection.gamma_e);

    mission.push_back(newSection);
  }

  FullSection.header.stamp = clock_.now();
  fullpath_pub_->publish(FullSection);

  farol_msgs::msg::Formation Form_Topic; 
  
  // +.+ Send the formation to the Cooperative_PF

  // +.+ If we are in the presence of a Formation definition
  if (formation_mode || biased_formation_mode) {
    Form_Topic.id.resize(formation.size());
    Form_Topic.x.resize(formation.size());
    Form_Topic.y.resize(formation.size());
    std::list<Formation>::iterator j;
    int i = 0;
    for (j = formation.begin(); j != formation.end(); ++j, ++i) {
      Form_Topic.id[i] = (*j).id;
      Form_Topic.x[i] = (*j).x;
      Form_Topic.y[i] = (*j).y;
    }
    if (formation_mode)
      // +.+ Change the mission accordingly
      missionFormation();            
  } 
  // +.+ If in the fille there is only gammas and specific missions for each vehicle - LEGACY
  else if (IDs_present.size() > 0) { 
    Form_Topic.id.resize(IDs_present.size());
    Form_Topic.x.resize(IDs_present.size());
    Form_Topic.y.resize(IDs_present.size());
    std::list<int>::iterator j;
    int i = 0;
    for (j = IDs_present.begin(); j != IDs_present.end(); ++j, ++i) {
      Form_Topic.id[i] = (*j);
      Form_Topic.x[i] = 0;
      Form_Topic.y[i] = 0;
    }
  } 
  // +.+ Normal Path-Following without Cooperation (constant speed)
  else {
    Form_Topic.id.resize(1);
    Form_Topic.x.resize(1);
    Form_Topic.y.resize(1);
    Form_Topic.id[0] = own_id;
    Form_Topic.x[0] = 0;
    Form_Topic.y[0] = 0;
  }

  // +.+ Print ref point 
  printf("---------------------------------------------------------\n");
  printf("Mission: xrefpoint=%lf yrefpoint=%lf\n\n", xrefpoint, yrefpoint);
  
  std::list<Section>::iterator i;
  
  for (i = mission.begin(); i != mission.end(); ++i) {
    // +.+ Gammas and Section Length for all the sections of this vehicle
    if (((*i).type == 2 || (*i).type == 3) && ((*i).nVehicle == -1 || (*i).nVehicle == own_id)) {
      
      Form_Topic.gamma_e.push_back((*i).gamma_e);

      // +.+ Start point, end point, center point
      Eigen::VectorXf ps(2), pe(2), pc(2); 
      ps[0] = (*i).xi;
      ps[1] = (*i).yi;
      pe[0] = (*i).xe;
      pe[1] = (*i).ye;
      pc[0] = (*i).xc;
      pc[1] = (*i).yc;

      // +.+ Lines
      if ((*i).type == 2) { // Lines
        Form_Topic.length.push_back((pe - ps).norm());
      } 
      // +.+ Arcs
      else if ((*i).type == 3) {
        // +.+ Initial angle                         
        double phi0 = atan2(ps(1) - pc(1), ps(0) - pc(0)); 
        
        // +.+ Final angle
        double phie = atan2(pe(1) - pc(1), pe(0) - pc(0)); 
        
        // +.+ Arc radius
        double R = (pc - ps).norm();                      
        double arc_len = 0;
        
        // +.+. Check direction
        // +.+ Clockwise
        if ((*i).adirection == -1) {
          arc_len = FarolUtils::wrapTo2Pi(phi0 - phie) * R;
        }
        // +.+ Counter-clockwise
        else{
          arc_len = (2 * M_PI - FarolUtils::wrapTo2Pi(phi0 - phie)) * R;
        }
        Form_Topic.length.push_back(arc_len);
      }
    }

    printf("SECTION type=%d Xi=%lf Yi=%lf Xc=%lf Yc=%lf Xe=%lf Ye=%lf Vl=%f "
           "Dir=%d R0=%lf NV=%d Gs=%lf Ge=%lf\n\n",
           (*i).type, (*i).xi, (*i).yi, (*i).xc, (*i).yc, (*i).xe, (*i).ye,
           (*i).velocity, (*i).adirection, (*i).radius, (*i).nVehicle,
           (*i).gamma_s, (*i).gamma_e);
  }
  printf("---------------------------------------------------------\n\n");

  // TODO: Legacy here, need to review
  // +.+ Send formation to the Cooperative_PF but it is only activated by the section
  if (biased_formation_mode)
    biased_formation_pub_->publish(Form_Topic);
  else
    formation_pub_->publish(Form_Topic);
  
  // +.+ Detected Mission to do
  if (mission.size() > 0) {
    // +.+ Initialize variables
    act_section = mission.begin();
    gamma_s = 0;
    gamma_e = 0;
    ENABLE = true;

    // +.+ Start the Mission
    startNewSection();
  }
}

// @.@ Function used in MissionString_Callback -> for cooperative pf
void ConsoleParser::missionFormation() {
  std::list<Section>::iterator i;
  for (i = mission.begin(); i != mission.end(); ++i) {
    // +.+ Can only change the mission if it doesn't have any vehicle associatead
    if ((*i).nVehicle != -1) 
      continue;

    // +.+ Waypoint
    if ((*i).type == 1) { // WayPoint
      continue;
    // +.+ Straight Line
    } else if ((*i).type == 2) { // Straight Line
      double psi = atan2((*i).ye - (*i).yi, (*i).xe - (*i).xi);
      (*i).xi = (*i).xi + x_forma * cos(psi) - y_forma * sin(psi);
      (*i).yi = (*i).yi + x_forma * sin(psi) + y_forma * cos(psi);
      (*i).xe = (*i).xe + x_forma * cos(psi) - y_forma * sin(psi);
      (*i).ye = (*i).ye + x_forma * sin(psi) + y_forma * cos(psi);
    // +.+ Arc
    } else if ((*i).type == 3) { 
      double psis = atan2((*i).yi - (*i).yc, (*i).xi - (*i).xc) + (*i).adirection * M_PI / 2;
      double psie = atan2((*i).ye - (*i).yc, (*i).xe - (*i).xc) + (*i).adirection * M_PI / 2;
      (*i).xi = (*i).xi + x_forma * cos(psis) - y_forma * sin(psis);
      (*i).yi = (*i).yi + x_forma * sin(psis) + y_forma * cos(psis);
      (*i).xe = (*i).xe + x_forma * cos(psie) - y_forma * sin(psie);
      (*i).ye = (*i).ye + x_forma * sin(psie) + y_forma * cos(psie);
    }
  }
}

// @.@ Function to manage switch between sections of the path from console, and what to do in the end
void ConsoleParser::startNewSection() {
  // +.+ Verifify if the section is for this vehicle
  while (act_section != mission.end() &&
         !((*act_section).nVehicle == -1 || (*act_section).nVehicle == own_id)) {
    RCLCPP_WARN(get_logger(), "This section is not for this vehicle [ID=%d]",
             (*act_section).nVehicle);
    RCLCPP_INFO(get_logger(), "SECTION T=%d Xi=%lf Yi=%lf Xc=%lf Yc=%lf Xe=%lf Ye=%lf Vl=%f "
             "Dir=%d R0=%lf NV=%d Gs=%lf Ge=%lf",
             (*act_section).type, (*act_section).xi, (*act_section).yi,
             (*act_section).xc, (*act_section).yc, (*act_section).xe,
             (*act_section).ye, (*act_section).velocity, (*act_section).adirection,
             (*act_section).radius, (*act_section).nVehicle,
             (*act_section).gamma_s, (*act_section).gamma_e);
    ++act_section;
  }

  // +.+ Mission End
  if (act_section == mission.end()) {
    RCLCPP_INFO(get_logger(), "Mission Finished");

    geometry_msgs::msg::PointStamped aux;
    aux.point.x = -3;
    aux.point.y = -3;
    gamma_s = 0;
    gamma_e = 0;
    // +.+  Hold Position in the same point
    wpref_pub_->publish(aux);

    // +.+ Publish, during 2 sec, 0 depth and kill timer
    DesiredDepth = 0.0;
    timer_.reset();

    ENABLE = false;
    return;
  }

  RCLCPP_INFO(get_logger(), "SECTION T=%d Xi=%lf Yi=%lf Xc=%lf Yc=%lf Xe=%lf Ye=%lf Vl=%f "
           "Dir=%d R0=%lf NV=%d Gs=%lf Ge=%lf",
           (*act_section).type, (*act_section).xi, (*act_section).yi,
           (*act_section).xc, (*act_section).yc, (*act_section).xe,
           (*act_section).ye, (*act_section).velocity, (*act_section).adirection,
           (*act_section).radius, (*act_section).nVehicle, (*act_section).gamma_s,
           (*act_section).gamma_e);
  
  if (p_console_new_){
      xrefpoint = 0;
      yrefpoint = 0;
    }
  // +.+ Waypoint
  if ((*act_section).type == 1) {

    RCLCPP_INFO(get_logger(), "Starting a Waypoint");
    geometry_msgs::msg::PointStamped aux;

    // +.+ Controller needs the Global position
    aux.point.x = (*act_section).xe + xrefpoint;
    aux.point.y = (*act_section).ye + yrefpoint;
    gamma_s = 0;
    // +.+ As long as a waypoint is required even in the middle of the mission gamma starts counting from 0
    gamma_e = 0; 

    wpref_pub_->publish(aux);
  // +.+ Line or Arc  
  } else if ((*act_section).type == 2 || (*act_section).type == 3) { 

    RCLCPP_INFO(get_logger(), "Starting a Path Following Mission");
    farol_msgs::msg::Section aux;
    aux.xrefpoint = xrefpoint;
    aux.yrefpoint = yrefpoint;
    aux.direction = (*act_section).adirection;
    aux.xs = (*act_section).xi;
    aux.ys = (*act_section).yi;
    aux.xc = (*act_section).xc;
    aux.yc = (*act_section).yc;
    aux.xe = (*act_section).xe;
    aux.ye = (*act_section).ye;
    aux.vl = (*act_section).velocity;
    aux.direction = (*act_section).adirection;
    aux.r0 = (*act_section).radius;
    aux.gamma_s = (*act_section).gamma_s;
    aux.gamma_e = (*act_section).gamma_e; 
    section_pub_->publish(aux);

    // +.+ New gamma is the previous one
    gamma_s = gamma_e;               
    // +.+ Updated gamma end
    gamma_e = (*act_section).gamma_e; 
    RCLCPP_WARN(get_logger(), "Gamma: %f", gamma_e);
  // +.+ Depth and Altitude References
  } else if ((*act_section).type == 4 || (*act_section).type == 5) {
    RCLCPP_INFO(get_logger(), "STARTING DEPTH PUBLISHING");
    if ((*act_section).type == 5)
      DesiredDepth = -(*act_section).depth;
    else
      DesiredDepth = (*act_section).depth;

    // +.+ Activate the end of references for depth
    if ((*act_section).time > 0) {
      depth_end = clock_.now() + rclcpp::Duration((*act_section).time, 0);
    } else {
      gamma_s = 0;
      gamma_e = 0;
      ++act_section;
      startNewSection();
      depth_end = rclcpp::Time(0,1);
    }
    /* Start the timer again */
    initializeTimer();
  }
}


/*
#######################################################################################################################
 @.@ Callbacks Section / Methods
#######################################################################################################################
*/

 // @.@ Depth Callback -> Iteration via timer callback
void ConsoleParser::depthCallback() {
  // +.+ Depth with schedule end
  if (depth_end != rclcpp::Time(0,1)) {
    RCLCPP_WARN(get_logger(), "bruh");
    if ((depth_end - clock_.now()).seconds() <= 0) {
      DesiredDepth = 0.0;
      timer_.reset();
      gamma_s = 0;
      gamma_e = 0;
      ++act_section;
      startNewSection();
      return;
    }
  }

  std_msgs::msg::Float64 depth;
  // +.+ Depth Reference
  if (DesiredDepth >= 0) {
    depth.data = DesiredDepth;
    depth_pub_->publish(depth);
    // RCLCPP_INFO_THROTTLE(get_logger(), clock_, 10000, "Desired Depth: %f", depth.data);
  } else {
    depth.data = -DesiredDepth;
    altitude_pub_->publish(depth);
    // RCLCPP_INFO_THROTTLE(get_logger(), clock_, 10000, "Desired Altitude: %f", depth.data);
  }
}

// @.@ Mission Status Callback 
void ConsoleParser::missionStatusCallback(const std_msgs::msg::Int8 &msg) {
  // +.+ Check the Mission Status if there is a mission running
  if (!ENABLE)
    return;

  // +.+ Abort everything if received Mission Status == 0
  if (msg.data == 0) { // IDLE
    ENABLE = false;
    timer_.reset();
  }
  // +.+ Path-Following Mission - Mission Status should be 6
  if ((*act_section).type == 2 || (*act_section).type == 3) {
    if (msg.data != 6) {
      ENABLE = false;
      timer_.reset();
      RCLCPP_ERROR(get_logger(), "Mission Ended due to changes in the Mission Status. Got %d instead of 6",
                msg.data);
    }
  }
}

// @.@ Callback for updating 2D position (X,Y) values in the State topic
void ConsoleParser::stateCallback(const farol_msgs::msg::NavigationState &msg) {

	// waypoint works in ENU but state from filter is in NED
  x_act = msg.utm_position.easting;
  y_act = msg.utm_position.northing;
  
  if (p_console_new_){
    xrefpoint = 0;
    yrefpoint = 0;
  }
  if (ENABLE && (*act_section).type == 1) { 
    // +.+ Position and end point
    Eigen::VectorXf Pos(2), pe(2);
    pe[0] = (*act_section).xe;
    pe[1] = (*act_section).ye;
    Pos[0] = x_act - xrefpoint;
    Pos[1] = y_act - yrefpoint;
    if ((Pos - pe).norm() < (*act_section).radius * 1.2) { // On the Point
      if ((*act_section).time != -1) {
        rclcpp::sleep_for(std::chrono::seconds(static_cast<int64_t>(rclcpp::Duration((*act_section).time, 0).seconds())));
        ++act_section;
        // +.+ Start Next Section
        startNewSection();
      }
    }
  }
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConsoleParser>());
  rclcpp::shutdown();
  return 0;
}
