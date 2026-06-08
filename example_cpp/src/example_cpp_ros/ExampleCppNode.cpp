/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include "ExampleCppNode.h"
#include "ExampleCppAlgorithm.h"

// @.@ Constructor
ExampleCppNode::ExampleCppNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  // initializeServices();
  initializeTimer();

}

// @.@ Destructor
ExampleCppNode::~ExampleCppNode() {

  // +.+ shutdown publishers


  // +.+ shutdown subscribers


  // +.+ stop timer
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

// @.@ Member helper to load parameters from parameter server
void ExampleCppNode::loadParams() {
  ROS_INFO("Load the ExampleCppNode parameters");

  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 5);
  count_ = FarolGimmicks::getParameters<int>(nh_private_, "start_num", 0);
  pause = false;

}


// @.@ Member helper function to set up subscribers
void ExampleCppNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for ExampleCppNode");
  pause_sub = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/pause"), 1, &ExampleCppNode::setPause, this);
}


// @.@ Member helper function to set up publishers
void ExampleCppNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for ExampleCppNode");
  count_pub = nh_private_.advertise<std_msgs::Int32>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/count"), 1);
}

void ExampleCppNode::setPause(const std_msgs::Bool msg){
  pause = msg.data;
}

// @.@ Member helper function to set up services
void ExampleCppNode::initializeServices() {
  ROS_INFO("Initializing Services for ExampleCppNode");

}


// @.@ Member helper function to set up the timer
void ExampleCppNode::initializeTimer() {
  timer_ =nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &ExampleCppNode::timerIterCallback, this);
}


// @.@ Where the magic should happen.
void ExampleCppNode::timerIterCallback(const ros::TimerEvent &event) {
  std_msgs::Int32 count_to_send;

  if(pause) count_to_send.data = count_;
  else count_to_send.data = counting(&count_);

  count_pub.publish(count_to_send);
}


/*
  @.@ Main
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "example_cpp_node"); //node name
  
  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type ExampleCppNode");

  // +.+ instantiate an ExampleCppNode class object and pass in pointers to nodehandle public and private for constructor to use
  ExampleCppNode example_cpp(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
