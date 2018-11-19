#include<ros/ros.h> 
#include<std_msgs/Float64.h> 
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped block_coordinates;
bool flag;
bool collected;

void myCallback(const geometry_msgs::PoseStamped& message_holder) 
{ 
  if (flag) {
 	ROS_INFO("In callback function");
  	block_coordinates = message_holder;
	flag = false;
	collected = true;
  }
} 

int main(int argc, char **argv) 
{ 
  ros::init(argc,argv,"minimal_subscriber"); //name this node 
  // when this compiled code is run, ROS will recognize it as a node called "minimal_subscriber" 
  ros::NodeHandle n; // need this to establish communications with our new node 
  ros::Subscriber my_subscriber_object= n.subscribe("block_pose",1,myCallback); 
  //create a Subscriber object and have it subscribe to the topic "topic1" 
  // the function "myCallback" will wake up whenever a new message is published to topic1 
  // the real work is done inside the callback function 
  ROS_INFO("About to go into callback function");

  flag = true;
  collected = false;
  while (!collected) {
	  ROS_INFO("callback hasn't been called yet");
	  ros::spinOnce();
	  ros::Duration(0.5).sleep();
	}
  ROS_INFO("received x value is: %f", block_coordinates.pose.position.x); 
  ROS_INFO("received y value is: %f", block_coordinates.pose.position.y); 
  ROS_INFO("received z value is: %f", block_coordinates.pose.position.z); 
  // forces refreshing wakeups upon new data arrival 
  // main program essentially hangs here, but it must stay alive to keep the callback function alive 
  return 0; // should never get here, unless roscore dies 
} 
