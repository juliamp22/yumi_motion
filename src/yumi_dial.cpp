
/* With this node, you can get information from the dialbox so as to sending to yumi Simulated or Real robot. In the .cpp file we have the 
 subsriber topic dialboxTopic and the publisher topic /yumi/joint_trajectory_pos_controller/command */

#include <yumi_motion/yumi_dial.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <dialbox/dialbox.h>
#include "dialbox/dialboxState.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener_to_dialbox");
  
  ros::NodeHandle nh;
  ros::Subscriber sub_dbox;
  ros::Subscriber sub_js;
  ros::Publisher joint_pub;
  yumi_dialbox joint1;
  yumi_dialbox dialbox;
  
  sub_dbox = nh.subscribe("dialboxTopic", 1, &yumi_dialbox::DialboxCallback, &dialbox);
  sub_js = nh.subscribe("yumi/joint_states", 1000, &yumi_dialbox::JointStatesCallback, &dialbox);
  joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/yumi/joint_trajectory_pos_controller/command", 1);
  ros::Rate loop_rate(10);	
  while (ros::ok()){
 	
		
	/*ROS_INFO("j1_l %f",dialbox.joint_states.points[0].positions[0]); 
	ROS_INFO("j1_r %f",dialbox.joint_states.points[0].positions[1]);
	ROS_INFO("j2_l %f",dialbox.joint_states.points[0].positions[2]);
	ROS_INFO("j2_r %f",dialbox.joint_states.points[0].positions[3]);
	ROS_INFO("j3_l %f",dialbox.joint_states.points[0].positions[4]);
	ROS_INFO("j3_r %f",dialbox.joint_states.points[0].positions[5]);
	ROS_INFO("j4_l %f",dialbox.joint_states.points[0].positions[6]);
	ROS_INFO("j4_r %f",dialbox.joint_states.points[0].positions[7]);
 	ROS_INFO("j5_l %f",dialbox.joint_states.points[0].positions[8]);
	ROS_INFO("j5_r %f",dialbox.joint_states.points[0].positions[9]);
	ROS_INFO("j6_l %f",dialbox.joint_states.points[0].positions[10]);
	ROS_INFO("j6_r %f",dialbox.joint_states.points[0].positions[11]);
	ROS_INFO("j7_l %f",dialbox.joint_states.points[0].positions[12]);
	ROS_INFO("j7_r %f",dialbox.joint_states.points[0].positions[13]);*/

	joint_pub.publish(dialbox.joint_states);
        ros::spinOnce();
	ros::Duration(0.005).sleep();
	
	}


  return 0;
} 
