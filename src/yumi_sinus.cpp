#include "ros/ros.h"
#include <sstream>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "std_msgs/String.h"


//#include <dynamic_reconfigure/server.h>


#include <dynamic_reconfigure/server.h>
#include <yumi_hw/yumihwConfig.h>
//#include <yumi_hw/yumi_hw_rapid.h>
#include <math.h>       /* sin */
#include <yumi_hw/YumiGrasp.h>
//#include "node_example_core.h"



//std::vector<float> ConfigFile(14);



bool START=true;
int i=0;
double yumi_joint_1_l;
double yumi_joint_2_l;
double yumi_joint_3_l;
double yumi_joint_4_l;
double yumi_joint_5_l;
double yumi_joint_6_l;
double yumi_joint_7_l;
double yumi_joint_1_r;
double yumi_joint_2_r;
double yumi_joint_3_r;
double yumi_joint_4_r;
double yumi_joint_5_r;
double yumi_joint_6_r;
double yumi_joint_7_r;
double valor;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish() //We need a class if we want to get something information from the joint_states subscriber.
  {
    //Topic which wants to be published//susbscriber
	ros::NodeHandle nh; 
        ros::ServiceClient gripper_client_open; 
  	ros::ServiceClient gripper_client_close; 
	joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/yumi/joint_trajectory_pos_controller/command", 1);
	gripper_client_close = nh.serviceClient<yumi_hw::YumiGrasp>("/yumi/yumi_gripper/do_grasp");
  	gripper_client_open = nh.serviceClient<yumi_hw::YumiGrasp>("/yumi/yumi_gripper/release_grasp");
        yumi_hw::YumiGrasp gripper_open_left;
        yumi_hw::YumiGrasp gripper_close_left;
        yumi_hw::YumiGrasp gripper_open_right;
        yumi_hw::YumiGrasp gripper_close_right;

   
	valor=0.0;
	joint_states.joint_names.resize(14);	
	joint_states.points.resize(2);
	joint_states.points[0].positions.resize(14);
        joint_states.points[1].positions.resize(14);	
        new_joint_states.points.resize(1);
	
	joint_states.joint_names[0]="yumi_joint_1_l";
	joint_states.joint_names[1]="yumi_joint_1_r";
	joint_states.joint_names[2]="yumi_joint_2_l";
	joint_states.joint_names[3]="yumi_joint_2_r";
	joint_states.joint_names[4]="yumi_joint_3_l";
	joint_states.joint_names[5]="yumi_joint_3_r";
	joint_states.joint_names[6]="yumi_joint_4_l";
	joint_states.joint_names[7]="yumi_joint_4_r";
	joint_states.joint_names[8]="yumi_joint_5_l";
	joint_states.joint_names[9]="yumi_joint_5_r";
	joint_states.joint_names[10]="yumi_joint_6_l";
	joint_states.joint_names[11]="yumi_joint_6_r";
	joint_states.joint_names[12]="yumi_joint_7_l";
	joint_states.joint_names[13]="yumi_joint_7_r";

	joint_states.points[0].time_from_start = ros::Duration(0,0);
	joint_states.points[1].time_from_start = ros::Duration(0,50);

        joint_states.points[1].positions[0]=-0.1*sin(valor);
	joint_states.points[1].positions[1]=+0.1*sin(valor);
	joint_states.points[1].positions[2]=-0.1*sin(valor);
	joint_states.points[1].positions[3]=+0.1*sin(valor);
	joint_states.points[1].positions[4]=-0.1*sin(valor);
	joint_states.points[1].positions[5]=+0.1*sin(valor);
	joint_states.points[1].positions[6]=-0.1*sin(valor);
	joint_states.points[1].positions[7]=+0.1*sin(valor);
	joint_states.points[1].positions[8]=-0.1*sin(valor);
	joint_states.points[1].positions[9]=+0.1*sin(valor);
	joint_states.points[1].positions[10]=-0.1*sin(valor);
	joint_states.points[1].positions[11]=+0.1*sin(valor);
	joint_states.points[1].positions[12]=1.5-0.1*sin(valor);
	joint_states.points[1].positions[13]=-1.5+0.1*sin(valor);
	gripper_open_left.request.gripper_id=1;
	gripper_close_left.request.gripper_id=1;
	gripper_open_right.request.gripper_id=2;
	gripper_close_right.request.gripper_id=2;
  }


  void callback(const sensor_msgs::JointState::ConstPtr& msg)
  {
	valor=valor+0.031459;
	ROS_INFO("changing joints");

	/*if (START){
                
		new_joint_states.points[0].positions=msg->position;
		START=false;
	}*/
        std::copy(joint_states.points[1].positions.begin(),joint_states.points[1].positions.end(),joint_states.points[0].positions.begin());
	
	//uncomment this if you want to compute the next point

	/*joint_states.points[1].positions[0]=-1*sin(valor);
	joint_states.points[1].positions[1]=+1*sin(valor);
	joint_states.points[1].positions[2]=-1*sin(valor);
	joint_states.points[1].positions[3]=+1*sin(valor);
	joint_states.points[1].positions[4]=-1*sin(valor);
	joint_states.points[1].positions[5]=+1*sin(valor);
	joint_states.points[1].positions[6]=-1*sin(valor);
	joint_states.points[1].positions[7]=+1*sin(valor);
	joint_states.points[1].positions[8]=-1*sin(valor);
	joint_states.points[1].positions[9]=+1*sin(valor);
	joint_states.points[1].positions[10]=-1*sin(valor);
	joint_states.points[1].positions[11]=+1*sin(valor);
	joint_states.points[1].positions[12]=1.5-1*sin(valor);
	joint_states.points[1].positions[13]=-1.5+1*sin(valor);
	
 	if (valor>6.2831){
		valor=valor-6.28;	
	}*/

	//uncomment this if you want to get callback information from the robot	
	/*ROS_INFO("j1l %f",joint_states.points[0].positions[0]);
	ROS_INFO("j1r %f",joint_states.points[0].positions[1]);
	ROS_INFO("j2l %f",joint_states.points[0].positions[2]);
	ROS_INFO("j2r %f",joint_states.points[0].positions[3]);
	ROS_INFO("j3l %f",joint_states.points[0].positions[4]);
	ROS_INFO("j3r %f",joint_states.points[0].positions[5]);
	ROS_INFO("j4l %f",joint_states.points[0].positions[6]);
	ROS_INFO("j4r %f",joint_states.points[0].positions[7]);
	ROS_INFO("j5l %f",joint_states.points[0].positions[8]);
	ROS_INFO("j5r %f",joint_states.points[0].positions[9]);
	ROS_INFO("j6l %f",joint_states.points[0].positions[10]);
	ROS_INFO("j6r %f",joint_states.points[0].positions[11]);
	ROS_INFO("j7l %f",joint_states.points[0].positions[12]);
	ROS_INFO("j7r %f",joint_states.points[0].positions[13]);*/


	joint_pub.publish(joint_states);
	

	START=false;
   	

  }

protected:
  ros::NodeHandle nh; 
  ros::Publisher joint_pub;
  trajectory_msgs::JointTrajectory joint_states;
  trajectory_msgs::JointTrajectory new_joint_states;

};//End of class SubscribeAndPublish


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  //SubscribeAndPublish SAPObject;  //ClassObject, uncoment this in case of work with subscriber topic joint_states
  ros::NodeHandle nh; 
  ros::Publisher joint_pub;
  ros::ServiceClient gripper_client_open; //service declaration
  ros::ServiceClient gripper_client_close; //service declaration
  joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/yumi/joint_trajectory_pos_controller/command", 1);   //moving publisher
  gripper_client_close = nh.serviceClient<yumi_hw::YumiGrasp>("/yumi/yumi_gripper/do_grasp");    //service, do grasping
  gripper_client_open = nh.serviceClient<yumi_hw::YumiGrasp>("/yumi/yumi_gripper/release_grasp");  //service, release grasping
  yumi_hw::YumiGrasp gripper_open_left; //Grasp message
  yumi_hw::YumiGrasp gripper_close_left; //Grasp message
  yumi_hw::YumiGrasp gripper_open_right; //Grasp message
  yumi_hw::YumiGrasp gripper_close_right; //Grasp message

 
  //ros::Subscriber sub; //uncomment this in case of working with subscribers
  //sub = nh.subscribe("/yumi/joint_states", 1, &SubscribeAndPublish::callback,&SAPObject); //uncomment this in case of working with     	joint_states topic (subscriber)

  trajectory_msgs::JointTrajectory joint_states;

  //defining joint names

  joint_states.joint_names.resize(14);	

  joint_states.joint_names[0]="yumi_joint_1_l";
  joint_states.joint_names[1]="yumi_joint_1_r";
  joint_states.joint_names[2]="yumi_joint_2_l";
  joint_states.joint_names[3]="yumi_joint_2_r";
  joint_states.joint_names[4]="yumi_joint_3_l";
  joint_states.joint_names[5]="yumi_joint_3_r";
  joint_states.joint_names[6]="yumi_joint_4_l";
  joint_states.joint_names[7]="yumi_joint_4_r";
  joint_states.joint_names[8]="yumi_joint_5_l";
  joint_states.joint_names[9]="yumi_joint_5_r";
  joint_states.joint_names[10]="yumi_joint_6_l";
  joint_states.joint_names[11]="yumi_joint_6_r";
  joint_states.joint_names[12]="yumi_joint_7_l";
  joint_states.joint_names[13]="yumi_joint_7_r";

  joint_states.points.resize(400);
  //joint_states.points.resize(1);
  //joint_states.points[0].positions.resize(14);
  double valor = 0.0;
   
     
  //for (int k = 0; k < 4; k++ ){
	  for (int i = 0; i < 400; i++ ){
		//making grippers left and right moving
 		gripper_close_right.request.gripper_id=2;
    		gripper_close_left.request.gripper_id=1;
		gripper_open_right.request.gripper_id=2;
    		gripper_open_left.request.gripper_id=1;
    		if(gripper_client_close.call(gripper_close_left)){
		}
    		gripper_client_close.call(gripper_close_right);


		//creating a sinus and setting the 400 points of this sinus to a joint configuration
	
		joint_states.points[i].positions.resize(14);
		joint_states.points[i].positions[0]=-0.5*sin(valor);
		joint_states.points[i].positions[1]=+0.5*sin(valor);
		joint_states.points[i].positions[2]=-0.5*sin(valor);
		joint_states.points[i].positions[3]=+0.5*sin(valor);
		joint_states.points[i].positions[4]=-0.5*sin(valor);
		joint_states.points[i].positions[5]=+0.5*sin(valor);
		joint_states.points[i].positions[6]=-0.5*sin(valor);
		joint_states.points[i].positions[7]=+0.5*sin(valor);
		joint_states.points[i].positions[8]=-0.5*sin(valor);
		joint_states.points[i].positions[9]=+0.5*sin(valor);
		joint_states.points[i].positions[10]=-0.5*sin(valor);
		joint_states.points[i].positions[11]=+0.5*sin(valor);
		joint_states.points[i].positions[12]=2-0.5*sin(valor);
		joint_states.points[i].positions[13]=-2+0.5*sin(valor);
		joint_states.points[i].time_from_start = ros::Duration(0.02*(i+1)); //change 0.02 value depending number of joints and joints discretization

		//Showing joint_states points
		ROS_INFO("j1l %f",joint_states.points[i].positions[0]);
		ROS_INFO("j1r %f",joint_states.points[i].positions[1]);
		ROS_INFO("j2l %f",joint_states.points[i].positions[2]);
		ROS_INFO("j2r %f",joint_states.points[i].positions[3]);
		ROS_INFO("j3l %f",joint_states.points[i].positions[4]);
		ROS_INFO("j3r %f",joint_states.points[i].positions[5]);
		ROS_INFO("j4l %f",joint_states.points[i].positions[6]);
		ROS_INFO("j4r %f",joint_states.points[i].positions[7]);
		ROS_INFO("j5l %f",joint_states.points[i].positions[8]);
		ROS_INFO("j5r %f",joint_states.points[i].positions[9]);
		ROS_INFO("j6l %f",joint_states.points[i].positions[10]);
		ROS_INFO("j6r %f",joint_states.points[i].positions[11]);
		ROS_INFO("j7l %f",joint_states.points[i].positions[12]);
		ROS_INFO("j7r %f",joint_states.points[i].positions[13]);

		valor=valor+0.031459*4./5.;
						
	  }

	joint_pub.publish(joint_states); //publishing the configuration message to the robot
	


	if(gripper_client_open.call(gripper_open_left)){
		
	
	}
    	
    	if(gripper_client_open.call(gripper_open_right)){
	}

	
	ros::Duration(0.02).sleep();

  ros::spin();

  return 0;
}


