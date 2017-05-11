

/* In the .h file is class yumi_dialbox implementation.  In the constructor we've have redefined the names of each yumi joints and the time duration */
#include <ctime>
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <dialbox/dialboxState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


class yumi_dialbox{
  private:
    void init();

    
  public:
    std::vector <unsigned char> joints_positions;
    trajectory_msgs::JointTrajectory joint_states;
    trajectory_msgs::JointTrajectory joint_states_init;
    void set_names(std::vector<std::string>& joint_trajectory_names);
    std::vector<std::string> name;
    void set_value(double val);
    void set_positions(trajectory_msgs::JointTrajectory& joint_trajectory,int i,std::vector<double> new_positions); 
    void DialboxCallback(const dialbox::dialboxState::ConstPtr& msg);
    void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    yumi_dialbox();
    ~yumi_dialbox();
    float dial_value;
    int id;
    double value;
    double delta;
    std::vector<double> dial_array;
    bool START;
    double dial_value_before;
    //int n_v;
 
 
};

    yumi_dialbox::yumi_dialbox(){
	joint_states.points.resize(1);
	joint_states_init.points.resize(1);
	joint_states.points[0].positions.resize(14);
	joint_states.points[0].velocities.resize(14);//
	joint_states.points[0].accelerations.resize(14);//
	joint_states_init.points[0].positions.resize(14);
	joint_states.joint_names.resize(14);
	joint_states_init.joint_names.resize(14);

	joint_states.joint_names[0]= "yumi_joint_1_l";
	joint_states.joint_names[1]= "yumi_joint_1_r";
	joint_states.joint_names[2]= "yumi_joint_2_l";
	joint_states.joint_names[3]= "yumi_joint_2_r";
	joint_states.joint_names[4]= "yumi_joint_3_l";
	joint_states.joint_names[5]= "yumi_joint_3_r";
	joint_states.joint_names[6]= "yumi_joint_4_l";
	joint_states.joint_names[7]= "yumi_joint_4_r";
	joint_states.joint_names[8]= "yumi_joint_5_l";
	joint_states.joint_names[9]= "yumi_joint_5_r";
	joint_states.joint_names[10]="yumi_joint_6_l";
	joint_states.joint_names[11]="yumi_joint_6_r";
	joint_states.joint_names[12]="yumi_joint_7_l";
	joint_states.joint_names[13]="yumi_joint_7_r";
	joint_states_init.joint_names=joint_states.joint_names;
	dial_array.resize(8);
	joint_states.points[0].positions[0]=-2;
	joint_states.points[0].positions[7]=-2.00004;
	joint_states.points[0].positions[1]=0.299542;
	joint_states.points[0].positions[8]=1.7;
	joint_states.points[0].positions[2]=0.0;
	joint_states.points[0].positions[9]=0.0;
	joint_states.points[0].positions[3]=0.0;
	joint_states.points[0].positions[10]=0.0;
	joint_states.points[0].positions[4]=-2.27;
	joint_states.points[0].positions[11]=0.5;
	joint_states.points[0].positions[5]=0.0;
	joint_states.points[0].positions[12]=0.5998;
	joint_states.points[0].positions[6]=0.0;
	joint_states.points[0].positions[13]=-2.0;
	for (int i = 0; i < 14; ++i){
		joint_states.points[0].velocities[i]=0.0;
		joint_states.points[0].accelerations[i]=0.0;
	}

	joint_states.points[0].time_from_start = ros::Duration(0,200);
	joint_states_init.points[0].time_from_start = ros::Duration(0,200);
	START=true;

    }
    yumi_dialbox::~yumi_dialbox() {}


    void yumi_dialbox::set_value(double val){
	value = val;
    }	


    /*void yumi_dialbox::sjoint_states_initet_names( std::vector<std::string>& joint_trajectory_names){

	joint_states.joint_names=joint_trajectory_names;
    }*/


    
    void yumi_dialbox::set_positions(trajectory_msgs::JointTrajectory& joint_trajectory,int i,std::vector<double> new_positions){
		
		joint_states.points[i].positions=new_positions;

    }

    void yumi_dialbox::DialboxCallback(const dialbox::dialboxState::ConstPtr& msg){
	std::vector<double> variation;
	variation.resize(8);
	id=msg->dial;
	//n_v=int (msg->dialValue)%16;
 	dial_value=((msg->dialValue/16.0)*2*M_PI);
	dial_value_before=dial_array[id];
	dial_array[id]=dial_value; //MODIFICACIÓ DEL DIAL
	variation[id]=dial_value-dial_value_before;
	


	if (START){
		START=false;
		joint_states.points[0].positions=joint_states_init.points[0].positions;
	}
  	if (-2.941<joint_states.points[0].positions[0]+variation[0]<2.941){
		joint_states.points[0].positions[0]=joint_states_init.points[0].positions[0]+variation[0]; //1_l
	}
	if (-2.94088<joint_states_init.points[0].positions[1]+variation[0]<2.94088){
		joint_states.points[0].positions[1]=joint_states_init.points[0].positions[1]+variation[0]; //1_r
	}
	if (-2.505<joint_states_init.points[0].positions[2]+variation[1]<1.01){//<0.8){//0.759){
		joint_states.points[0].positions[2]=joint_states_init.points[0].positions[2]+variation[1]; //2_l
	}
	if (-2.50455<joint_states_init.points[0].positions[3]+variation[1]<1.01){//<0.8){//<0.759218){
		joint_states.points[0].positions[3]=joint_states_init.points[0].positions[3]+variation[1]; //2_r
	}
	if (-2.155<joint_states_init.points[0].positions[4]+variation[2]<1.396){
		joint_states.points[0].positions[4]=joint_states_init.points[0].positions[4]+variation[2]; //3_l
	}
	if (-2.15548<joint_states_init.points[0].positions[5]+variation[2]<1.39626){
		joint_states.points[0].positions[5]=joint_states_init.points[0].positions[5]+variation[2]; //3_r
	}
	if (-5.061<joint_states_init.points[0].positions[6]+variation[3]<5.061){
		joint_states.points[0].positions[6]=joint_states_init.points[0].positions[6]+variation[3]; //4_l
	}
	if (-5.06145<joint_states_init.points[0].positions[7]+variation[3]<5.06145){
		joint_states.points[0].positions[7]=joint_states_init.points[0].positions[7]+variation[3]; //4_r
	}
	if (-1.536<joint_states_init.points[0].positions[8]+variation[4]<2.409){
		joint_states.points[0].positions[8]=joint_states_init.points[0].positions[8]+variation[4]; //5_l
	}
	if (-1.53589<joint_states_init.points[0].positions[9]+variation[4]<2.40855){
		joint_states.points[0].positions[9]=joint_states_init.points[0].positions[9]+variation[4]; //5_r
	}
	if (-3.997<joint_states_init.points[0].positions[10]+variation[5]<3.997){
		joint_states.points[0].positions[10]=joint_states_init.points[0].positions[10]+variation[5]; //6_l
	}
	if (-3.9968<joint_states_init.points[0].positions[11]+variation[5]<3.9968){
		joint_states.points[0].positions[11]=joint_states_init.points[0].positions[11]+variation[5]; //6_r
	}
	if (-2.941<joint_states_init.points[0].positions[12]+variation[6]<2.941){
		joint_states.points[0].positions[12]=joint_states_init.points[0].positions[12]+variation[6]; //7_l
	}
	if (-2.94088<joint_states_init.points[0].positions[13]+variation[6]<2.94088){
		joint_states.points[0].positions[13]=joint_states_init.points[0].positions[13]+variation[6]; //7_r
	}	
	/*ROS_INFO("v1_l %f",variation[0]); 
	ROS_INFO("v1_r %f",variation[0]);
	ROS_INFO("v2_l %f",variation[1]);
	ROS_INFO("v2_r %f",variation[1]);
	ROS_INFO("v3_l %f",variation[2]);
	ROS_INFO("v3_r %f",variation[2]);
	ROS_INFO("v4_l %f",variation[3]);
	ROS_INFO("v4_r %f",variation[3]);
 	ROS_INFO("v5_l %f",variation[4]);
	ROS_INFO("v5_r %f",variation[4]);
	ROS_INFO("v6_l %f",variation[5]);
	ROS_INFO("v6_r %f",variation[5]);
	ROS_INFO("v7_l %f",variation[6]);
	ROS_INFO("v7_r %f",variation[6]);*/

    }

    //comprovar les limitacions
    void yumi_dialbox::JointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){

	joint_states.joint_names=msg->name;
	joint_states_init.joint_names=msg->name;
	joint_states_init.points[0].positions=msg->position;

	

    }



