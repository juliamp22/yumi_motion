
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
#include <yumi_motion/yumimotionConfig.h>
#include <dynamic_reconfigure/server.h>
#include <yumi_hw/YumiGrasp.h>


class yumi_dialbox{
  private:
    void init();

    
  public:
    std::vector <unsigned char> joints_positions;
    trajectory_msgs::JointTrajectory joint_states;
    trajectory_msgs::JointTrajectory joint_states_init;
    void set_names(std::vector<std::string>& joint_trajectory_names);
    std::vector<std::string> name;
    void set_values_dk(bool val1, bool val2);
    void set_values_ik(double x_l, double y_l, double z_l, double z_tcp_l, bool T, int n_solution, bool left, bool right,double x_r, double y_r, double z_r, double z_tcp_r, bool gripper_open_left, bool gripper_close_left, bool gripper_open_right,bool gripper_close_right);
    void set_positions(trajectory_msgs::JointTrajectory& joint_trajectory,int i,std::vector<double> new_positions); 
    
    void DialboxCallback(const dialbox::dialboxState::ConstPtr& msg);
    void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    //void cfgcallback(const yumi_motion::yumimotionConfig &config, uint32_t level);
    yumi_dialbox();
    ~yumi_dialbox();
    float dial_value;
    int id;
    double value;
    double delta;
    std::vector<double> dial_array;
    bool START;
    double dial_value_before;
    bool left;
    bool right;
    double x;
    double y;
    double l;
    double z_tcp;
    double x_l;
    double y_l;
    double l_l;
    double z_tcp_l;
    double x_r;
    double y_r;
    double l_r;
    double z_tcp_r;
    int n_solution;
    bool configure_ik;
    std::vector<double> variation;
    yumi_hw::YumiGrasp gripper_open_left;
    yumi_hw::YumiGrasp gripper_close_left;
    yumi_hw::YumiGrasp gripper_open_right;
    yumi_hw::YumiGrasp gripper_close_right;
    bool is_dialbox;
    //int n_v;
 
 
};

     yumi_dialbox::yumi_dialbox(){
	x=300;
    	y=200;
	l=500;
	z_tcp=200;
	START=false;
	n_solution=1;

        //names
	joint_states.joint_names.resize(14);
	joint_states_init.joint_names.resize(14);
	//points

	joint_states.points.resize(1);
	joint_states_init.points.resize(1);

	//positions,accelerations
	joint_states.points[0].positions.resize(14);
	joint_states_init.points[0].positions.resize(14);
	joint_states.points[0].velocities.resize(14);//
	joint_states.points[0].accelerations.resize(14);//

	//JOINT_STATES names

	joint_states.joint_names[0]= "yumi_joint_1_l";
	joint_states.joint_names[1]= "yumi_joint_2_l";
	joint_states.joint_names[2]= "yumi_joint_3_l";
	joint_states.joint_names[3]= "yumi_joint_4_l";
	joint_states.joint_names[4]= "yumi_joint_5_l";
	joint_states.joint_names[5]= "yumi_joint_6_l";
	joint_states.joint_names[6]= "yumi_joint_7_l";
	joint_states.joint_names[7]= "yumi_joint_1_r";
	joint_states.joint_names[8]= "yumi_joint_2_r";
	joint_states.joint_names[9]= "yumi_joint_3_r";
	joint_states.joint_names[10]="yumi_joint_4_r";
	joint_states.joint_names[11]="yumi_joint_5_r";
	joint_states.joint_names[12]="yumi_joint_6_r";
	joint_states.joint_names[13]="yumi_joint_7_r";

	joint_states_init.joint_names[0]= "yumi_joint_1_l";
	joint_states_init.joint_names[1]= "yumi_joint_2_l";
	joint_states_init.joint_names[2]= "yumi_joint_3_l";
	joint_states_init.joint_names[3]= "yumi_joint_4_l";
	joint_states_init.joint_names[4]= "yumi_joint_5_l";
	joint_states_init.joint_names[5]= "yumi_joint_6_l";
	joint_states_init.joint_names[6]= "yumi_joint_7_l";
	joint_states_init.joint_names[7]= "yumi_joint_1_r";
	joint_states_init.joint_names[8]= "yumi_joint_2_r";
	joint_states_init.joint_names[9]= "yumi_joint_3_r";
	joint_states_init.joint_names[10]="yumi_joint_4_r";
	joint_states_init.joint_names[11]="yumi_joint_5_r";
	joint_states_init.joint_names[12]="yumi_joint_6_r";
	joint_states_init.joint_names[13]="yumi_joint_7_r";
	
	//initial position

	joint_states_init.points[0].positions[0]=-2;
	joint_states_init.points[0].positions[1]=-2.0004;
	joint_states_init.points[0].positions[2]=0.299542;
	joint_states_init.points[0].positions[3]=1.7;
	joint_states_init.points[0].positions[4]=0.0;
	joint_states_init.points[0].positions[5]=0.0;
	joint_states_init.points[0].positions[6]=0.0;
	joint_states_init.points[0].positions[7]=0.0;
	joint_states_init.points[0].positions[8]=-2.27;
	joint_states_init.points[0].positions[9]=0.5;
	joint_states_init.points[0].positions[10]=0.0;
	joint_states_init.points[0].positions[11]=0.5998;
	joint_states_init.points[0].positions[12]=0.0;
	joint_states_init.points[0].positions[13]=-2.0;
	joint_states.points[0]=joint_states_init.points[0];
	dial_array.resize(8);
	
	/*for (int i = 0; i < 14; ++i){
		
		joint_states.points[0].velocities[i]=0.0;
		joint_states.points[0].accelerations[i]=0.0;
	}*/

	joint_states.points[0].time_from_start = ros::Duration(0,500);
	joint_states_init.points[0].time_from_start = ros::Duration(0,500);
	START=true;

    }
    yumi_dialbox::~yumi_dialbox() {}


    void yumi_dialbox::set_values_dk(bool val1, bool val2){
	left=val1;
    	right=val2;
	ROS_INFO("left %d",left);
	ROS_INFO("right %d",right);
    }	

    void yumi_dialbox::set_values_ik(double val1, double val2, double val3, double val4, bool val5, int val6, bool val7, bool val8, double val9, double val10, double val11, double val12,bool val13, bool val14, bool val15, bool val16){
	x_l=val1;
    	y_l=val2;
	l_l=val3;
	z_tcp_l=val4;
	START=val5;
	left=val7;
	right=val8;
	n_solution=val6;
	x_r=val9;
    	y_r=val10;
	l_r=val11;
	z_tcp_r=val12;
	ROS_INFO("gripper_open_left %d",val13);
	ROS_INFO("gripper_close_left %d",val14);
	ROS_INFO("gripper_open_right %d",val15);
	ROS_INFO("gripper_close_right %d",val16);
    	
	if((val13==1)&&(val14==0)){
		gripper_open_left.request.gripper_id=1;
		gripper_close_left.request.gripper_id=0;
	}
	else if((val13==0)&&(val14==1)){
		gripper_open_left.request.gripper_id=0;
		gripper_close_left.request.gripper_id=1;
	}
	else{
		gripper_open_left.request.gripper_id=0;
		gripper_close_left.request.gripper_id=0;
	}
	if((val15==1)&&(val16==0)){
		gripper_open_right.request.gripper_id=2;
		gripper_close_right.request.gripper_id=0;
	}
	else if((val15==0)&&(val16==1)){
		gripper_open_right.request.gripper_id=0;
		gripper_close_right.request.gripper_id=2;
	}
	else{
		gripper_open_right.request.gripper_id=0;
		gripper_close_right.request.gripper_id=0;
	}


	
    }


    /*void yumi_dialbox::sjoint_states_initet_names( std::vector<std::string>& joint_trajectory_names){

	joint_states.joint_names=joint_trajectory_names;
    }*/


    /*void yumi_dialbox::cfgcallback( yumi_motion::yumimotionConfig &config, uint32_t level){
	set_values(config.left_cfg, config.right_cfg);
	
	


    }*/
    void yumi_dialbox::set_positions(trajectory_msgs::JointTrajectory& joint_trajectory,int i,std::vector<double> new_positions){
		
		joint_states.points[i].positions=new_positions;

    }
   
   
    
    //comprovar les limitacions

