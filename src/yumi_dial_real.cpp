/* With this node, you can get information from the dialbox so as to sending to yumi Simulated or Real robot. In the .cpp file we have the 
 subsriber topic dialboxTopic and the publisher topic /yumi/joint_trajectory_pos_controller/command */

#include <yumi_motion/yumi_dial_real.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <dialbox/dialbox.h>
#include "dialbox/dialboxState.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>
#include <yumi_motion/yumimotionConfig.h>
#include <yumi_hw/YumiGrasp.h>

void cfgcallback(yumi_motion::yumimotionConfig &config, uint32_t level,yumi_dialbox *dialbox){
	dialbox->set_values_dk(config.left_cfg, config.right_cfg);
	ROS_INFO("is_left %d",config.left_cfg);
	ROS_INFO("is_right %d",config.right_cfg);
	ROS_INFO("is_left %d",dialbox->left);
	ROS_INFO("is_right %d",dialbox->right);
    }
void yumi_dialbox::JointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
	//JOINT_STATES.points


	//joint_states=joint_states_init;
	/*if (START){
		joint_states_init.points[0].positions[0]=msg->position[0]; //1_l
		joint_states_init.points[0].positions[1]=msg->position[2]; //2_l
		joint_states_init.points[0].positions[2]=msg->position[4]; //3_l
		joint_states_init.points[0].positions[3]=msg->position[6]; //4_l
		joint_states_init.points[0].positions[4]=msg->position[8]; //5_l
		joint_states_init.points[0].positions[5]=msg->position[10]; //6_l
		joint_states_init.points[0].positions[6]=msg->position[12]; //7_l
		joint_states_init.points[0].positions[7]=msg->position[1]; //1_r
		joint_states_init.points[0].positions[8]=msg->position[3]; //2_r
		joint_states_init.points[0].positions[9]=msg->position[5]; //3_r
		joint_states_init.points[0].positions[10]=msg->position[7]; //4_r
		joint_states_init.points[0].positions[11]=msg->position[9]; //5_r
		joint_states_init.points[0].positions[12]=msg->position[11]; //6_r
		joint_states_init.points[0].positions[13]=msg->position[13]; //7_r
		START=false;
		ROS_INFO("v1_l_n %f",msg->position[0]); 
		ROS_INFO("v1_r %f",msg->position[1]);
		ROS_INFO("v2_l %f",msg->position[2]);
		ROS_INFO("v2_r %f",msg->position[3]);
		ROS_INFO("v3_l %f",msg->position[4]);
		ROS_INFO("v3_r %f",msg->position[5]);
		ROS_INFO("v4_l %f",msg->position[6]);
		ROS_INFO("v4_r %f",msg->position[7]);
	 	ROS_INFO("v5_l %f",msg->position[8]);
		ROS_INFO("v5_r %f",msg->position[9]);
		ROS_INFO("v6_l %f",msg->position[10]);
		ROS_INFO("v6_r %f",msg->position[11]);
		ROS_INFO("v7_l %f",msg->position[12]);
		ROS_INFO("v7_r %f",msg->position[13]);
	}
		/*ROS_INFO("v1_l_n_1 %f",msg->position[0]); 
		ROS_INFO("v1_r %f",msg->position[1]);
		ROS_INFO("v2_l %f",msg->position[2]);
		ROS_INFO("v2_r %f",msg->position[3]);
		ROS_INFO("v3_l %f",msg->position[4]);
		ROS_INFO("v3_r %f",msg->position[5]);
		ROS_INFO("v4_l %f",msg->position[6]);
		ROS_INFO("v4_r %f",msg->position[7]);
	 	ROS_INFO("v5_l %f",msg->position[8]);
		ROS_INFO("v5_r %f",msg->position[9]);
		ROS_INFO("v6_l %f",msg->position[10]);
		ROS_INFO("v6_r %f",msg->position[11]);
		ROS_INFO("v7_l %f",msg->position[12]);
		ROS_INFO("v7_r %f",msg->position[13]);*/
	
	
   }
	
   void yumi_dialbox::DialboxCallback(const dialbox::dialboxState::ConstPtr& msg){

	//gripper_open_left.request.gripper_id=0;
	//gripper_close_left.request.gripper_id=0;
	//gripper_open_right.request.gripper_id=0;
	//gripper_close_right.request.gripper_id=0;
	//yumi_dialbox param;
	variation.resize(8);
	id=msg->dial;
	//n_v=int (msg->dialValue)%16;
 	dial_value=((msg->dialValue/16.0)*2*M_PI);
	dial_value_before=dial_array[id];
	dial_array[id]=dial_value; //MODIFICACIÓ DEL DIAL
	variation[id]=dial_value-dial_value_before;
	ROS_INFO("is_left %d",left);
	ROS_INFO("is_right %d",right);

	switch(id){
	
	case 0:
		if (left){
	
			if (-2.941<joint_states.points[0].positions[0]+dial_array[0]<2.941){
				joint_states.points[0].positions[0]=joint_states_init.points[0].positions[0]+dial_array[id]; //1_l
			}
		}
		if (right){
			if (-2.941<joint_states_init.points[0].positions[7]+dial_array[0]<2.941){
				joint_states.points[0].positions[7]=joint_states_init.points[0].positions[7]+dial_array[id]; //1_r
			}
		}
		break;
	case 1:
		if (left){
			if (-2.505<joint_states_init.points[0].positions[1]+dial_array[1]<1.01){//<0.8){//0.759){
				joint_states.points[0].positions[1]=joint_states_init.points[0].positions[1]+dial_array[id]; //2_l
			}
		}
		if (right){
			if (-2.505<joint_states_init.points[0].positions[8]+dial_array[1]<1.01){//<0.8){//<0.759218){
				joint_states.points[0].positions[8]=joint_states_init.points[0].positions[8]+dial_array[id]; //2_r
			}
		}
		break;
	case 2:
		if (left){
			if (-2.155<joint_states_init.points[0].positions[2]+dial_array[2]<1.396){
				joint_states.points[0].positions[2]=joint_states_init.points[0].positions[2]+dial_array[2]; //3_l
			}
		}
		if (right){
			if (-2.155<joint_states_init.points[0].positions[9]+dial_array[2]<1.396){
				joint_states.points[0].positions[9]=joint_states_init.points[0].positions[9]+dial_array[2]; //3_r
			}
		}
		break;
	case 3:
		if (left){
			if (-5.061<joint_states_init.points[0].positions[3]+dial_array[3]<5.061){
				joint_states.points[0].positions[3]=joint_states_init.points[0].positions[3]+dial_array[3]; //4_l
			}
		}
		if (right){
			if (-5.061<joint_states_init.points[0].positions[10]+dial_array[3]<5.061){
				joint_states.points[0].positions[10]=joint_states_init.points[0].positions[10]+dial_array[3]; //4_r
			}
		}
		break;
	case 4:
		if (left){
			if (-1.536<joint_states_init.points[0].positions[4]+dial_array[4]<2.409){
				joint_states.points[0].positions[4]=joint_states_init.points[0].positions[4]+dial_array[4]; //5_l
			}
		}
		if (right){
			if (-1.536<joint_states_init.points[0].positions[11]+dial_array[4]<2.40855){
				joint_states.points[0].positions[11]=joint_states_init.points[0].positions[11]+dial_array[4]; //5_r
			}
		}
		break;
	case 5:
		if (left){
			if (-3.997<joint_states_init.points[0].positions[5]+dial_array[5]<3.997){
				joint_states.points[0].positions[5]=joint_states_init.points[0].positions[5]+dial_array[5]; //6_l
			}
		}
		if (right){
			if (-3.9968<joint_states_init.points[0].positions[12]+dial_array[5]<3.9968){
				joint_states.points[0].positions[12]=joint_states_init.points[0].positions[12]+dial_array[5]; //6_r
			}
		}
		break;
	case 6:
		if (left){
			if (-2.941<joint_states_init.points[0].positions[6]+dial_array[6]<2.941){
				joint_states.points[0].positions[6]=joint_states_init.points[0].positions[6]+dial_array[6]; //7_l
			}
		}
		if (right){
			if (-2.94088<joint_states_init.points[0].positions[13]+dial_array[6]<2.94088){
				joint_states.points[0].positions[13]=joint_states_init.points[0].positions[13]+dial_array[6]; //7_r
			}
		}
		break;
	

	case 7:
		ROS_INFO("dial_array_7 %f",dial_array[7]); 
		if (left){
			if (dial_array[7]>0){
				
				gripper_open_left.request.gripper_id=1;
				gripper_close_left.request.gripper_id=0;
			}
			else if (dial_array[7]<0){
				gripper_open_left.request.gripper_id=0;
				gripper_close_left.request.gripper_id=1;
			}
			else{
				gripper_open_left.request.gripper_id=0;
				gripper_close_left.request.gripper_id=0;
			}
		}
		if (right){
			if (dial_array[7]>0){
				gripper_open_right.request.gripper_id=2;
				gripper_close_right.request.gripper_id=0;
			}
			else if (dial_array[7]<0){
				gripper_open_right.request.gripper_id=0;
				gripper_close_right.request.gripper_id=2;
			}
			else{
				gripper_open_right.request.gripper_id=0;
				gripper_close_right.request.gripper_id=0;
			}
		}
		break;
		

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

 

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener_to_dialbox");
  
  ros::NodeHandle nh;
  ros::Subscriber sub_dbox;
  ros::Subscriber sub_js;
  ros::Publisher joint_pub;
  ros::ServiceClient gripper_client_open; 
  ros::ServiceClient gripper_client_close; 
  yumi_dialbox dialbox;
  dynamic_reconfigure::Server<yumi_motion::yumimotionConfig> server;
  dynamic_reconfigure::Server<yumi_motion::yumimotionConfig>::CallbackType f;
	
  f=boost::bind(&cfgcallback,_1,_2, &dialbox);
  server.setCallback(f);
  sub_dbox = nh.subscribe("dialboxTopic", 1, &yumi_dialbox::DialboxCallback, &dialbox);
  sub_js = nh.subscribe("/yumi/joint_states", 1000, &yumi_dialbox::JointStatesCallback, &dialbox);
  gripper_client_close = nh.serviceClient<yumi_hw::YumiGrasp>("/yumi/yumi_gripper/do_grasp");
  gripper_client_open = nh.serviceClient<yumi_hw::YumiGrasp>("/yumi/yumi_gripper/release_grasp");
  joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/yumi/joint_trajectory_pos_controller/command", 1);
  ros::Rate loop_rate(100);	
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
	ROS_INFO("j7_l %f",dialbox.joint_states.pyumi_dialbox::JointStatesCallbackoints[0].positions[12]);
	ROS_INFO("j7_r %f",dialbox.joint_states.points[0].positions[13]);*/
	
	
	
	if((dialbox.left)&&(dialbox.gripper_close_left.request.gripper_id==1)){
		ROS_INFO("gripper_close_left %d",dialbox.gripper_close_left.request.gripper_id); 
		gripper_client_close.call(dialbox.gripper_close_left);
		
	}
	else if((dialbox.left)&&(dialbox.gripper_open_left.request.gripper_id==1)){

		ROS_INFO("gripper_open_left %d",dialbox.gripper_open_left.request.gripper_id); 
		gripper_client_open.call(dialbox.gripper_open_left);
	}
	else{
		ROS_INFO("gripper_close_left %d",dialbox.gripper_close_left.request.gripper_id); 
		ROS_INFO("gripper_open_left %d",dialbox.gripper_open_left.request.gripper_id); 		
		ROS_INFO("do_nothing_left");
	}
	if(dialbox.right&&(dialbox.gripper_close_right.request.gripper_id==2)){
		ROS_INFO("gripper_close_right %d",dialbox.gripper_close_right.request.gripper_id); 
		gripper_client_close.call(dialbox.gripper_close_right);
	}
	else if(dialbox.right&&(dialbox.gripper_open_right.request.gripper_id==2)){
		ROS_INFO("gripper_open_right %d",dialbox.gripper_open_right.request.gripper_id); 
		gripper_client_open.call(dialbox.gripper_open_right);
	}
	else{
		ROS_INFO("do_nothing_right");
		ROS_INFO("gripper_close_right %d",dialbox.gripper_close_right.request.gripper_id);
		ROS_INFO("gripper_copen_right %d",dialbox.gripper_open_right.request.gripper_id); 

	}

        
	//joint_pub.publish(dialbox.joint_states_init);
	joint_pub.publish(dialbox.joint_states);
        ros::spinOnce();
	//ros::Duration(0.005).sleep();
	
	}


  return 0;
} 
