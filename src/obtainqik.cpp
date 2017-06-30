#include <mt/mt.h>
#include <mt/scalar.h>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>
#include <yumik/YumIK.h>
#include <yumik/YumIKResponse.h>
#include <dynamic_reconfigure/server.h>
#include <yumi_motion/ikConfig.h>
#include <yumi_motion/yumi_dial_real.h>
#include <sensor_msgs/JointState.h>
#include "dialbox/dialboxState.h"

//NaN:the position is not reachable.
//Respect the shoulder, it is reachable (r=516,theta, gamma ) --> espherical coord.
//It will only work with RIGHT ARM!!!!
void cfgcallback(yumi_motion::ikConfig &config, uint32_t level,yumi_dialbox *reconfigure){
	if (reconfigure->is_dialbox==false){
		reconfigure->set_values_ik(config.X_l, config.Y_l,config.L_l,config.Z_tcp_l,config.START,config.n_solution,config.left,config.right,config.X_r, config.Y_r,config.L_r,config.Z_tcp_r,config.gripper_open_left,config.gripper_close_left,config.gripper_open_right,config.gripper_close_right );
		reconfigure->is_dialbox=false;
		//ROS_INFO("X_l %f",config.X_l);
		//ROS_INFO("Y_l %f",config.Y_l);
		//ROS_INFO("L_l %f",config.L_l);
		//ROS_INFO("Z_tcp_l %f",config.Z_tcp_l);
		//ROS_INFO("START %f",config.START);
		//ROS_INFO("left %f",config.left);
		//ROS_INFO("right %f",config.right);
		//ROS_INFO("X_r %f",config.X_r);
		//ROS_INFO("Y_r %f",config.Y_r);
		//ROS_INFO("L_r %f",config.L_r);
		/*ROS_INFO("Z_tcp_r %f",config.Z_tcp_r);
		ROS_INFO("gripper_open_left %d",config.gripper_open_left);
		ROS_INFO("gripper_close_left %d",config.gripper_close_left);
		ROS_INFO("gripper_open_right %d",config.gripper_open_right);
		ROS_INFO("gripper_close_right %d",config.gripper_close_right);*/



	}
   
		
    }
void printTransform(mt::Transform T)
{
	mt::Rotation R=T.getRotation();
	mt::Matrix3x3 M=R.getMatrix();
	mt::Point3 d = T.getTranslation();
	std::cout<<M[0][0]<<" "<<M[0][1]<<" "<<M[0][2]<<" "<<d[0]<< std::endl;
	std::cout<<M[1][0]<<" "<<M[1][1]<<" "<<M[1][2]<<" "<<d[1]<< std::endl;
	std::cout<<M[2][0]<<" "<<M[2][1]<<" "<<M[2][2]<<" "<<d[2]<< std::endl;
	std::cout<<"0 0 0 1"<< std::endl;	
    	std::cout<<"T = (" << d[0] << ", " << d[1] <<", " << d[2];
    	std::cout<< ", "<< R[0] << ", " << R[1] << ", "<< R[2] << ", "<< R[3] <<")"<<std::endl;
}
void yumi_dialbox::DialboxCallback(const dialbox::dialboxState::ConstPtr& msg){
	id=msg->dial;
	//n_v=int (msg->dialValue)%16;
 	dial_value=(msg->dialValue/16.0)*500.0;
	dial_value_before=dial_array[id];
	dial_array[id]=dial_value; //MODIFICACIÓ DEL DIAL
	switch(id){

		case 0:
			if (dial_array[id]/30.0>15){
				n_solution=15;
			}	
			else if (dial_array[id]/30.0<0){
				n_solution=0;
			}
			else{
				n_solution= int(dial_array[id]/30.0);
			}	
			ROS_INFO("dial_array %f",dial_array[id]);
			break;
		case 1:
			x=dial_array[id];
			break;		
		case 3:
			y=dial_array[id];
			break;
		case 5:
			l=dial_array[id];
			break;
			
		case 7:
			z_tcp=dial_array[id];
			break;

		
	}
	ROS_INFO("c_x %f",x);
	ROS_INFO("c_y %f",y);
	ROS_INFO("c_l %f",l);
	ROS_INFO("c_z_tcp %f",z_tcp);
	ROS_INFO("c_START %d",START);
	ROS_INFO("c_n_solution %d",n_solution);
	ROS_INFO("id %d",id);

	
	is_dialbox=true;

	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "YumIK_client");
   	ROS_INFO("Starting YumIK_client");
  	ros::NodeHandle nh;
	ros::ServiceClient gripper_client_open; 
  	ros::ServiceClient gripper_client_close; 
    	ros::ServiceClient Yumi_client_left = nh.serviceClient<yumik::YumIK>("Yumi/IKSolverLeft");
   	ros::ServiceClient Yumi_client_right = nh.serviceClient<yumik::YumIK>("Yumi/IKSolverRight"); 
	gripper_client_close = nh.serviceClient<yumi_hw::YumiGrasp>("/yumi/yumi_gripper/do_grasp");
  	gripper_client_open = nh.serviceClient<yumi_hw::YumiGrasp>("/yumi/yumi_gripper/release_grasp");
	ros::Publisher joint_pub;
  	ros::Subscriber sub_dbox;
  	joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/yumi/joint_trajectory_pos_controller/command", 1);
	yumik::YumIK Yumi_srv_left;
	yumik::YumIK Yumi_srv_right;
	yumi_dialbox reconfigure;
	dynamic_reconfigure::Server<yumi_motion::ikConfig> server;
	dynamic_reconfigure::Server<yumi_motion::ikConfig>::CallbackType f;
	f=boost::bind(&cfgcallback,_1,_2, &reconfigure);
	server.setCallback(f);
  	sub_dbox = nh.subscribe("dialboxTopic", 1, &yumi_dialbox::DialboxCallback, &reconfigure);
	ros::Rate loop_rate(1000);	
	std::vector< yumik::YumIK  > joint_results_left;
	std::vector< yumik::YumIK > joint_results_right;
	std::vector<double> joint_configuration_left;
	std::vector<double> joint_configuration_right;

	 while (ros::ok()){


		//wTo
		mt::Transform wTor;
		mt::Transform wTol;
			//left
	     	wTol.setRotation(mt::Matrix3x3(1.0, 0.0, 0.0,
		                              0.0, 1.0, 0.0,
		                              0.0, 0.0, 1.0));
		wTol.setTranslation(mt::Point3(reconfigure.x_l, reconfigure.y_l, reconfigure.l_l/2));
			//right
		wTor.setRotation(mt::Matrix3x3(1.0, 0.0, 0.0,
		                              0.0, 1.0, 0.0,
		                              0.0, 0.0, 1.0));
		wTor.setTranslation(mt::Point3(reconfigure.x_r, reconfigure.y_r, reconfigure.l_r/2));


		//oTtcp
		mt::Transform oTtcpl;
		mt::Transform oTtcpr;
			//left
	     	oTtcpl.setRotation(mt::Matrix3x3(0.0, -1.0, 0.0,
		                                -1.0, 0.0, 0.0,
		                              	0.0, 0.0, -1.0));
		oTtcpl.setTranslation(mt::Point3(0, 0, reconfigure.z_tcp_l));
			//right
	     	oTtcpr.setRotation(mt::Matrix3x3(0.0, -1.0, 0.0,
		                                -1.0, 0.0, 0.0,
		                              	0.0, 0.0, -1.0));
		oTtcpr.setTranslation(mt::Point3(0, 0, reconfigure.z_tcp_r));
	
		//rTtcp
		mt::Transform yTtcpl;
		mt::Transform yTtcpr;
			//left
		yTtcpl=wTol*oTtcpl;//*identity
		mt::Rotation R_l=yTtcpl.getRotation();
	    	mt::Matrix3x3 M_l=R_l.getMatrix();
	    	mt::Point3 d_l =yTtcpl.getTranslation();

			//right
		yTtcpr=wTor*oTtcpr;//*identity
		mt::Rotation R_r=yTtcpr.getRotation();
	    	mt::Matrix3x3 M_r=R_r.getMatrix();
	    	mt::Point3 d_r =yTtcpr.getTranslation();
		/*std::cout<<"\nyTr:" <<std::endl;
	     	printTransform(yTr);
		std::cout<<"\nwTo:" <<std::endl;
	     	printTransform(wTo);
		std::cout<<"\noTtcp:" <<std::endl;
	     	printTransform(oTtcp);*/
		//std::cout<<"\nrTtcp:" <<std::endl;
	     	//printTransform(rTtcp);

		Yumi_srv_left.request.theta3=0.0;
		Yumi_srv_left.request.pose.position.x=d_l[0];
		Yumi_srv_left.request.pose.position.y=d_l[1];
		Yumi_srv_left.request.pose.position.z=d_l[2];
		Yumi_srv_left.request.pose.orientation.x=R_l[0];
		Yumi_srv_left.request.pose.orientation.y=R_l[1];
		Yumi_srv_left.request.pose.orientation.z=R_l[2];
		Yumi_srv_left.request.pose.orientation.w=R_l[3];
		Yumi_srv_right.request.theta3=0.0;
		Yumi_srv_right.request.pose.position.x=d_r[0];
		Yumi_srv_right.request.pose.position.y=d_r[1];
		Yumi_srv_right.request.pose.position.z=d_r[2];
		Yumi_srv_right.request.pose.orientation.x=R_r[0];
		Yumi_srv_right.request.pose.orientation.y=R_r[1];
		Yumi_srv_right.request.pose.orientation.z=R_r[2];
		Yumi_srv_right.request.pose.orientation.w=R_r[3];
		ROS_INFO("position.x_l %f",d_l[0]);
		ROS_INFO("position.y_l %f",d_l[1]);
		ROS_INFO("position.z_l %f",d_l[2]);
           //OS_INFO("orientation.x_l %f",R_l[0]);
             //ROS_INFO(" %f",d_l[1]);
             //ROS_INFO("position.z_l %f",d_l[2]);
	

		/*Yumi_srv_right.request.theta3 = 0.0;
	    Yumi_srv_right.request.pose.position.x = 0.5;
	    Yumi_srv_right.request.pose.position.y =-0.5;
	    Yumi_srv_right.request.pose.position.z = 0.4;

	    Yumi_srv_right.request.pose.orientation.x = 0.0;
	    Yumi_srv_right.request.pose.orientation.y = 0.0;
	    Yumi_srv_right.request.pose.orientation.z = 0.0;
	    Yumi_srv_right.request.pose.orientation.w = 1.0;

	    Yumi_srv_left.request.theta3 = 0.0;
	    Yumi_srv_left.request.pose.position.x = 0.5;
	    Yumi_srv_left.request.pose.position.y = 0.5;
	    Yumi_srv_left.request.pose.position.z = 0.6;

	    Yumi_srv_left.request.pose.orientation.x = 0.0;
	    Yumi_srv_left.request.pose.orientation.y = 0.0;
	    Yumi_srv_left.request.pose.orientation.z = 0.0;
   	    Yumi_srv_left.request.pose.orientation.w = 1.0;*/
;
		//ROS_INFO("start %d",reconfigure.START);
		if ((reconfigure.START)||(reconfigure.is_dialbox)){
		ROS_INFO("HE ENTRAT!");

			if((Yumi_client_left.call(Yumi_srv_left))&&(reconfigure.left)){

				 joint_configuration_left.resize(Yumi_srv_left.response.ik_solution.size());

						ROS_INFO("hola3");
				ROS_INFO("c_n_solution %d",reconfigure.n_solution);

				joint_configuration_left=Yumi_srv_left.response.ik_solution;	
				ROS_INFO("joint_configuration_left %f",joint_configuration_left[0]);
				
				reconfigure.joint_states.points[0].positions[0]=joint_configuration_left[0]; //1_l
				reconfigure.joint_states.points[0].positions[1]=joint_configuration_left[1]; //2_l
				reconfigure.joint_states.points[0].positions[2]=joint_configuration_left[3]; //3_l
				reconfigure.joint_states.points[0].positions[3]=joint_configuration_left[4]; //4_l
				reconfigure.joint_states.points[0].positions[4]=joint_configuration_left[5]; //5_l
				reconfigure.joint_states.points[0].positions[5]=joint_configuration_left[6]; //6_l
				reconfigure.joint_states.points[0].positions[6]=joint_configuration_left[2]; //7_l

			}	
			if((Yumi_client_right.call(Yumi_srv_right))&&(reconfigure.right)){

				joint_configuration_right.resize(Yumi_srv_right.response.ik_solution.size());

						ROS_INFO("hola5");
				ROS_INFO("c_n_solution %d",reconfigure.n_solution);
				joint_configuration_right=Yumi_srv_right.response.ik_solution;	
				
						ROS_INFO("hola4");
						ROS_INFO("c_n_solution %d",reconfigure.n_solution);
				
			
				ROS_INFO("joint_configuration_left %f",joint_configuration_right[0]);
				
				reconfigure.joint_states.points[0].positions[7]=joint_configuration_right[0]; //1_r
				reconfigure.joint_states.points[0].positions[8]=joint_configuration_right[1]; //2_r
				reconfigure.joint_states.points[0].positions[9]=joint_configuration_right[3]; //3_r
				reconfigure.joint_states.points[0].positions[10]=joint_configuration_right[4]; //4_r
				reconfigure.joint_states.points[0].positions[11]=joint_configuration_right[5]; //5_r
				reconfigure.joint_states.points[0].positions[12]=joint_configuration_right[6]; //6_r
				reconfigure.joint_states.points[0].positions[13]=joint_configuration_right[2]; //7_r
			
			}

			joint_pub.publish(reconfigure.joint_states);	
			if(reconfigure.gripper_open_left.request.gripper_id==1){
				gripper_client_open.call(reconfigure.gripper_open_left);	
			}
			if(reconfigure.gripper_open_right.request.gripper_id==2){
				gripper_client_open.call(reconfigure.gripper_open_right);	
			}
			if(reconfigure.gripper_close_left.request.gripper_id=1){ 
				gripper_client_close.call(reconfigure.gripper_close_left);
			}
			if(reconfigure.gripper_close_right.request.gripper_id=2){
				gripper_client_close.call(reconfigure.gripper_close_right);	
				
			}
				

		}
		else{
					
			if(reconfigure.gripper_open_left.request.gripper_id==1){
				gripper_client_open.call(reconfigure.gripper_open_left);	
			}
			else if (reconfigure.gripper_close_left.request.gripper_id=1){ 

				gripper_client_close.call(reconfigure.gripper_close_left);
			}
			if(reconfigure.gripper_open_right.request.gripper_id==2){
				gripper_client_open.call(reconfigure.gripper_open_right);	
			}
			else if(reconfigure.gripper_close_right.request.gripper_id=2){
				gripper_client_close.call(reconfigure.gripper_close_right);	
				
			}
		
		}
		ros::spinOnce();

	}
	  return 0;


}


