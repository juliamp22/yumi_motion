#include <mt/mt.h>
#include <mt/scalar.h>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>
#include <yumik/YumIK.h>
#include <dynamic_reconfigure/server.h>
#include <yumi_motion/ikConfig.h>
#include <yumi_motion/yumi_dial_real.h>
#include <sensor_msgs/JointState.h>

//NaN:the position is not reachable.
//Respect the shoulder, it is reachable (r=516,theta, gamma ) --> espherical coord.
//It will only work with RIGHT ARM!!!!
void cfgcallback(yumi_motion::ikConfig &config, uint32_t level,yumi_dialbox *reconfigure){
	reconfigure->set_values_ik(config.X, config.Y,config.L,config.Z_tcp,config.START,config.n_solution);
	/*ROS_INFO("X %f",config.X);
	ROS_INFO("Y %f",config.Y);
	ROS_INFO("L %f",config.L);
	ROS_INFO("Z_tcp %f",config.Z_tcp);
	ROS_INFO("START %f",config.START);*/
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener_to_dialbox");
  	ros::NodeHandle nh;
	ros::ServiceClient yumik; 
	ros::Publisher joint_pub;
	yumik = nh.serviceClient<yumik::YumIK>("/Yumi/IKSolver");
  	joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/yumi/joint_trajectory_pos_controller/command", 1);
	yumik::YumIK msg;
	yumi_dialbox reconfigure;
	dynamic_reconfigure::Server<yumi_motion::ikConfig> server;
	dynamic_reconfigure::Server<yumi_motion::ikConfig>::CallbackType f;
	f=boost::bind(&cfgcallback,_1,_2, &reconfigure);
	server.setCallback(f);
	ros::Rate loop_rate(1000);	
	std::vector< yumik::ik > joint_results;
	std::vector<double> joint_configuration;

	 while (ros::ok()){
		float x=reconfigure.x;
		float y=reconfigure.y;
		float L=reconfigure.l;
		float z_tcp=reconfigure.z_tcp;
		int n_solution=reconfigure.n_solution;
		
		ROS_INFO("x %f",reconfigure.x);
		ROS_INFO("y %f",reconfigure.y);
		ROS_INFO("l %f",reconfigure.l);
		ROS_INFO("z_tcp %f",reconfigure.z_tcp);
		ROS_INFO("START %d",reconfigure.START);
		ROS_INFO("n_solution %d",reconfigure.n_solution);
		//yTr

		mt::Rotation ypr(-2.3180, -0.5716,-0.9781);
		mt::Point3 d1(0.05355, -0.0725, 0.41429);
	     	mt::Transform yTr(ypr, d1);

		//wTo
		mt::Transform wTo;
	     	wTo.setRotation(mt::Matrix3x3(1.0, 0.0, 0.0,
		                              0.0, 1.0, 0.0,
		                              0.0, 0.0, 1.0));
		wTo.setTranslation(mt::Point3(x, y, L/2));

		//oTtcp
		mt::Transform oTtcp;
	     	oTtcp.setRotation(mt::Matrix3x3(0.0, -1.0, 0.0,
		                                -1.0, 0.0, 0.0,
		                              	0.0, 0.0, -1.0));
		oTtcp.setTranslation(mt::Point3(0, 0, z_tcp));
	
		//rTtcp
		mt::Transform rTtcp=yTr.inverse()*wTo*oTtcp;
		mt::Rotation R=rTtcp.getRotation();
	    	mt::Matrix3x3 M=R.getMatrix();
	    	mt::Point3 d =rTtcp.getTranslation();


		/*std::cout<<"\nyTr:" <<std::endl;
	     	printTransform(yTr);
		std::cout<<"\nwTo:" <<std::endl;
	     	printTransform(wTo);
		std::cout<<"\noTtcp:" <<std::endl;
	     	printTransform(oTtcp);*/
		//std::cout<<"\nrTtcp:" <<std::endl;
	     	//printTransform(rTtcp);

		msg.request.theta3=0.0;
		msg.request.pose.position.x=d[0];
		msg.request.pose.position.y=d[1];
		msg.request.pose.position.z=d[2];
		msg.request.pose.orientation.x=R[0];
		msg.request.pose.orientation.y=R[1];
		msg.request.pose.orientation.z=R[2];
		msg.request.pose.orientation.w=R[3];

	
		if (reconfigure.START){
	
			if(yumik.call(msg)){
				
				//for (int i = 0; i < msg.response.ikSolutions.size(); ++i) { 
				joint_results.resize(msg.response.ikSolutions.size());
				//}
				joint_results=msg.response.ikSolutions;	
				joint_configuration.resize(msg.response.ikSolutions[n_solution].ik.size());
				std::copy(msg.response.ikSolutions[n_solution].ik.begin(),msg.response.ikSolutions[n_solution].ik.end(),joint_configuration.begin());
				ROS_INFO("N_solution %d",n_solution);
				reconfigure.joint_states.points[0].positions[7]=joint_configuration[0]; //1_r
				reconfigure.joint_states.points[0].positions[8]=joint_configuration[1]; //2_r
				reconfigure.joint_states.points[0].positions[9]=joint_configuration[3]; //3_r
				reconfigure.joint_states.points[0].positions[10]=joint_configuration[4]; //4_r
				reconfigure.joint_states.points[0].positions[11]=joint_configuration[5]; //5_r
				reconfigure.joint_states.points[0].positions[12]=joint_configuration[6]; //6_r
				reconfigure.joint_states.points[0].positions[13]=joint_configuration[2]; //7_r
			}	
			joint_pub.publish(reconfigure.joint_states);				


		}
		ros::spinOnce();
	
	}
	  return 0;


}


