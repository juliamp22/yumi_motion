## Run dialbox with yumi (ROS_CONTROL)
Author: Júlia Marsal Perendreu

This package contains the information for running yumi (real or simulated) with dialbox, having the option to choose between right arm, left arm or both arms. 
You need dialbox and yumi packages.

You can found it in (git clone https://github.com/OrebroUniversity/yumi)
		    (git clone ssh://ris/git/dialbox.git)


1- Choose between GAZEBO or real robot:

# Option1.1: Run with GAZEBO

1. roslaunch yumi_launch yumi_gazebo_pos.launch (to enable the connection between gazebo and ROS)

# Option1.2: Run with real robot
C
1. roslaunch yumi_launch yumi_pos_control.launch (to enable the connection between ROS and the robot)


2-You've got two options:
		
		1- Running a node moving yumi with dialbox, leaving from joint_states_position
		2- Running a node moving yumi with dialbox, leaving from a fixed position


# Option2.1: Running yumi from joint_states_position

2. roslaunch yumi_motion real_dial_joint_states.launch

# Option2.2: Running yumi from a fixed position

2. roslaunch yumi_motion real_dial.launch



## HOW TO LAUNCH INVERSE KINEMATIKS WITH DIALBOX || RQT_RECONFIGURE --> CAUTION InverseKinematiks is uploading, don't prove wit the REAL ROBOT YET!!

1- Choose between GAZEBO or real robot:

# Option1.1: Run with GAZEBO

1. roslaunch yumi_launch yumi_gazebo_pos.launch (to enable the connection between gazebo and ROS)

# Option1.2: Run with real robot

1. roslaunch yumi_launch yumi_pos_control.launch (to enable the connection between ROS and the robot)

2-You've got two options:

	1-Running a node moving yumi with dialbox, ch1:X, ch3:Y, ch5:Z, ch7:Z_tcp, ch0:choose which of the 16 configurations do you want to prove
	1-Running with rqt_reconfigure.

#Option 2.1 Run yumi with dialbox


2. roslaunch yumi_motion ik_dialbox.launch 
Make sure that dialbox has been established correctly (try making roslaunch dialbox dialbox.launch)

#Option 2.2 Run yumi with rqt_reconfigure


2. roslaunch yumi_motion ik_reconfigure.launch
	


IMPORTANT NOTES:

#include
- Also in include/yumi_motion folder, there is a library which makes easier yumi's job. (yumi_dial_real.h)
- Also there are another library (yumi_dial_real_2.h) which is used to get the joint_states position (the actual position of the robot).

#src
- yumi_sinus.cpp: Is a node made to move arms as a sinus. There is an option to start from the actual position of the robot or, from an random initial position predefined


- yumi_dial.cpp: It is the node created to move the robot as dialbox sais. 
  Joints are going to be defined by dialbox with the following distribution: 
				
				--------------------------	
				|  - 1  |-------| - 0    |
   				|  - 3  |       | - 2    |
   				|  - 5  |       | - 4    |
   				|  - 7  |-------| - 6    |
				--------------------------


- obtainqik.cpp: 

- yumi_dial_real.cpp:



- yumi_dial_real_joint_states.cpp:




