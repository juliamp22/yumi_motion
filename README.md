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

1. roslaunch yumi_launch yumi_pos_control.launch (to enable the connection between ROS and the robot)


2-You've got two options:
		
		1- Running a node moving yumi with dialbox, leaving from joint_states_position
		2- Running a node moving yumi with dialbox, leaving from a fixed position


# Option2.1: Running yumi from joint_states_position

2. roslaunch yumi_motion real_dial_joint_states.launch

# Option2.2: Running yumi from a fixed position

2. roslaunch yumi_motion real_dial.launch







# yumi_motion
