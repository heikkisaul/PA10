/* Written by Songyot Piriyakulkit
   For Hirata Lab. PA10 Training
   November 2017*/
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
void initJointState(sensor_msgs::JointState *joint_state)
{
	joint_state->name.resize(9);
	joint_state->position.resize(9);
	joint_state->name[0] = "joint_1";
	joint_state->name[1] = "joint_2";
	joint_state->name[2] = "joint_3";
	joint_state->name[3] = "joint_4";
	joint_state->name[4] = "joint_5";
	joint_state->name[5] = "joint_6";
	joint_state->name[6] = "joint_7";
	joint_state->name[7] = "gripper_finger";
	joint_state->name[8] = "gripper_finger_mimic_joint";
	return;
}
