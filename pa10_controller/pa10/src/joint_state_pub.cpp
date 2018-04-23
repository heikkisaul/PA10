/*
  Manipulator Training 2018 Spring
  Sample Program
  written by Rikuto SATO(2018/4)
*/


#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>

#include "pa10/pa10_params.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"

#define pi 3.1415926

class PA10Controller
{
public:
  void StartMoving();
  PA10Controller(); //constructor

private:
  ros::NodeHandle node; //node handler
  ros::Publisher joint_pub; //define publisher
  void initJointState(sensor_msgs::JointState *joint_state); // see init folder

  /*Example definition of kinematics compute functions*/
  std::vector<double> ForwardKinematics(sensor_msgs::JointState *joint); //forward kinematics
  sensor_msgs::JointState InverseKinematics(std::vector<double>& position); //inverse kinematics

  void MoveJoint();
  void PathGenerate();

  sensor_msgs::JointState cur_joint; //current joint state
  bool ReachGoalFlag; //true:Reached goal, false: Not reached goal
  double ticks;
};


//Constructor
  PA10Controller::PA10Controller()
  {
    //initialize
    ReachGoalFlag = false;
    //define ROS node
    joint_pub = node.advertise<sensor_msgs::JointState>("/pa10/joint_states",10);
  }


void PA10Controller::initJointState(sensor_msgs::JointState *joint_state)
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

  //Initialize Position (All angles are 0.0[rad])
  for(int i=0; i<7; ++i)joint_state->position[i] = 0.0;

  return;
}

/*example template
std::vector<double> PA10Controller::ForwardKinematics(sensor_msgs::JointState *joint)
{

//write by yourself

}


sensor_msgs::joint_state PA10Controller::InverseKinematics(std::vector<double>& position)
{
  //write by yourself
  // note: there are multiple answers regarding to inverse kinematics.
}


void PA10Controller::PathGenerate()
{
//write by yourself, by using 5-order interpolation
}
*/

void PA10Controller::MoveJoint()
{

  for(int i=0; i<7; ++i)
    {
      cur_joint.position[i]=pi/6*i*ticks/30.0;
    }

  //publish joint states
  joint_pub.publish(cur_joint);
}


void PA10Controller::StartMoving()
{
  ros::Rate loop_rate(10); //set loop rate 10[Hz]
  ROS_INFO("Start Moving");
  ticks=0.0;

  //initiallize joint names
  initJointState(&cur_joint);

  while(ros::ok() && ReachGoalFlag == false)
    {
      MoveJoint();
      ros::spinOnce();
      loop_rate.sleep();//sleep 1 loop rate(0.1sec)
      if(ticks>31) ReachGoalFlag=true;
      ticks++;
    }

  ROS_INFO("Finished");
}


//main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_pub");

  PA10Controller pa10controller;

  pa10controller.StartMoving();

  return 0;
}
