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
#define CLOCK 10


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
    std::vector<double> FindCoeffVector(double init_pos, double final_pos, double movement_dur);
    double Interpolate(std::vector<double> coeff_vector, double cur_time);

    void MoveJoint();
    void PathGenerate();

    sensor_msgs::JointState cur_joint; //current joint state
    bool ReachGoalFlag; //true:Reached goal, false: Not reached goal
    double ticks;

    double movement_dur = 100;
    std::vector<double> final_angle_vector;
    std::vector<std::vector<double>> theta_coeff_matrix;

};

//Constructor
PA10Controller::PA10Controller()
{
    //initialize
    ReachGoalFlag = false;
    //define ROS node
    joint_pub = node.advertise<sensor_msgs::JointState>("/pa10/joint_states", CLOCK);
    final_angle_vector = std::vector<double>({10, 20, 0, 30, 40, 50, 60});

    std::cout << "\n";

    for (int j = 0; j < 7; ++j) {
        final_angle_vector[j] = (final_angle_vector[j] * (pi / 180.0));

        std::cout << final_angle_vector[j];
        std::cout << "; ";
    }

    std::cout << "\n";

    theta_coeff_matrix = std::vector<std::vector<double>>();
    theta_coeff_matrix.resize(7);

    for ( int i = 0 ; i < 7 ; i++ ) {
        theta_coeff_matrix[i].resize(5);
        theta_coeff_matrix[i] = FindCoeffVector(0, final_angle_vector[i], movement_dur);
    }
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

}
*/


//find coefficient values for each interpolation function
std::vector<double> PA10Controller::FindCoeffVector(double init_pos, double final_pos, double movement_dur)
{
    std::vector<double> coeff_vector(6);

    coeff_vector[0] = init_pos;
    coeff_vector[1] = 0;
    coeff_vector[2] = 0 / 2;
    coeff_vector[3] = ((20 * (final_pos - init_pos)) -
                        (movement_dur * (8 * 0 + 12 * 0)) -
                        (pow(movement_dur, 2) * (3 * 0 - 0))) / (2 * pow(movement_dur, 3));
    coeff_vector[4] = ((30 * (init_pos - final_pos)) +
                        (movement_dur * (14 * 0 + 16 * 0)) +
                        (pow(movement_dur, 2) * (3 * 0 - 2 * 0))) / (2 * pow(movement_dur, 4));
    coeff_vector[5] = ((12 * (final_pos - init_pos)) -
                        (6 * movement_dur * (0 + 0)) -
                        (pow(movement_dur, 2) * (0 - 0))) / (2 * pow(movement_dur, 5));

    for (int i = 0; i < 5; ++i) {
        std::cout << coeff_vector[i];
        std::cout << "\n";

    }

    return coeff_vector;
}

double PA10Controller::Interpolate(std::vector<double> coeff_vector, double cur_time)
{
    double inter_val;


    //find 5th order polynomial value for theta(t)
    inter_val = coeff_vector[0] + (coeff_vector[1] * cur_time) + (coeff_vector[2] * pow(cur_time, 2)) +
                (coeff_vector[3] * pow(cur_time, 3)) + (coeff_vector[4] * pow(cur_time, 4)) +
                (coeff_vector[5] * pow(cur_time, 5));

    return inter_val;
}

void PA10Controller::MoveJoint()
{

    for(int i=0; i<7; ++i)
    {
        cur_joint.position[i] = Interpolate(theta_coeff_matrix[i], ticks);

        std::cout << (cur_joint.position[i] * (180 / pi));
        std::cout << "\n";
    }

    //publish joint states
    joint_pub.publish(cur_joint);
}


void PA10Controller::StartMoving()
{
    ros::Rate loop_rate(10); //set loop rate 10[Hz]
    ROS_INFO("Start Moving");
    ticks=0.0;

    //initialize joint names
    initJointState(&cur_joint);

    while(ros::ok() && !ReachGoalFlag)
    {
        MoveJoint();
        ros::spinOnce();
        loop_rate.sleep();//sleep 1 loop rate(0.1sec)
        if(ticks >= (movement_dur)) ReachGoalFlag=true;
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
