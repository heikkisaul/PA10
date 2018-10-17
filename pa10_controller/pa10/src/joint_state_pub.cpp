/*
  Manipulator Training 2018 Spring
  Sample Program
  written by Rikuto SATO(2018/4)
*/


#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <cmath>

#include "pa10/pa10_params.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"
#include "Matrix.hpp"
//#include "PA10Kinematics.hpp"

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

    std::vector<double> InverseKinematics(std::vector<double> end_effector_vector); //inverse kinematics
    double bind_angle(double angle);

    std::vector<double> FindCoeffVector(double init_pos, double final_pos, double movement_dur);
    double Interpolate(std::vector<double> coeff_vector, double cur_time);
    //std::vector<std::vector<double>> LinePathGenerator(std::vector<double> start_cartesian_vector, std::vector<double> final_cartesian_vector,
    //        double movement_dur);

    std::vector<double> LinePathGenerator(std::vector<double> start_cartesian_vector, std::vector<double> final_cartesian_vector, double movement_dur, double ticks);
    //std::vector<std::vector<double>> CreateCoeffMatrix(std::vector<double> initial_angles_vector, std::vector<double> final_angles_vector, double ticks);
    double findCur5thOrderVal(double ticks, double duration_ticks, double x_0, double x_f);

    void MoveJoint(std::vector<double> theta_coeff_matrix);
    void PathGenerate();

    sensor_msgs::JointState cur_joint; //current joint state
    bool ReachGoalFlag; //true:Reached goal, false: Not reached goal
    double ticks;

    double movement_dur = 100;

    std::vector<double> final_angle_vector;
    //std::vector<std::vector<double>> theta_coeff_matrix;

    std::vector<double> initial_cartesian_vector;
    std::vector<double> start_cartesian_vector;
    std::vector<double> final_cartesian_vector;

    //PA10Kinematics converter;
    std::vector<std::vector<double>> final_angles_matrix;


};

//Constructor
PA10Controller::PA10Controller()
{
    //initialize
    ReachGoalFlag = false;
    //define ROS node
    joint_pub = node.advertise<sensor_msgs::JointState>("/pa10/joint_states", CLOCK);
    final_angle_vector = std::vector<double>({10, 20, 0, 30, 40, 50, 60});

    initial_cartesian_vector = std::vector<double>({0, 0, 1317, 0, 0, 0});

    //start_cartesian_vector = std::vector<double>({-400, 300, 500, (-M_PI / 2), (-M_PI / 2), 0});
    //final_cartesian_vector = std::vector<double>({300, 600, 500, (-M_PI / 2), (-M_PI / 2), 0});

    start_cartesian_vector = {600, -210, 700, -M_PI/2, -M_PI/2, 0};
    final_cartesian_vector = {450, 400, 720, -M_PI/2, -M_PI/2, 0};

    //start_cartesian_vector = {0, 0, 1317, 0, 0, 0};
    //final_cartesian_vector = {0, 400, 1000, 0, 0, 0};

    //converter = new PA10Kinematics();

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


std::vector<double> PA10Controller::InverseKinematics(std::vector<double> end_effector_vector){

    Matrix mat_a01, mat_a12, mat_a23, mat_a34, mat_a45, mat_a56;
    Matrix mat_phi, mat_theta, mat_psi;

    mat_phi = Matrix(3, 3);
    mat_theta = Matrix(3, 3);
    mat_psi = Matrix(3, 3);

    //fill constant values into joint matrices
    mat_a01.set_value(2, 1, -1);
    mat_a01.set_value(2, 3, 317);
    mat_a01.set_value(3, 3, 1);

    mat_a12.set_value(2, 2, 1);
    mat_a12.set_value(3, 3, 1);

    mat_a23.set_value(2, 1, 1);
    mat_a23.set_value(3, 3, 1);


    mat_a34.set_value(2, 1, -1);
    mat_a34.set_value(2, 3, 480);
    mat_a34.set_value(3, 3, 1);

    mat_a45.set_value(2, 1, 1);
    mat_a45.set_value(3, 3, 1);

    mat_a56.set_value(2, 2, 1);
    mat_a56.set_value(2, 3, 70);
    mat_a56.set_value(3, 3, 1);

    //fill constant values into orientation matrices
    mat_phi.set_value(2, 2, 1);
    mat_theta.set_value(0, 0, 1);
    mat_psi.set_value(2, 2, 1);

    Matrix mat_end_eff_ori(3, 3), mat_inter(3, 3), mat_inter_inv(3, 3), T_prim(3, 3);
    std::vector<double> result_vector(6), prev_val_vector(6);

    //fill variable values to phi, theta and psi matrices
    mat_phi.set_value(0, 0, cos(end_effector_vector[3]));
    mat_phi.set_value(0, 1, -sin(end_effector_vector[3]));
    mat_phi.set_value(1, 0, sin(end_effector_vector[3]));
    mat_phi.set_value(1, 1, cos(end_effector_vector[3]));

    mat_theta.set_value(1, 1, cos(end_effector_vector[4]));
    mat_theta.set_value(1, 2, -sin(end_effector_vector[4]));
    mat_theta.set_value(2, 1, sin(end_effector_vector[4]));
    mat_theta.set_value(2, 2, cos(end_effector_vector[4]));

    mat_psi.set_value(0, 0, cos(end_effector_vector[5]));
    mat_psi.set_value(0, 1, -sin(end_effector_vector[5]));
    mat_psi.set_value(1, 0, sin(end_effector_vector[5]));
    mat_psi.set_value(1, 1, cos(end_effector_vector[5]));


    //calculate summary orientation matrix
    mat_end_eff_ori = mat_phi * mat_theta * mat_psi;


    //define intermediate variables
    double a2, d1, d4, d6;
    double pxi, pyi, pzi, bxp, byp, bzp, nxp, nyp, txp, typ, inter1, inter21, inter22, inter3, theta11, theta12, theta2,
            theta3, theta4, theta5, theta6;

    a2 = 450.0;
    d1 = 317.0;
    d4 = 480.0;
    d6 = 70.0;

    std::cout.precision(3);

    pxi = end_effector_vector[0] - d6 * mat_end_eff_ori.get_value(0, 2);
    pyi = end_effector_vector[1] - d6 * mat_end_eff_ori.get_value(1, 2);
    pzi = end_effector_vector[2] - d6 * mat_end_eff_ori.get_value(2, 2);

    inter1 = pow(pxi, 2) + pow(pyi, 2);

    //make sure inter1 is positive and calculate theta1
    theta1 = bind_angle(2 * atan2(((inter1 > 0) ? -pxi + sqrt(inter1) : -pxi - sqrt(inter1)), pyi));
    //theta1 = 2 * atan2(((inter1 > 0) ? -pxi + sqrt(inter1) : -pxi - sqrt(inter1)), pyi);

    inter3 = (pxi * cos(theta1)) + (pyi * sin(theta1));

    // calculate theta3 from acos, change to atan if necessary
    //theta3 = acos((pow(inter3, 2) + pow((-pzi + d1), 2) - pow(a2, 2) - pow(d4, 2)) / (2 * a2 * d4));

    theta3 = atan2(sqrt(1 - pow(((pow(inter3, 2) + pow((-pzi + d1), 2) - pow(a2, 2) - pow(d4, 2)) / (2 * a2 * d4)), 2)), ((pow(inter3, 2) + pow((-pzi + d1), 2) - pow(a2, 2) - pow(d4, 2)) / (2 * a2 * d4)));

    inter21 = (pow(d4 * sin(theta3), 2) + pow(a2 + d4 * cos(theta3), 2) - pow(d1 - pzi, 2));
    inter22 = a2 - d1 + pzi + (d4 * cos(theta3));

    //calculate theta2
    //theta2 = -2 * atan2((((pxi > 0) ? (d4 * sin(theta3)) + sqrt(inter21) : (d4 * sin(theta3)) - sqrt(inter21))), inter22);
    theta2 = -2 * atan2((((d4 * sin(theta3)) - sqrt(inter21))), inter22);

    //create intermediate matrix for theta4,5,6 calculation
    mat_a01.set_value(0, 0, cos(theta1));
    mat_a01.set_value(0, 2, -sin(theta1));
    mat_a01.set_value(1, 0, sin(theta1));
    mat_a01.set_value(1, 2, cos(theta1));

    mat_a12.set_value(0, 0, sin(theta2));
    mat_a12.set_value(0, 1, cos(theta2));
    mat_a12.set_value(0, 3, 450 * sin(theta2));
    mat_a12.set_value(1, 0, -cos(theta2));
    mat_a12.set_value(1, 1, sin(theta2));
    mat_a12.set_value(1, 3, -450 * cos(theta2));

    mat_a23.set_value(0, 0, -sin(theta3));
    mat_a23.set_value(0, 2, cos(theta3));
    mat_a23.set_value(1, 0, cos(theta3));
    mat_a23.set_value(1, 2, sin(theta3));

    mat_inter = mat_a01 * mat_a12 * mat_a23;

    //Find inverse of intermediate matrix. Because the matrix is orthogonal inverse == transpose
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            mat_inter_inv.set_value(i, j, mat_inter.get_value(j, i));
        }
    }

    //find matrix T' (p. 45 of training materials)
    T_prim = mat_inter_inv * mat_end_eff_ori;

    nxp = T_prim.get_value(0, 0);
    nyp = T_prim.get_value(1, 0);

    txp = T_prim.get_value(0, 1);
    typ = T_prim.get_value(1, 1);

    bxp = T_prim.get_value(0, 2);
    byp = T_prim.get_value(1, 2);
    bzp = T_prim.get_value(2, 2);


    // calculate theta4, theta5, theta6
    theta4 = atan2(byp, bxp);

    theta5 = atan2((bxp * cos(theta4) + byp * sin(theta4)), bzp);

    theta6 = atan2((-nxp * sin(theta4) + nyp * cos(theta4)), (-txp * sin(theta4) + typ * cos(theta4)));

/*
    result_vector[0] = theta1 * 180.0 / M_PI;
    result_vector[1] = theta2 * 180.0 / M_PI;
    result_vector[2] = theta3 * 180.0 / M_PI;
    result_vector[3] = theta4 * 180.0 / M_PI;
    result_vector[4] = theta5 * 180.0 / M_PI;
    result_vector[5] = theta6 * 180.0 / M_PI;
*/

    result_vector[0] = theta1;
    result_vector[1] = theta2;
    result_vector[2] = theta3;
    result_vector[3] = theta4;
    result_vector[4] = theta5;
    result_vector[5] = theta6;

    prev_val_vector = result_vector;

    return result_vector;

}


double PA10Controller::bind_angle(double angle)
{
    if(angle < 0) {
        return std::abs(angle) < angle + (2*M_PI) ? angle : angle + (2*M_PI);
    }
    return angle < std::abs(angle - (2*M_PI)) ? angle : angle - (2*M_PI);
}

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

    return coeff_vector;
}

double PA10Controller::findCur5thOrderVal(double ticks, double duration_ticks, double x_0, double x_f){

    std::vector<double> coeff = FindCoeffVector(x_0, x_f, duration_ticks);
    return coeff[0]+coeff[1]*ticks+coeff[2]*ticks*ticks+coeff[3]*ticks*ticks*ticks+coeff[4]*ticks*ticks*ticks*ticks+coeff[5]*ticks*ticks*ticks*ticks*ticks;
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

std::vector<double> PA10Controller::LinePathGenerator(std::vector<double> start_cartesian_vector, std::vector<double> final_cartesian_vector, double movement_dur, double ticks)
{
    std::vector<double> via_point_vector(6);
    std::vector<double> via_angles_vector(7);
    std::vector<double> theta_vector(6);
    double val5thorder = findCur5thOrderVal(ticks, movement_dur, 0, 1);
    /*via_point_vector[0] = start_cartesian_vector[0] + ((final_cartesian_vector[0] - start_cartesian_vector[0]) * (ticks / (movement_dur)));
    via_point_vector[1] = start_cartesian_vector[1] + ((final_cartesian_vector[1] - start_cartesian_vector[1]) * (ticks / (movement_dur)));
    via_point_vector[2] = start_cartesian_vector[2] + ((final_cartesian_vector[2] - start_cartesian_vector[2]) * (ticks / (movement_dur)));*/
    via_point_vector[0] = start_cartesian_vector[0] + ((final_cartesian_vector[0] - start_cartesian_vector[0]) * val5thorder);
    via_point_vector[1] = start_cartesian_vector[1] + ((final_cartesian_vector[1] - start_cartesian_vector[1]) * val5thorder);
    via_point_vector[2] = start_cartesian_vector[2] + ((final_cartesian_vector[2] - start_cartesian_vector[2]) * val5thorder);

    via_point_vector[3] = (-M_PI / 2);
    via_point_vector[4] = (-M_PI / 2);
    via_point_vector[5] = 0;

    //theta_vector = converter.inverse_kinematics(via_point_vector);
    theta_vector = InverseKinematics(via_point_vector);

    via_angles_vector[0] = theta_vector[0];
    via_angles_vector[1] = theta_vector[1];
    via_angles_vector[2] = 0;
    via_angles_vector[3] = theta_vector[2];
    via_angles_vector[4] = theta_vector[3];
    via_angles_vector[5] = theta_vector[4];
    via_angles_vector[6] = theta_vector[5];

    for (int i = 0; i < 6; ++i) {
        std::cout << theta_vector[i];
        //std::cout << via_point_vector[i];
        std::cout << "; ";

    }
    std::cout << "\n";

    return via_angles_vector;
}


/*std::vector<std::vector<double>> PA10Controller::LinePathGenerator(std::vector<double> start_cartesian_vector,
                                       std::vector<double> final_cartesian_vector,
                                       double movement_dur)
{
    //calculate parametric formula and create via-point matrix
    std::vector<std::vector<double>> via_point_matrix((movement_dur));
    std::vector<std::vector<double>> via_angles_matrix((movement_dur));

    for ( int i = 0 ; i < movement_dur; i++ ) {
        via_point_matrix[i].resize(6);

        via_point_matrix[i][0] = start_cartesian_vector[0] + ((final_cartesian_vector[0] - start_cartesian_vector[0]) * (i / (movement_dur - 1)));
        via_point_matrix[i][1] = start_cartesian_vector[1] + ((final_cartesian_vector[1] - start_cartesian_vector[1]) * (i / (movement_dur - 1)));
        via_point_matrix[i][2] = start_cartesian_vector[2] + ((final_cartesian_vector[2] - start_cartesian_vector[2]) * (i / (movement_dur - 1)));

        via_point_matrix[i][3] = (-M_PI / 2);
        via_point_matrix[i][4] = (-M_PI / 2);
        via_point_matrix[i][5] = 0;


    }

    std::vector<double> via_point_temp(7);

    via_angles_matrix.resize(movement_dur);

    std::vector<double> theta_vector(6);
    for (int k = 0; k < 6; k++) theta_vector[k] = 0;

    for ( int i = 0 ; i < movement_dur ; i++ ) {
        via_angles_matrix[i].resize(7);

        via_point_temp = via_point_matrix[i];


        theta_vector = converter.inverse_kinematics(via_point_matrix[i]);
        via_angles_matrix[i][0] = theta_vector[0];
        via_angles_matrix[i][1] = theta_vector[1];
        via_angles_matrix[i][2] = 0;
        via_angles_matrix[i][3] = theta_vector[2];
        via_angles_matrix[i][4] = theta_vector[3];
        via_angles_matrix[i][5] = theta_vector[4];
        via_angles_matrix[i][6] = theta_vector[5];

        //via_angles_matrix[i][4] = 0;
        //via_angles_matrix[i][5] = 0;
        //via_angles_matrix[i][6] = 0;

        //std::cout << via_angles_matrix[i][6];
        //std::cout << "; ";

    }

    for (int j = 0; j < movement_dur ; j++) {
        for (int i = 0; i < 6; i++) {

            std::cout << via_point_matrix[j][i];
            std::cout << "; ";

        }

        std::cout << "\n";

    }
    for (int j = 0; j < movement_dur ; j++) {
        for (int i = 0; i < 7; i++) {

                std::cout << via_angles_matrix[j][i];
                std::cout << "; ";

        }

        std::cout << "\n";

    }


    //std::cout << "\n";

    return via_angles_matrix;

}*/

/*std::vector<std::vector<double>> PA10Controller::CreateCoeffMatrix(std::vector<double> initial_angles_vector, std::vector<double> final_angles_vector, double ticks)
{
    std::vector<double> temp_coeff_vector(5);
    std::vector<std::vector<double>> result_matrix(7);
    std::vector<double> cur_angles_vector(7);


    for (int i = 0; i < 7 ; ++i) {

        temp_coeff_vector = FindCoeffVector(cur_angles_vector[i],  )

        result_matrix[i] = temp_coeff_vector;

    }
}*/


void PA10Controller::MoveJoint(std::vector<double> theta_coeff_matrix)
{

    for(int i=0; i<7; ++i)
    {
        cur_joint.position[i] = theta_coeff_matrix[i];

    }

    //publish joint states
    joint_pub.publish(cur_joint);
}


void PA10Controller::StartMoving()
{

    std::vector<std::vector<double>> via_angles_matrix;

    ros::Rate loop_rate(10); //set loop rate 10[Hz]

    ROS_INFO("Start Moving");
    ticks=0.0;


    //initialize joint names
    initJointState(&cur_joint);

    while(ros::ok() && !ReachGoalFlag)
    {
        std::vector<double> d  = LinePathGenerator(start_cartesian_vector, final_cartesian_vector, movement_dur, ticks);
        MoveJoint(d);
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
