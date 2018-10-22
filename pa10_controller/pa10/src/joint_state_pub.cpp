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
    std::vector<std::vector<double>> ForwardKinematics(std::vector<double> angle_vector); //forward kinematics
    std::vector<double> ForwardConverter(std::vector<std::vector<double>> transform_mat);

    std::vector<double> InverseKinematics(std::vector<double> end_effector_vector, std::vector<std::vector<double>> transformation_matrix, std::vector<double> cartesian_vector_0); //inverse kinematics
    double bind_angle(double angle);

    std::vector<double> FindCoeffVector(double init_pos, double final_pos, double movement_dur);
    double Interpolate(std::vector<double> coeff_vector, double cur_time);

    std::vector<double> LinePathGenerator(std::vector<double> cartesian_vector_0, std::vector<double> cartesian_vector_f, double movement_dur, double ticks, std::vector<std::vector<double>> transformation_mat);
    std::vector<double> CirclePathGenerator(std::vector<double> cartesian_vector_0, std::vector<double> cartesian_vector_d, double movement_dur, double ticks, std::vector<std::vector<double>> transformation_mat);
    double findCur5thOrderVal(double ticks, double duration_ticks, double x_0, double x_f);

    void MoveJoint(std::vector<double> theta_coeff_matrix);
    void PathGenerate();

    sensor_msgs::JointState cur_joint; //current joint state
    bool ReachGoalFlag; //true:Reached goal, false: Not reached goal
    double ticks;

    double movement_dur = 50;

    std::vector<double> initial_angle_vector;
    //std::vector<std::vector<double>> theta_coeff_matrix;

    //std::vector<double> initial_cartesian_vector;
    std::vector<double> start_cartesian_vector;
    std::vector<double> delta_cartesian_vector;


};

//Constructor
PA10Controller::PA10Controller() {
    //initialize
    ReachGoalFlag = false;
    //define ROS node
    joint_pub = node.advertise<sensor_msgs::JointState>("/pa10/joint_states", CLOCK);
    initial_angle_vector = std::vector<double>({10, 10, 0, 10, 10, 10, 10});

    delta_cartesian_vector = {100, 200, -800};

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


// Calculate forward kinematics based on intial_angle_vector
std::vector<std::vector<double>> PA10Controller::ForwardKinematics(std::vector<double> angle_vector)
{

    Matrix mat_a01, mat_a12, mat_a23, mat_a34, mat_a45, mat_a56;
    Matrix transform_mat;

    std::vector<double> theta_vector(6);
    std::vector<std::vector<double>> result_vector(4);
    std::vector<double> temp_row_vector(4);

    // Initialize matrices with zeroes to avoid random values
    for (int k = 0; k < 4; ++k) {
        for (int i = 0; i < 4 ; ++i) {
            mat_a01.set_value(k, i, 0.0);
            mat_a12.set_value(k, i, 0.0);
            mat_a23.set_value(k, i, 0.0);
            mat_a34.set_value(k, i, 0.0);
            mat_a45.set_value(k, i, 0.0);
            mat_a56.set_value(k, i, 0.0);
        }
    }

    // Fill constant values into joint matrices
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


    // Convert angles from deg to rad.
    for(int i = 0; i < 7; i++){
        angle_vector[i] = angle_vector[i] * M_PI/180;
    }

    theta_vector[0] = angle_vector[0];
    theta_vector[1] = angle_vector[1];
    theta_vector[2] = angle_vector[3];
    theta_vector[3] = angle_vector[4];
    theta_vector[4] = angle_vector[5];
    theta_vector[5] = angle_vector[6];

    // Fill theta-dependent variables into joint matrices
    mat_a01.set_value(0, 0, cos(theta_vector[0]));
    mat_a01.set_value(0, 2, -sin(theta_vector[0]));
    mat_a01.set_value(1, 0, sin(theta_vector[0]));
    mat_a01.set_value(1, 2, cos(theta_vector[0]));

    mat_a12.set_value(0, 0, sin(theta_vector[1]));
    mat_a12.set_value(0, 1, cos(theta_vector[1]));
    mat_a12.set_value(0, 3, 450 * sin(theta_vector[1]));
    mat_a12.set_value(1, 0, -cos(theta_vector[1]));
    mat_a12.set_value(1, 1, sin(theta_vector[1]));
    mat_a12.set_value(1, 3, -450 * cos(theta_vector[1]));

    mat_a23.set_value(0, 0, -sin(theta_vector[2]));
    mat_a23.set_value(0, 2, cos(theta_vector[2]));
    mat_a23.set_value(1, 0, cos(theta_vector[2]));
    mat_a23.set_value(1, 2, sin(theta_vector[2]));

    mat_a34.set_value(0, 0, cos(theta_vector[3]));
    mat_a34.set_value(0, 2, -sin(theta_vector[3]));
    mat_a34.set_value(1, 0, sin(theta_vector[3]));
    mat_a34.set_value(1, 2, cos(theta_vector[3]));

    mat_a45.set_value(0, 0, cos(theta_vector[4]));
    mat_a45.set_value(0, 2, sin(theta_vector[4]));
    mat_a45.set_value(1, 0, sin(theta_vector[4]));
    mat_a45.set_value(1, 2, -cos(theta_vector[4]));

    mat_a56.set_value(0, 0, cos(theta_vector[5]));
    mat_a56.set_value(0, 1, -sin(theta_vector[1]));
    mat_a56.set_value(1, 0, sin(theta_vector[5]));
    mat_a56.set_value(1, 1, cos(theta_vector[1]));

    transform_mat = mat_a01 * mat_a12 * mat_a23 * mat_a34 * mat_a45 * mat_a56;

    //transform_mat.print();

    // Convert transformation matrix to vector of vector form.
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            temp_row_vector[i] = transform_mat.get_value(j , i);

        }
        result_vector[j] = temp_row_vector;
    }


    return result_vector;
}


// Read values from transformation matrix to a vector of coordinates
std::vector<double> PA10Controller::ForwardConverter(std::vector<std::vector<double>> transform_mat)
{
    std::vector<double> cartesian_vector(3);

    cartesian_vector[0] = transform_mat[0][3];
    cartesian_vector[1] = transform_mat[1][3];
    cartesian_vector[2] = transform_mat[2][3];

    return cartesian_vector;
}


// Calculate inverse kinematics to find joint positions based on cartesian positions
std::vector<double> PA10Controller::InverseKinematics(std::vector<double> end_effector_vector, std::vector<std::vector<double>> transformation_matrix, std::vector<double> cartesian_vector_0)
{

    std::vector<double> result_vector(6), prev_val_vector(6);
    Matrix inverse_temp_matrix, transform_matrix_prime, transformation_matrix_mat;

    // Define intermediate variables
    double a2, d1, d4, d6;
    double pxi, pyi, pzi, bxp, byp, bzp, nxp, nyp, txp, typ, inter1, inter21, inter22, inter3, inter31, theta1, theta11, theta12, theta2,
            theta21, theta22, theta3, theta31, theta32, theta4, theta5, theta6;
    double px, py, pz, bx, by, bz;

    // Calculate intermediate values
    px = transformation_matrix[0][3] + (end_effector_vector[0] - cartesian_vector_0[0]);
    py = transformation_matrix[1][3] + (end_effector_vector[1] - cartesian_vector_0[1]);
    pz = transformation_matrix[2][3] + (end_effector_vector[2] - cartesian_vector_0[2]);

    bx = transformation_matrix[0][2];
    by = transformation_matrix[1][2];
    bz = transformation_matrix[2][2];

    a2 = 450.0;
    d1 = 317.0;
    d4 = 480.0;
    d6 = 70.0;

    pxi = px - d6 * bx;
    pyi = py - d6 * by;
    pzi = pz - d6 * bz;

    inter1 = pow(pxi, 2) + pow(pyi, 2);


    //theta11 = bind_angle(2 * atan2(-pxi + sqrt(inter1), pyi));
    //theta12 = bind_angle(2 * atan2(-pxi - sqrt(inter1), pyi));

    theta11 = 2 * atan2(-pxi + sqrt(inter1), pyi);
    theta12 = 2 * atan2(-pxi - sqrt(inter1), pyi);

    if((std::abs(prev_val_vector[0]) - theta11) > std::abs(prev_val_vector[0] - theta12)){
        theta1 = theta12;
    } else {
        theta1 = theta11;
    }

    inter3 = (pxi * cos(theta1)) + (pyi * sin(theta1));
    inter31 = (pow(inter3, 2) + pow((-pzi + d1), 2) - pow(a2, 2) - pow(d4, 2)) / (2 * a2 * d4);

    theta31 = atan2(sqrt(1 - pow(inter31, 2)), ((pow(inter3, 2) + pow((-pzi + d1), 2) - pow(a2, 2) - pow(d4, 2)) / (2 * a2 * d4)));
    theta32 = atan2(-sqrt(1 - pow(inter31, 2)), ((pow(inter3, 2) + pow((-pzi + d1), 2) - pow(a2, 2) - pow(d4, 2)) / (2 * a2 * d4)));

    if((std::abs(prev_val_vector[1]) - theta31) > std::abs(prev_val_vector[1] - theta32)){
        theta3 = theta32;
    } else {
        theta3 = theta31;
    }

    inter21 = (pow(d4 * sin(theta3), 2) + pow(a2 + d4 * cos(theta3), 2) - pow(d1 - pzi, 2));
    inter22 = a2 - d1 + pzi + (d4 * cos(theta3));

    theta21 = (-2 * atan2((((d4 * sin(theta3)) - sqrt(inter21))), inter22));
    theta22 = (-2 * atan2((((d4 * sin(theta3)) + sqrt(inter21))), inter22));

    if((std::abs(prev_val_vector[2]) - theta21) > std::abs(prev_val_vector[2] - theta22)){
        theta2 = theta22;
    } else {
        theta2 = theta21;
    }

    // Create inverse matrix of A1 * A2 * A3, based on manually calculated values.
    inverse_temp_matrix.set_value(0, 0, (cos(theta2 + theta3)*cos(theta1)));
    inverse_temp_matrix.set_value(0, 1, (cos(theta2 + theta3)*sin(theta1)));
    inverse_temp_matrix.set_value(0, 2, (-sin(theta2 + theta3)));
    inverse_temp_matrix.set_value(0, 3, (317*sin(theta2 + theta3) + 450*sin(theta3)));

    inverse_temp_matrix.set_value(1, 0, (-sin(theta1)));
    inverse_temp_matrix.set_value(1, 1, (cos(theta1)));
    inverse_temp_matrix.set_value(1, 2, 0);
    inverse_temp_matrix.set_value(1, 3, 0);

    inverse_temp_matrix.set_value(2, 0, (sin(theta2 + theta3)*cos(theta1)));
    inverse_temp_matrix.set_value(2, 1, (sin(theta2 + theta3)*sin(theta1)));
    inverse_temp_matrix.set_value(2, 2, (cos(theta2 + theta3)));
    inverse_temp_matrix.set_value(2, 3, (-317*cos(theta2 + theta3) - 450*cos(theta3)));

    inverse_temp_matrix.set_value(3, 0, 0);
    inverse_temp_matrix.set_value(3, 1, 0);
    inverse_temp_matrix.set_value(3, 2, 0);
    inverse_temp_matrix.set_value(3, 3, 1);

    for (int i = 0; i < 4 ; ++i) {
        for (int j = 0; j < 4; ++j) {
            double val = transformation_matrix[i][j];
            transformation_matrix_mat.set_value(i, j, val);

        }

    }

    //inverse_temp_matrix.print();
    //transformation_matrix_mat.print();

    // Create matrix T'
    transform_matrix_prime = inverse_temp_matrix * transformation_matrix_mat;

    transform_matrix_prime.print();

    // Calculate intermediate values for theta4, 5, 6 calculation.
    nxp = transform_matrix_prime.get_value(0, 0);
    nyp = transform_matrix_prime.get_value(1, 0);

    txp = transform_matrix_prime.get_value(0, 1);
    typ = transform_matrix_prime.get_value(1, 1);

    bxp = transform_matrix_prime.get_value(0, 2);
    byp = transform_matrix_prime.get_value(1, 2);
    bzp = transform_matrix_prime.get_value(2, 2);


    // Calculate theta4, theta5, theta6
    theta4 = atan2(byp, bxp);

    theta5 = atan2(((bxp * cos(theta4)) + (byp * sin(theta4))), bzp);

    theta6 = atan2(((-nxp * sin(theta4)) + (nyp * cos(theta4))), ((-txp * sin(theta4)) + (typ * cos(theta4))));

    result_vector[0] = theta1;
    result_vector[1] = theta2;
    result_vector[2] = theta3;
    result_vector[3] = theta4;
    result_vector[4] = theta5;
    result_vector[5] = theta6;

    // For delta-value based movement, record previous joint angle values
    prev_val_vector = result_vector;

    return result_vector;
}


// Function used to limit joint angles (currently not used)
double PA10Controller::bind_angle(double angle)
{
    if(angle < 0) {
        return std::abs(angle) < angle + (2*M_PI) ? angle : angle + (2*M_PI);
    }
    return angle < std::abs(angle - (2*M_PI)) ? angle : angle - (2*M_PI);
}


// Find coefficient values for each interpolation function
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


// Find interpolated values between x_0 and x_f to use in smooth path generation
double PA10Controller::findCur5thOrderVal(double ticks, double duration_ticks, double x_0, double x_f)
{

    std::vector<double> coeff = FindCoeffVector(x_0, x_f, duration_ticks);
    //return coeff[0]+(coeff[1]*ticks)+(coeff[2]*ticks*ticks)+(coeff[3]*ticks*ticks*ticks)+(coeff[4]*ticks*ticks*ticks*ticks)+(coeff[5]*ticks*ticks*ticks*ticks*ticks);

    return coeff[0]+(coeff[1] * ticks)+(coeff[2] * pow(ticks, 2))+(coeff[3] * pow(ticks, 3))+(coeff[4] * pow(ticks, 4))+(coeff[5] * pow(ticks, 5));
}


// Generate line path and calculate corresponding joint positions
std::vector<double> PA10Controller::LinePathGenerator(std::vector<double> cartesian_vector_0, std::vector<double> cartesian_vector_d, double movement_dur, double ticks, std::vector<std::vector<double>> transformation_mat)
{
    std::vector<double> via_point_vector(3);
    std::vector<double> cartesian_vector_f(6);
    std::vector<double> via_angles_vector(7);
    std::vector<double> theta_vector(6);

    // Interpolate values between 0 and 1 for smooth movement
    double val5thorder = findCur5thOrderVal(ticks, movement_dur, 0, 1);

    // Find final cartesian positions for each step (based on initial and delta values)
    for (int j = 0; j < 6; ++j) {
        cartesian_vector_f[j] = cartesian_vector_0[j] + cartesian_vector_d[j];
    }

    via_point_vector[0] = cartesian_vector_0[0] + ((cartesian_vector_f[0] - cartesian_vector_0[0]) * val5thorder);
    via_point_vector[1] = cartesian_vector_0[1] + ((cartesian_vector_f[1] - cartesian_vector_0[1]) * val5thorder);
    via_point_vector[2] = cartesian_vector_0[2] + ((cartesian_vector_f[2] - cartesian_vector_0[2]) * val5thorder);


    theta_vector = InverseKinematics(via_point_vector, transformation_mat, ForwardConverter(transformation_mat));

    via_angles_vector[0] = theta_vector[0];
    via_angles_vector[1] = theta_vector[1];
    via_angles_vector[2] = 0;
    via_angles_vector[3] = theta_vector[2];
    via_angles_vector[4] = theta_vector[3];
    via_angles_vector[5] = theta_vector[4];
    via_angles_vector[6] = theta_vector[5];

    // Debugging printing
    for (int i = 0; i < 6; ++i) {
        std::cout << theta_vector[i];
        //std::cout << via_point_vector[i];
        std::cout << "; ";
    }
    std::cout << "\n";


    return via_angles_vector;
}


// Generate circular path and calculate corresponding joint positions
std::vector<double> PA10Controller::CirclePathGenerator(std::vector<double> cartesian_vector_0, std::vector<double> cartesian_vector_d, double movement_dur, double ticks, std::vector<std::vector<double>> transformation_mat)
{
    std::vector<double> via_point_vector(3);
    std::vector<double> cartesian_vector_f(6);
    std::vector<double> via_angles_vector(7);
    std::vector<double> theta_vector(6);

    // Interpolate values between 0 and 2*pi for smooth movement
    double val5thorder = findCur5thOrderVal(ticks, movement_dur, 0, (M_PI * 2));

    // Find final cartesian positions for each step (based on initial and delta values)
    for (int j = 0; j < 6; ++j) {
        cartesian_vector_f[j] = cartesian_vector_0[j] + cartesian_vector_d[j];
    }

//    via_point_vector[0] = cartesian_vector_0[0] + ((cartesian_vector_f[0] - cartesian_vector_0[0]) * val5thorder);
//    via_point_vector[1] = cartesian_vector_0[1] + ((cartesian_vector_f[1] - cartesian_vector_0[1]) * val5thorder);
//    via_point_vector[2] = cartesian_vector_0[2] + ((cartesian_vector_f[2] - cartesian_vector_0[2]) * val5thorder);


    // Debugging printing
    for (int i = 0; i < 3; ++i) {
        //std::cout << theta_vector[i];
        std::cout << via_point_vector[i];
        std::cout << "; ";
    }
    std::cout << "\n";

}

void PA10Controller::MoveJoint(std::vector<double> via_angles_vector)
{

    for(int i=0; i<7; ++i)
    {
        cur_joint.position[i] = via_angles_vector[i];
    }

    //publish joint states
    joint_pub.publish(cur_joint);
}


void PA10Controller::StartMoving()
{
    ros::Rate loop_rate(10); //set loop rate 10[Hz]

    ROS_INFO("Start Moving");
    ticks=0.0;

    std::vector<std::vector<double>> trans_mat = ForwardKinematics(initial_angle_vector);

    //initialize joint names
    initJointState(&cur_joint);

    while(ros::ok() && !ReachGoalFlag)
    {
        std::vector<double> d  = LinePathGenerator(ForwardConverter(trans_mat), delta_cartesian_vector, movement_dur, ticks, trans_mat);
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
