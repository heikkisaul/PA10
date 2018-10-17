#include <iostream>
#include <vector>
#include <cmath>

#include "Matrix.hpp"
#include "PA10Kinematics.hpp"


PA10Kinematics::PA10Kinematics(){
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
}


std::vector<double> PA10Kinematics::forward_kinematics(std::vector<double> theta_vector) {

    Matrix transform_mat;
    std::vector<double> result_vector(6);

    for(int i = 0; i <= 6; i = i + 1){
        theta_vector[i] = theta_vector[i] * M_PI/180;
    }

    //fill theta-dependent variables into joint matrices
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

    result_vector[0] = transform_mat.get_value(0, 3);
    result_vector[1] = transform_mat.get_value(1, 3);
    result_vector[2] = transform_mat.get_value(2, 3);
    result_vector[3] = atan2(transform_mat.get_value(0, 2), -transform_mat.get_value(1, 2));
    result_vector[4] = acos(transform_mat.get_value(2, 2));
    result_vector[5] = atan2(transform_mat.get_value(2, 0), transform_mat.get_value(2, 1));


    return result_vector;
}


std::vector<double> PA10Kinematics::inverse_kinematics(std::vector<double> end_effector_vector){
    
    Matrix mat_end_eff_ori(3, 3), mat_inter(3, 3), mat_inter_inv(3, 3), T_prim(3, 3);
    std::vector<double> result_vector(6);

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
    double pxi, pyi, pzi, bxp, byp, bzp, nxp, nyp, txp, typ, inter1, inter21, inter22, inter3, theta1, theta2,
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

    return result_vector;

}


double PA10Kinematics::bind_angle(double angle)
{
    if(angle < 0) {
        return std::abs(angle) < angle + (2*M_PI) ? angle : angle + (2*M_PI);
    }
    return angle < std::abs(angle - (2*M_PI)) ? angle : angle - (2*M_PI);
}