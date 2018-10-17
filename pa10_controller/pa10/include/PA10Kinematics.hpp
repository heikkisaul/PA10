//
// Created by heikki on 18/10/05.
//

#ifndef PA10_KINEMATICS_PA10KINEMATICS_HPP
#define PA10_KINEMATICS_PA10KINEMATICS_HPP

#include <vector>

#include "Matrix.hpp"

class PA10Kinematics
{
public:
    PA10Kinematics();
    std::vector<double> forward_kinematics(std::vector<double> theta_vector);
    std::vector<double> inverse_kinematics(std::vector<double> position_vector);
private:
    //declare joint angle matrices and end effector position-orientation matrices
    Matrix mat_a01, mat_a12, mat_a23, mat_a34, mat_a45, mat_a56;
    Matrix mat_phi, mat_theta, mat_psi;
    //declare function to deal with negative angles
    double bind_angle(double angle);
};

#endif //PA10_KINEMATICS_PA10KINEMATICS_HPP
