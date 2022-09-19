#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <iostream>
#include "Math/orientation_tools.h"

using namespace pinocchio;

class RobotModel{

public:
    RobotModel();
    ~RobotModel() = default;

    // Eigen::Vector3d getJointTranslation();
    Eigen::Vector3d getJointTranslation_LOCAL(Eigen::VectorXd q_now, int Joint_ID);
    // Eigen::Matrix3d getJointRotation();
    // Data::Matrix6x getJointJacobian();
    // Data::Matrix6x getJacobDerivative();
    
    int GetNv();
    int GetNq();
    void ComputeAllTerm(Eigen::VectorXd q, Eigen::VectorXd v);
    void ComputeCoriolisMatrix(Eigen::VectorXd q, Eigen::VectorXd v);
    void ComputeGeneralizedGravity(Eigen::VectorXd q);
    void InverseKinematic(Eigen::Vector3d Pdes, Eigen::VectorXd q_now, Eigen::VectorXd &qdes);

    Data data;
    Model model;
private:
    Eigen::VectorXd _q;
    Eigen::VectorXd _dq;
    Eigen::MatrixXd _J;
    Eigen::MatrixXd _dJ;
    Eigen::MatrixXd _Ji;
    Eigen::Matrix<double, 6, 1> err;
    Eigen::Vector3d _oneLegDq;

    int JOINT_ID;
    double eps;
    double damp;
    int IT_MAX;
    double DT;
    double ydes;
    bool success = false;
    const std::string _urdfFilename = std::string("../../robot_model/xml_v1/urdf_v1.urdf");
};

#endif