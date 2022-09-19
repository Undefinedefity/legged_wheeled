#ifndef ROBOT_CTRLDATA_H
#define ROBOT_CTRLDATA_H

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "cmath"
#include <vector>

struct RobotParameter
{
    bool getParamsFromYAML(const char *filename);
    Eigen::Matrix<double, 16, 1> torquelimit;
    std::vector<double> jointlimit;
    std::vector<double> Kp;
    std::vector<double> Kd;
    double desHeight;
    double velocityRef;
    int controlMode;
};

struct RobotData
{
    Eigen::Matrix<double, 16, 1> motorTau;

    Eigen::Matrix<double, 16, 1> motorQ;
    Eigen::Matrix<double, 16, 1> motorDq;

    Eigen::MatrixXd J;
    Eigen::MatrixXd dJ;

    Eigen::MatrixXd M;
	Eigen::MatrixXd C;
	Eigen::MatrixXd G;
};

struct RobotCmd
{
    Eigen::Matrix<double, 16, 1> cmdTau;
    Eigen::Matrix<double, 16, 1> cmdQ;
    Eigen::Matrix<double, 16, 1> cmdDq;
};

struct LastState
{
    Eigen::Matrix<double, 16, 1> motorTauLast = Eigen::Matrix<double, 16, 1>::Zero();
    Eigen::Matrix<double, 16, 1> motorQLast = Eigen::Matrix<double, 16, 1>::Zero();
    Eigen::Matrix<double, 16, 1> motorDqLast = Eigen::Matrix<double, 16, 1>::Zero();    
};

#endif