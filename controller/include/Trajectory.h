#ifndef XIAOTIANARM_TRAJECTORY_H
#define XIAOTIANARM_TRAJECTORY_H

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "eigen3/unsupported/Eigen/MatrixFunctions"
#include <vector>
#include "Timer.h"
#include <iostream>

class Trajectory
{
public:
    Trajectory();
    // ~Trajectory();

    void generateTraj(Eigen::VectorXd &Qstart, Eigen::VectorXd &Qend, double speed_ratio);
    void getJointCmd(Eigen::VectorXd &q, Eigen::VectorXd &dq);
    double currentTime;
    double startTime;
    double T;
    std::vector<double> jointMaxq;
    std::vector<double> jointMinq;
    std::vector<double> jointSpeedLimit;
    Eigen::VectorXd initialQ, endQ;

private:
    double _motionTime, _tmp_motionTime;
    double _a3, _a4, _a5;   // Quintic Polynomials 
    double _s, _sdot;
};

#endif