#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <mutex>
#include <thread>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "eigen3/unsupported/Eigen/MatrixFunctions"
#include "Timer.h"
#include "savemat.h"
#include "SharedMemory.h"
#include "SharedMemoryInterface.h"
#include "qpOASES.hpp"
#include "qpOASES/QProblem.hpp"
#include "RobotModel.h"
#include "ControlData.h"
#include "RobotCommData.h"
#include "Trajectory.h"
#include "UserInterface.h"

using namespace qpOASES;

class RobotController{

public:
    RobotController();
    ~RobotController();

    void init();
    void run();
    void updateState();
    void updateCommand();
    void updateBuffer();
    void runController();
    void standUp();
    void legMode();
    void turning();

    void computeKinematicsDynamics();
    void VirtualMITMode(Eigen::VectorXd q, Eigen::VectorXd Kp, Eigen::VectorXd dq, Eigen::VectorXd Kd, Eigen::VectorXd tau_ff);

    int modelNv, modelNq;
    int k = 0;
    Eigen::VectorXd qDes, dqDes;
    Eigen::VectorXd standQ;
    Eigen::VectorXd tauFeedForward, posStiffness, velocityStiffness;
    SharedMemoryObject<SharedMemoryInterface> _sharedMemory;
private:
    bool turningMode = false;
    bool standMode = true;
    JointsState *_motorState;
    JointsCmd *_motorCmd;
    RobotModel leggedWheel;
    RobotParameter robotParam;
    RobotData robotData;
    RobotCmd robotCmd;
    Trajectory trajectory;
    LastState lastState;
    UserInterface *interface;
    Timer time;
};

#endif