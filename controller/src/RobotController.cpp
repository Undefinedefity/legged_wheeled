#include "RobotController.h"

RobotController::RobotController(){
    std::cout << "Enter the contsructor function" << std::endl;
    robotParam.getParamsFromYAML("../../config/parameters.yaml");
    
    _sharedMemory.attach(ROBOT_SHARE_MEMORY_NAME);
    _sharedMemory().init();

    _motorState = &_sharedMemory().measuredState.jointsState;
    _motorCmd = &_sharedMemory().jointsCmd;

    interface = new UserInterface(&robotParam);

    this->modelNq = leggedWheel.GetNq();
    this->modelNv = leggedWheel.GetNv();
    std::cout << "Robot controller construct finished" << std::endl; 
}

void RobotController::init(){
    for (int i = 0; i < 16; ++i)
        robotParam.torquelimit[i] = robotParam.jointlimit[i];
    robotData.J.resize(6, this->modelNv);
    robotData.dJ.resize(6, this->modelNv);
    for(int i = 0; i < 16; ++i)
        robotData.motorQ[i] = 0;

    trajectory.jointMaxq.resize(16, 1);
    trajectory.jointMinq.resize(16, 1);
    trajectory.jointSpeedLimit.resize(16, 1);
    trajectory.endQ.resize(16, 1);
    trajectory.initialQ = robotData.motorQ;

    this->standQ.resize(16, 1);
    this->tauFeedForward.resize(16, 1);
    this->posStiffness.resize(16, 1);
    this->velocityStiffness.resize(16, 1);
    this->qDes.resize(16, 1);
    this->dqDes.resize(16, 1);

    for(int i = 0; i < 16; ++i){
        trajectory.jointMaxq[i] = 3.14;
        trajectory.jointMinq[i] = -3.14;
        trajectory.jointSpeedLimit[i] = 21;
        trajectory.endQ[i] = 0;
        this->tauFeedForward[i] = 0;
        this->posStiffness[i] = robotParam.Kp[i];
        this->velocityStiffness[i] = robotParam.Kd[i];
    }

    std::cout << "generating trajectory!" << std::endl;
    
    Eigen::Vector3d WheelPos;
    WheelPos[0] = -0.124295;
    WheelPos[1] = 0.241749; // y distance between origin of wheel frame and origin of base frame
    WheelPos[2] = -robotParam.desHeight + 0.09;
    leggedWheel.InverseKinematic(WheelPos, robotData.motorQ, standQ);
    for (int i = 1; i < 4; ++i){
        trajectory.endQ[i] = standQ[i];
        trajectory.endQ[i + 4] = standQ[i];
        trajectory.endQ[i + 8] = -standQ[i];
        trajectory.endQ[i + 12] = -standQ[i];
    }
    // trajectory.endQ[0] = 37.87 / 180 * 3.14;
    // trajectory.endQ[4] = - 37.87 / 180 * 3.14;
    // trajectory.endQ[8] = 37.87 / 180 * 3.14;
    // trajectory.endQ[12] = 37.87 / 180 * 3.14;

    trajectory.generateTraj(trajectory.initialQ, trajectory.endQ, 0.05);
    trajectory.startTime = time.getMs();
    std::cout << "Trajectory generate done!" << std::endl;

    // std::cout << "model_nv: " << leggedWheel.GetNv() << std::endl; 
    // std::cout << "model_nq: " << leggedWheel.GetNq() << std::endl; 
    // std::cout << "!!! initialize finished !!!" << std::endl;
}

void RobotController::updateState(){
    for (int i = 0; i < 16; i++){
        robotData.motorQ(i) = _motorState->qPos(i);
        robotData.motorDq(i) = _motorState->qVel(i);
    }
}

void RobotController::computeKinematicsDynamics(){
    robotData.J.setZero();
    robotData.dJ.setZero();

    leggedWheel.ComputeAllTerm(robotData.motorQ, robotData.motorDq);
    leggedWheel.ComputeCoriolisMatrix(robotData.motorQ, robotData.motorDq);
    leggedWheel.ComputeGeneralizedGravity(robotData.motorDq);

    robotData.M = leggedWheel.data.M;
    robotData.C = leggedWheel.data.C;
    robotData.G = leggedWheel.data.g;
}

void RobotController::updateCommand(){
    // set limit for joint torque
    for (int i = 0; i < 16; i++){
        if(abs(robotCmd.cmdTau(i)) > robotParam.torquelimit(i)){
            robotCmd.cmdTau(i) = robotParam.torquelimit(i) * abs(robotCmd.cmdTau(i)) / robotCmd.cmdTau(i);
        }
    }

    if (robotCmd.cmdTau.hasNaN()){
        robotCmd.cmdTau = lastState.motorTauLast;
    }

    for (int i = 0; i < 16; i++){
        _motorCmd->tauFeedForward[i] = robotCmd.cmdTau[i];
    }
    // for (int i = 0; i < 16; i++){
    //     _motorCmd->qposDes[i] = robotCmd.cmdQ[i];
    // }
}

void RobotController::updateBuffer(){
    lastState.motorTauLast = robotCmd.cmdTau;
}

void RobotController::run(){
    std::cout << "k: " << k << std::endl;

    interface->update();

    updateState();
    if(k == 0)
        init();

    // trajectory.currentTime = time.getMs();
    // trajectory.T = (trajectory.currentTime - trajectory.startTime ) * pow(10, -3);
    runController();
    updateCommand();
    updateBuffer();

    k++;
}

void RobotController::VirtualMITMode(Eigen::VectorXd q, Eigen::VectorXd Kp, Eigen::VectorXd dq, Eigen::VectorXd Kd, Eigen::VectorXd tau_ff){
    for(int i = 0; i < 16; ++i){
        robotCmd.cmdTau[i] = Kp[i] * (q[i] - robotData.motorQ[i]) + Kd[i] * (dq[i] - robotData.motorDq[i]) + tau_ff[i];
    }
}

void RobotController::runController(){
    computeKinematicsDynamics();
    if(robotParam.controlMode == 0){
        if(!standMode){
            for (int i = 0; i < 16; ++i){
                trajectory.initialQ[i] = robotData.motorQ[i];
                trajectory.endQ[i] = 0;
            }
            trajectory.initialQ[3] = 0;
            trajectory.initialQ[7] = 0;
            trajectory.initialQ[11] = 0;
            trajectory.initialQ[15] = 0;
            for (int i = 1; i < 3; ++i){
                trajectory.endQ[i] = standQ[i];
                trajectory.endQ[i + 4] = standQ[i];
                trajectory.endQ[i + 8] = -standQ[i];
                trajectory.endQ[i + 12] = -standQ[i];
            }
            trajectory.generateTraj(trajectory.initialQ, trajectory.endQ, 0.1);
            trajectory.startTime = time.getMs();
            standMode = true;
            turningMode = false;
        }

        trajectory.currentTime = time.getMs();
        trajectory.T = (trajectory.currentTime - trajectory.startTime ) * pow(10, -3);
        standUp();
    }
    else if(robotParam.controlMode == 1)
        legMode();
    else if(robotParam.controlMode == 2){
        if(!turningMode){
            for (int i = 0; i < 16; ++i){
                trajectory.initialQ[i] = robotData.motorQ[i];
                trajectory.endQ[i] = 0;
            }
            trajectory.initialQ[3] = 0;
            trajectory.initialQ[7] = 0;
            trajectory.initialQ[11] = 0;
            trajectory.initialQ[15] = 0;
            for (int i = 1; i < 3; ++i){
                trajectory.endQ[i] = standQ[i];
                trajectory.endQ[i + 4] = standQ[i];
                trajectory.endQ[i + 8] = -standQ[i];
                trajectory.endQ[i + 12] = -standQ[i];
            }
            trajectory.endQ[0] = 37.87 / 180 * 3.14;
            trajectory.endQ[4] = - 37.87 / 180 * 3.14;
            trajectory.endQ[8] = 37.87 / 180 * 3.14;
            trajectory.endQ[12] = 37.87 / 180 * 3.14;
            trajectory.generateTraj(trajectory.initialQ, trajectory.endQ, 0.1);
            trajectory.startTime = time.getMs();
            standMode = false;
            turningMode = true;
            // getchar();
        }
        trajectory.currentTime = time.getMs();
        trajectory.T = (trajectory.currentTime - trajectory.startTime ) * pow(10, -3);
        turning();
    }
}

void RobotController::standUp(){
    trajectory.getJointCmd(qDes, dqDes);

    // test
    // qDes[0] = 1;
    // qDes[5] = 0.9;
    // qDes[6] = 0.528;
    // qDes[9] = -0.9;
    // qDes[10] = -0.529;

    // std::cout << "Current Time: " << trajectory.T << std::endl;
    std::cout << "Mode: " << robotParam.controlMode << std::endl;
    // std::cout << leggedWheel.data.oMi[8].translation() << std::endl;
    std::cout << "Velocity ref: " << robotParam.velocityRef << std::endl;

    // update wheel velocity
    // forward
    dqDes[3] = 4 * robotParam.velocityRef / 0.095;
    dqDes[7] = 4 * robotParam.velocityRef / 0.095;
    dqDes[11] = -4 * robotParam.velocityRef / 0.095;
    dqDes[15] = -4 * robotParam.velocityRef / 0.095;


    VirtualMITMode(qDes, posStiffness, dqDes, velocityStiffness, tauFeedForward);
}

void RobotController::turning(){
    trajectory.getJointCmd(qDes, dqDes);
    
    //turning
    dqDes[3] = 4 * robotParam.velocityRef / 0.095;
    dqDes[7] = 4 * robotParam.velocityRef / 0.095;
    dqDes[11] = 4 * robotParam.velocityRef / 0.095;
    dqDes[15] = 4 * robotParam.velocityRef / 0.095;
    std::cout << "Mode: " << robotParam.controlMode << std::endl;
    std::cout << "Velocity ref: " << robotParam.velocityRef << std::endl;
    VirtualMITMode(qDes, posStiffness, dqDes, velocityStiffness, tauFeedForward);
}

void RobotController::legMode(){
    
}

RobotController::~RobotController(){
    _sharedMemory().ctrl_attached = false;
    _sharedMemory.detach();
    delete _motorCmd;
    delete _motorState;
}