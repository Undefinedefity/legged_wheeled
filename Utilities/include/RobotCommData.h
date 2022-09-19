#ifndef ROBOT_COMM_DATA_H
#define ROBOT_COMM_DATA_H

#include "cppTypes.h"

struct IMUData {
    Eigen::Vector4d quat;
    Eigen::Vector3d gyro;
    Eigen::Vector3d acc;
};

struct JointsState {
    Vec16 qPos;  // left leg (hip knee wheel), right leg, arm
    Vec16 qVel;
    Vec16 tau;
};

struct MeasuredState {
    IMUData imuData; // [w x y z]
    JointsState jointsState;
};

struct JointsCmd {
    Vec16 Kp = Vec16::Zero();
    Vec16 Kd = Vec16::Zero();
    Vec16 qposDes = Vec16::Zero();
    Vec16 qvelDes = Vec16::Zero();
    Vec16 tauFeedForward = Vec16::Zero();
};

#endif //MININEZHA_COMM_DATA_H