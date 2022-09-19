//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_ESTIMATESTATE_H
#define XIAOTIANHYBRID_ESTIMATESTATE_H

#include "MiniNezhaCommData.h"


struct FloatingBaseState {
    Eigen::Vector3d pos, rpy, vWorld, vBody, omegaWorld, omegaBody, aBody, aWorld;
    Eigen::Matrix3d R_wb;
    Eigen::Vector4d quat;
};

struct FootState {
    Eigen::Vector3d pos, rpy, vWorld, omegaWorld;
    Eigen::Matrix3d R_wf, R_wh;
//    ContactState contactState;
};

enum class Feet {
    FL, FR, HL, HR
};

struct EstimatedState {
    FloatingBaseState floatingBaseState;
    JointsState jointsState;  //change to Jointstate
    FootState footState[4];

    Eigen::Vector4d contactPhaseDes = Eigen::Vector4d::Zero();
};
#endif //XIAOTIANHYBRID_ESTIMATESTATE_H
