#include "Trajectory.h"

Trajectory::Trajectory(){
    _motionTime = 0;
    _a3 = 0;
    _a4 = 0;
    _a5 = 0;
}

void Trajectory::generateTraj(Eigen::VectorXd &Qstart, Eigen::VectorXd &Qend, double speed_ratio){
    initialQ = Qstart;
    endQ = Qend;

    _motionTime = 0, _tmp_motionTime = 0;
    for(int i = 0; i < 16; ++i){
        _tmp_motionTime = 15 *fabs(endQ(i) - initialQ(i)) / (8 * jointSpeedLimit.at(i) * speed_ratio);
        if(_tmp_motionTime > _motionTime)
            _motionTime = _tmp_motionTime;
    }
    std::cout << "Total motion time: " << _motionTime << std::endl;
    _a3 =  10 / pow(_motionTime, 3);
    _a4 = -15 / pow(_motionTime, 4);
    _a5 =   6 / pow(_motionTime, 5);

}

void Trajectory::getJointCmd(Eigen::VectorXd &q, Eigen::VectorXd &dq){
    if (T < _motionTime){
        _s = _a3 * pow(T, 3) + _a4 * pow(T, 4) + _a5 * pow(T, 5);
        _sdot = 3 * _a3 * pow(T, 2) + 4 * _a4 * pow(T, 3) + 5 * _a5 * pow(T, 4);

        q = _s * (endQ - initialQ) + initialQ;
        dq = _sdot * (endQ - initialQ);
    }
    else{
        q = endQ;
        dq.setZero();
    }
}