#include "RobotModel.h"

RobotModel::RobotModel(){
    // load the model from urdf file
    pinocchio::urdf::buildModel(_urdfFilename, model);
    data = Data(model);
    _q = pinocchio::neutral(model);
    _dq = Eigen::VectorXd::Zero(model.nv);
    _J = Eigen::MatrixXd::Zero(6, model.nv);
    _dJ = Eigen::MatrixXd::Zero(6, model.nv);

    eps = 1e-2;
    IT_MAX = 500;
    DT = 1e-1;
    damp = 1e-6;

    std::cout << "model name: " << model.name << " initialize succeefully! " << std::endl;

    //  1-4 left_back
    //  5-8 left_front
    //  9-12 right_back
    //  13-16 right_front

    // for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
    // {
    //     std::cout
    //         << "[" << joint_id << "]"
    //         << std::setw(24) << std::left
    //         << model.names[joint_id] << ": "
    //         << std::fixed << std::setprecision(2)
    //         << data.oMi[joint_id].translation().transpose()
    //         << std::endl;
    // }
}

void RobotModel::InverseKinematic(Eigen::Vector3d Pdes, Eigen::VectorXd q_now, Eigen::VectorXd &qdes){
    JOINT_ID = 4;
    _dq.setZero();
    _q.setZero();
    _q = q_now;

    pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Pdes);
    for (int i = 0;; i++)
    {
        pinocchio::forwardKinematics(model, data, _q);
        const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
        err = pinocchio::log6(dMi).toVector();
        if (err.block<3, 1>(0, 0).norm() < eps)
        {
            success = true;
            break;
        }
        if (i >= IT_MAX)
        {
            success = false;
            std::cout << "conergence failed" << std::endl;
            break;
        }
        pinocchio::computeJointJacobian(model, data, _q, JOINT_ID, _J);
        _Ji = _J.block<3, 3>(0, 0);
        Eigen::Matrix3d JJt;
        JJt.noalias() = _Ji * _Ji.transpose();
        JJt.diagonal().array() += damp;
        _oneLegDq.noalias() = -_Ji.transpose() * JJt.ldlt().solve(err.block<3, 1>(0, 0));
        for (int j = 0 ; j < 3; ++j)
            _dq[j] = _oneLegDq[j];
        _q = pinocchio::integrate(model, _q, _dq * DT);
    }

    if (success)
    {
        for (int i = 0; i < model.nq; i++)
        {
            _q(i) = fmod(_q(i), (2 * M_PI));
            if (_q(i) >= M_PI && _q(i) <= 2 * M_PI)
            {
                _q(i) = _q(i) - 2 * M_PI;
            }
            if (_q(i) <= -M_PI && _q(i) >= -2 * M_PI)
            {
                _q(i) = _q(i) + 2 * M_PI;
            }

        }
        qdes = _q;
        std::cout << "Des: " << Pdes << std::endl;
        std::cout << data.oMi[JOINT_ID] << std::endl;
        std::cout << "err: " << err.block<3, 1>(0, 0).norm() << std::endl;
        for(int i = 0; i < 3; ++i)
            std::cout << i << ": " << _q[i] << std::endl;
    }
    else
    {
        qdes = q_now;
        std::cout << "Joint id:" << JOINT_ID << std::endl
                  << " Warning: the iterative algorithm has not reached convergence to the desired precision"
                  << std::endl;
    }
}

void RobotModel::ComputeAllTerm(Eigen::VectorXd q_now, Eigen::VectorXd v_now)
{
    _q = q_now;
    _dq = v_now;
    pinocchio::computeAllTerms(model, data, _q, _dq);
}

void RobotModel::ComputeCoriolisMatrix(Eigen::VectorXd q_now, Eigen::VectorXd v_now)
{
    _q = q_now;
    _dq = v_now;
    pinocchio::computeCoriolisMatrix(model, data, _q, _dq);
}

void RobotModel::ComputeGeneralizedGravity(Eigen::VectorXd q_now)
{
    _q = q_now;
    pinocchio::computeGeneralizedGravity(model, data, _q);
}

Eigen::Vector3d RobotModel::getJointTranslation_LOCAL(Eigen::VectorXd q_now, int JOINT_ID){
    _q = q_now;
    pinocchio::forwardKinematics(model, data, _q);
    return (data.oMi[JOINT_ID].translation());
}

int RobotModel::GetNq()
{
    return model.nq;
}

int RobotModel::GetNv()
{
    return model.nv;
}