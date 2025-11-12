#include "kdl_control.h"

#include <algorithm>

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // Lettura stato corrente (q, dq)
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // Errori
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity();
}

/*
 * Punto 1b: controllo in velocità con ottimizzazione nello spazio nullo
 * dq = J^\dagger Kp e_p + (I - J^\dagger J) dq0
 * dove dq0 deriva da un gradiente che penalizza l'avvicinamento ai limiti di giunto.
 */
Eigen::VectorXd KDLController::velocity_ctrl_null(const Eigen::Vector3d &_p_des,
                                                  const Eigen::Vector3d &_p_cur,
                                                  double _lambda,
                                                  double _Kp)
{
    // Errore di posizione cartesiana
    Eigen::Vector3d ep = _p_des - _p_cur;

    // Jacobiano 3xN (solo parte lineare) e sua pseudoinversa
    Eigen::MatrixXd J = robot_->getEEJacobian().data.topRows(3);
    Eigen::MatrixXd pinv_J = pseudoinverse(J);

    // Termine di task: inseguimento cartesiano
    Eigen::VectorXd dq_task = pinv_J * (_Kp * ep);

    // Termine di spazio nullo: repulsione dai limiti
    Eigen::VectorXd dq0 = joint_limit_avoidance(_lambda);

    // Proiezione nello spazio nullo e combinazione
    Eigen::MatrixXd null_proj = Eigen::MatrixXd::Identity(robot_->getNrJnts(), robot_->getNrJnts()) - pinv_J * J;
    return dq_task + null_proj * dq0;
}

Eigen::VectorXd KDLController::vision_ctrl(const Eigen::Vector3d &_object_in_camera,
                                           const Eigen::Matrix<double, 6, Eigen::Dynamic> &_ee_jacobian,
                                           const Eigen::Matrix3d &_R_base_camera,
                                           const Eigen::Vector3d &_gain,
                                           double _lambda)
{
    const unsigned int n = robot_->getNrJnts();
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(n);

    const double distance = _object_in_camera.norm();
    if (distance < 1e-6) {
        return dq;
    }

    // Direzione attuale s (asse dal centro camera all'oggetto) e direzione desiderata sd
    Eigen::Vector3d s = _object_in_camera / distance;          // s = P_c^o / ||P_c^o||
    const Eigen::Vector3d sd(0.0, 0.0, 1.0);                   // Asse desiderato (guardare lungo +Z camera)

    // Guadagni diagonali K (matrix 3x3)
    Eigen::Matrix3d K = _gain.asDiagonal();

    // Jacobiano espresso nel frame della camera
    Eigen::Matrix3d R_cb = _R_base_camera.transpose();
    Eigen::Matrix<double, 6, 6> rot_block;
    rot_block.setZero();
    rot_block.block<3,3>(0,0) = R_cb;
    rot_block.block<3,3>(3,3) = R_cb;
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_camera = rot_block * _ee_jacobian;

    // L(s): top-left = -(I - s s^T)/||P||  (derivata direzione rispetto velocità lineare)
    Eigen::Matrix3d linear_block = -(Eigen::Matrix3d::Identity() - s * s.transpose()) / distance;
    Eigen::Matrix3d angular_block = skew(s);

    Eigen::Matrix<double, 3, 6> L;
    L.block<3,3>(0,0) = linear_block;
    L.block<3,3>(0,3) = angular_block;

    Eigen::Matrix<double, 3, Eigen::Dynamic> task_jac = L * J_camera;      // L(s) J_c
    Eigen::MatrixXd pinv_task = pseudoinverse(task_jac);                   // (L J_c)^†

    // Homework formula: dq_task = (L J_c)^† K sd  (usa direttamente sd come target direzione)
    Eigen::VectorXd dq_task = pinv_task * (K * sd);

    Eigen::VectorXd dq0 = joint_limit_avoidance(_lambda);
    Eigen::MatrixXd null_proj = Eigen::MatrixXd::Identity(n, n) - pinv_task * task_jac;  // N = I - (LJ_c)^† (LJ_c)

    return dq_task + null_proj * dq0;
}

Eigen::VectorXd KDLController::joint_limit_avoidance(double _lambda) const
{
    Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(robot_->getNrJnts());
    double lambda = std::max(_lambda, 1e-3);
    int n = robot_->getNrJnts();
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::MatrixXd limits = robot_->getJntLimits();
    Eigen::VectorXd q_max = limits.col(1);
    Eigen::VectorXd q_min = limits.col(0);

    for (int i = 0; i < n; ++i) {
        double qi = q(i);
        double qi_max = q_max(i);
        double qi_min = q_min(i);
        double A = (qi_max - qi_min) * (qi_max - qi_min);
        double B = (qi_max - qi) * (qi - qi_min);
        double C = qi_max + qi_min - 2.0 * qi;

        dq0(i) = - (1.0 / lambda) * (A / (B * B)) * C;
    }

    return dq0;
}
