#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

/*
 * Controller KDL
 * 
 * Note di progetto (commenti in italiano come richiesto):
 * - Questo header espone i controllori usati nel nodo principale e nell'action server.
 * - Punto 1b: è stata aggiunta l'azione di controllo in velocità con ottimizzazione
 *   nello spazio nullo per l'evitamento dei limiti di giunto (velocity_ctrl_null).
 */
class KDLController
{
public:
    KDLController(KDLRobot &_robot);

    // Controllo a dinamica inversa (non usato nell'homework, mantenuto per completezza)
    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    // Punto 1b: Controllo in velocità con proiezione nello spazio nullo
    // _p_des: posizione cartesiana desiderata dell'EE
    // _p_cur: posizione cartesiana corrente dell'EE
    // _lambda: fattore di scala per l'ottimizzazione nello spazio nullo (evitamento limiti)
    // _Kp: guadagno proporzionale sullo spazio dei task
    Eigen::VectorXd velocity_ctrl_null(const Eigen::Vector3d &_p_des,
                                      const Eigen::Vector3d &_p_cur,
                                      double _lambda,
                                      double _Kp);

    // Punto 2b: controllo visivo (look-at) basato su marker ArUco
    Eigen::VectorXd vision_ctrl(const Eigen::Vector3d &_object_in_camera,
                               const Eigen::Matrix<double, 6, Eigen::Dynamic> &_ee_jacobian,
                               const Eigen::Matrix3d &_R_base_camera,
                               const Eigen::Vector3d &_gain,
                               double _lambda);
    
    Eigen::VectorXd joint_limit_avoidance(double _lambda) const;

private:
    KDLRobot* robot_;
    
    
};

#endif
