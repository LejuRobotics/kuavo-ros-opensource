#ifndef _ankle_solver_h_
#define _ankle_solver_h_

#include "Eigen/Dense"
#include <iostream>
enum AnkleSolverType {
    ANKLE_SOLVER_TYPE_4GEN = 0,
    ANKLE_SOLVER_TYPE_4GEN_PRO = 1,
    ANKLE_SOLVER_TYPE_5GEN = 2
};
class AnkleSolver
{
public:
    Eigen::VectorXd joint_to_motor_position(const Eigen::VectorXd& q);
    Eigen::VectorXd joint_to_motor_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);
    Eigen::VectorXd joint_to_motor_current(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t);
    Eigen::VectorXd motor_to_joint_position(const Eigen::VectorXd& p);
    Eigen::VectorXd motor_to_joint_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v);
    Eigen::VectorXd motor_to_joint_torque(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c);

    Eigen::VectorXd joint_to_motor_position_(const Eigen::VectorXd& q);
    Eigen::VectorXd joint_to_motor_velocity_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);
    Eigen::VectorXd joint_to_motor_current_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t);
    Eigen::VectorXd motor_to_joint_position_(const Eigen::VectorXd& p);
    Eigen::VectorXd motor_to_joint_velocity_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v);
    Eigen::VectorXd motor_to_joint_torque_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c);

    Eigen::VectorXd joint_to_motor_position_pro_(const Eigen::VectorXd& q);
    Eigen::VectorXd joint_to_motor_velocity_pro_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);
    Eigen::VectorXd joint_to_motor_current_pro_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t);
    Eigen::VectorXd motor_to_joint_position_pro_(const Eigen::VectorXd& p);
    Eigen::VectorXd motor_to_joint_velocity_pro_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v);
    Eigen::VectorXd motor_to_joint_torque_pro_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c);
    void getconfig(const int robot_version);
    AnkleSolverType getAnkleSolverType(){
        return  static_cast<AnkleSolverType>(ankle_solver_type_);
    };
private:
    Eigen::VectorXd config;
    int N_ITER = 10;
    int ankle_solver_type_ = AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN;
    Eigen::Vector2d ankle_pitch_limits_;
    Eigen::Vector2d ankle_roll_limits_;

};
#endif
