#ifndef _ankle_solver_h_
#define _ankle_solver_h_

#include "Eigen/Dense"
#include <iostream>
#include <memory>

namespace kuavo_solver {
    class Roban_ankle_solver;
    
    struct Roban_ankle_solver_deleter {
        void operator()(Roban_ankle_solver* p) const;
    };
}

enum AnkleSolverType {
    ANKLE_SOLVER_TYPE_4GEN = 0,
    ANKLE_SOLVER_TYPE_4GEN_PRO = 1,
    ANKLE_SOLVER_TYPE_5GEN = 2,
    ANKLE_SOLVER_TYPE_S1GEN = 3, 
    ANKLE_SOLVER_TYPE_S2GEN = 4,
    ANKLE_SOLVER_TYPE_S2GEN_2 = 5,
    ANKLE_SOLVER_TYPE_NONE = -1,
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

    Eigen::VectorXd joint_to_motor_position_s2_(const Eigen::VectorXd& q);
    Eigen::VectorXd joint_to_motor_velocity_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);
    Eigen::VectorXd joint_to_motor_current_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t);
    Eigen::VectorXd motor_to_joint_position_s2_(const Eigen::VectorXd& p);
    Eigen::VectorXd motor_to_joint_velocity_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v);
    Eigen::VectorXd motor_to_joint_torque_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c);

    Eigen::VectorXd joint_to_motor_position_s2_2_(const Eigen::VectorXd& q);
    Eigen::VectorXd joint_to_motor_velocity_s2_2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);
    Eigen::VectorXd joint_to_motor_current_s2_2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t);
    Eigen::VectorXd motor_to_joint_position_s2_2_(const Eigen::VectorXd& p);
    Eigen::VectorXd motor_to_joint_velocity_s2_2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v);
    Eigen::VectorXd motor_to_joint_torque_s2_2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c);
    void getconfig(const int ankle_solver_type);
    AnkleSolverType getAnkleSolverType(){
        return  static_cast<AnkleSolverType>(ankle_solver_type_);
    };
    
    ~AnkleSolver();
    
private:

    void applyRollLimitBasedOnPitch(Eigen::VectorXd& joint_q);
private:
    Eigen::VectorXd config;
    int N_ITER = 10;
    int ankle_solver_type_ = AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN;
    Eigen::Vector2d ankle_pitch_limits_;
    Eigen::Vector2d ankle_roll_limits_;
    std::unique_ptr<kuavo_solver::Roban_ankle_solver, kuavo_solver::Roban_ankle_solver_deleter> roban_solver_;

};
#endif
