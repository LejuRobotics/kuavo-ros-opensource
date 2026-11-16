#ifndef KUAVO_SOLVER_FIXED_AXIS_ANKLE_SOLVER_H_
#define KUAVO_SOLVER_FIXED_AXIS_ANKLE_SOLVER_H_

#include "Eigen/Dense"
#include <memory>
#include <string>
#include <utility>

namespace kuavo_solver {

enum class FixedAxisAnkleLayout {
    Legacy37,
    Pro43,
    S2Legacy37,
};

class FixedAxisAnkleSolver;

struct FixedAxisAnkleSolverDeleter {
    void operator()(FixedAxisAnkleSolver* p) const;
};

struct FixedAxisLoadedParam {
    FixedAxisAnkleLayout layout;
    Eigen::VectorXd config;
    /** S2GEN：joint_to_motor_position 对膝角不做 >=0 截断（与历史 AnkleSolver 行为一致） */
    bool bypass_knee_floor_for_joint_to_motor_position{false};
};

class FixedAxisAnkleSolver {
public:
    explicit FixedAxisAnkleSolver(FixedAxisAnkleLayout layout, Eigen::VectorXd config, int n_iter,
                                  bool bypass_knee_floor_for_joint_to_motor_position = false);

    /** 从 fixaxisanklesolver.yaml 读取并解析指定 token 的 variant（禁用 fallback；失败抛异常）。 */
    static FixedAxisLoadedParam loadFixedParam(const std::string& ankle_solver_type_token,
                                               const std::string& ankle_config_dir);

    Eigen::VectorXd joint_to_motor_position(const Eigen::VectorXd& q);
    Eigen::VectorXd motor_to_joint_position(const Eigen::VectorXd& p);
    Eigen::VectorXd joint_to_motor_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);
    Eigen::VectorXd motor_to_joint_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v);
    Eigen::VectorXd joint_to_motor_current(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t);
    Eigen::VectorXd motor_to_joint_torque(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c);

private:
    Eigen::VectorXd joint_to_motor_position_legacy37_(const Eigen::VectorXd& q);
    Eigen::VectorXd joint_to_motor_velocity_legacy37_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);
    Eigen::VectorXd joint_to_motor_current_legacy37_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t);
    Eigen::VectorXd motor_to_joint_position_legacy37_(const Eigen::VectorXd& p);
    Eigen::VectorXd motor_to_joint_velocity_legacy37_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v);
    Eigen::VectorXd motor_to_joint_torque_legacy37_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c);

    Eigen::VectorXd joint_to_motor_position_pro43_(const Eigen::VectorXd& q);
    Eigen::VectorXd joint_to_motor_velocity_pro43_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);
    Eigen::VectorXd joint_to_motor_current_pro43_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t);
    Eigen::VectorXd motor_to_joint_position_pro43_(const Eigen::VectorXd& p);
    Eigen::VectorXd motor_to_joint_velocity_pro43_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v);
    Eigen::VectorXd motor_to_joint_torque_pro43_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c);

    Eigen::VectorXd joint_to_motor_position_s2_(const Eigen::VectorXd& q);
    Eigen::VectorXd joint_to_motor_velocity_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);
    Eigen::VectorXd joint_to_motor_current_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t);
    Eigen::VectorXd motor_to_joint_position_s2_(const Eigen::VectorXd& p);
    Eigen::VectorXd motor_to_joint_velocity_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v);
    Eigen::VectorXd motor_to_joint_torque_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c);

    FixedAxisAnkleLayout layout_;
    Eigen::VectorXd config_;
    int n_iter_;
    /** S2GEN：joint_to_motor_position 对膝角不做 >=0 截断（与历史 AnkleSolver 行为一致） */
    bool bypass_knee_floor_for_joint_to_motor_position_{false};
};

} // namespace kuavo_solver

#endif // KUAVO_SOLVER_FIXED_AXIS_ANKLE_SOLVER_H_
