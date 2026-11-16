#ifndef _ankle_solver_h_
#define _ankle_solver_h_

#include "Eigen/Dense"
#include "kuavo_solver/ankle/axis_offset_ankle_solver.h"
#include "kuavo_solver/ankle/fixed_axis_ankle_solver.h"
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace kuavo_solver {

enum AnkleSolverType {
    // kuavo
    ANKLE_SOLVER_TYPE_4GEN,
    ANKLE_SOLVER_TYPE_4GEN_PRO,
    ANKLE_SOLVER_TYPE_5GEN,
    // roban
    ANKLE_SOLVER_TYPE_S1GEN,
    ANKLE_SOLVER_TYPE_S2GEN,
    ANKLE_SOLVER_TYPE_S2GEN_2,
    ANKLE_SOLVER_TYPE_5GEN_2,
    ANKLE_SOLVER_TYPE_7GEN,

    ANKLE_SOLVER_TYPE_NONE,
};

struct AnkleSolverTypeEntry {
    const char* token;
    AnkleSolverType type;
};

// Single source of truth:
// add one row here when introducing a new AnkleSolverType token.
inline const std::vector<AnkleSolverTypeEntry>& GetAnkleSolverTypeTable() {
    static const std::vector<AnkleSolverTypeEntry> kTable = {
        {"4gen", AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN},
        {"4gen_pro", AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO},
        {"5gen", AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN},
        {"5gen_2", AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN_2},
        {"7gen", AnkleSolverType::ANKLE_SOLVER_TYPE_7GEN},
        {"s1gen", AnkleSolverType::ANKLE_SOLVER_TYPE_S1GEN},
        {"s2gen", AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN},
        {"s2gen_2", AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN_2},
        {"none", AnkleSolverType::ANKLE_SOLVER_TYPE_NONE},
    };
    return kTable;
}

inline AnkleSolverType AnkleSolverTypeFromString(const std::string& ankle_solver_type) {
    for (const auto& entry : GetAnkleSolverTypeTable()) {
        if (ankle_solver_type == entry.token) {
            return entry.type;
        }
    }
    throw std::invalid_argument("unknown ankle_solver_type token: " + ankle_solver_type);
}

inline const char* AnkleSolverTypeToString(AnkleSolverType ankle_solver_type) {
    for (const auto& entry : GetAnkleSolverTypeTable()) {
        if (ankle_solver_type == entry.type) {
            return entry.token;
        }
    }
    return "unknown";
}

inline bool IsAxisOffsetSolverType(AnkleSolverType ankle_solver_type) {
    return ankle_solver_type == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN_2 ||
           ankle_solver_type == AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN_2 ||
           ankle_solver_type == AnkleSolverType::ANKLE_SOLVER_TYPE_7GEN;
}

/** 定轴踝（37/43 维、S2 布局）：统一由 FixedAxisAnkleSolver 处理。 */
inline bool UsesFixedAxisAnkleSolver(AnkleSolverType ankle_solver_type) {
    return ankle_solver_type == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN ||
           ankle_solver_type == AnkleSolverType::ANKLE_SOLVER_TYPE_S1GEN ||
           ankle_solver_type == AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN ||
           ankle_solver_type == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO ||
           ankle_solver_type == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN;
}

inline FixedAxisAnkleLayout FixedAxisAnkleLayoutForType(AnkleSolverType ankle_solver_type) {
    using L = FixedAxisAnkleLayout;
    switch (ankle_solver_type) {
        case AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN:
        case AnkleSolverType::ANKLE_SOLVER_TYPE_S1GEN:
            return L::Legacy37;
        case AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN:
        case AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO:
            return L::Pro43;
        case AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN:
            return L::S2Legacy37;
        default:
            throw std::logic_error("FixedAxisAnkleLayoutForType: not a fixed-axis ankle solver type");
    }
}

class AnkleSolver
{
public:
    Eigen::VectorXd joint_to_motor_position(const Eigen::VectorXd& q);
    Eigen::VectorXd joint_to_motor_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);
    Eigen::VectorXd joint_to_motor_current(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t);
    Eigen::VectorXd motor_to_joint_position(const Eigen::VectorXd& p);
    Eigen::VectorXd motor_to_joint_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v);
    Eigen::VectorXd motor_to_joint_torque(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c);
    /** leg12：膝 3/9 走 knee_* 1D；踝关节在 leg 向量槽位 4,5 与 10,11（与 AxisOffset 历史 leg12_* 一致），
     *  左腿电机两支路在 leg 上为 4↔5 交叉写回。 */
    Eigen::VectorXd joint_to_motor_position_leg12(const Eigen::VectorXd& q12);
    Eigen::VectorXd motor_to_joint_position_leg12(const Eigen::VectorXd& p12);
    /** ankle_solver_type_token 的唯一来源应来自 kuavo.json；config_dir 由编译期宏 KUAVO_ANKLE_CONFIG_DIR 提供（缺失直接 throw）。 */
    void getconfig(const std::string& ankle_solver_type_token);

    ~AnkleSolver();

private:
    int N_ITER = 10;
    std::string ankle_solver_type_token_;
    std::unique_ptr<AxisOffsetAnkleSolver> axis_offset_solver_;
    std::unique_ptr<FixedAxisAnkleSolver, FixedAxisAnkleSolverDeleter> fixed_axis_solver_;
};

}  // namespace kuavo_solver

#endif
