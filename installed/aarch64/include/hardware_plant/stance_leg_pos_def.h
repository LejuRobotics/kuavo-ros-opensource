#ifndef STANCE_POS_DEF_H
#define STANCE_POS_DEF_H
#include<array>
namespace HighlyDynamic {

/* 以下这些变量用于在只控制上半身模式下, 下肢始终返回站立时的关节值, 解决手臂抽搐问题.
    注意: 在只控制上半身模式下, 下肢的电机是相当于未使能状态, 因此 kStanceLegJointPos并非其真实值.
    QA: https://www.lejuhub.com/highlydynamic/kuavodevlab/-/issues/819
    */

std::array<double, 12> kStanceLegJointPos_Robot40 = { 
    -0.96, -0.07, -30.60, 45.98, -18.40, 0.97,
    1.12, 0.08, -30.60, 45.99, -18.40, -1.19
};
std::array<double, 12> kStanceLegJointPos_Robot41 = {
    1.19, 0.06, -23.30, 42.11, -21.75, -1.18, 
    -0.97, -0.05, -23.31, 42.16, -21.80, 0.97
};

std::array<double, 12> kStanceLegJointPos_Robot42 = {
    1.53, -1.195, -27.08, 48.55, -24.50, -1.165, 
    -0.676, 1.34, -25.88, 47.95, -25.10, 0.91
};

std::array<double, 12> kStanceLegJointPos_Robot43 = {
    -0.96, -0.05, -30.43, 46.01, -18.60, 0.97, 
    1.12, 0.06, -30.432, 46.02, -18.60, -1.12
};

std::array<double, 12> kStanceLegJointPos_Robot45 = {
  1.14, 0.06, -24.72, 45.99, -24.20, -1.14, 
  -1.04, -0.06, -24.74, 46.03, -24.24, 1.04
};

} // namespace HighlyDynamic
#endif