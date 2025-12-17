#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
namespace leju_utils {

bool updateElbowPosition(const Eigen::Vector3d& shoulderPos,
                         double upperArmLinkLen,
                         double lowerArmLinkLen,
                         const Eigen::Vector3d& wristPos,
                         const Eigen::Vector3d& humanForearmDirection,
                         Eigen::Vector3d& elbowPos);

inline bool updateElbowPosition(const Eigen::Vector3d& shoulderPos,
                                double upperArmLinkLen,
                                double lowerArmLinkLen,
                                const Eigen::Vector3d& wristPos,
                                const Eigen::Vector3d& humanForearmDirection,
                                Eigen::Vector3d& elbowPos) {
  // 定义一个小的容差值，用于浮点数比较
  const double epsilon = 1e-6;

  // 步骤 1: 计算从肩膀到手腕的向量及其长度
  Eigen::Vector3d swVec = wristPos - shoulderPos;
  double swLen = swVec.norm();

  // 步骤 2: 检查目标可达性 (满足优先级1的前提)
  // 根据三角形不等式原理判断:
  // 1. 两边之和大于等于第三边
  // 2. 两边之差小于等于第三边
  if (swLen > upperArmLinkLen + lowerArmLinkLen + epsilon) {  // 手臂完全伸直也够不到
    std::cerr << "[updateElbowPosition] 目标不可达: 肩腕距离过远" << std::endl;
    std::cerr << "  肩腕距离 swLen: " << std::fixed << std::setprecision(6) << swLen << std::endl;
    std::cerr << "  上臂长度 upperArmLinkLen: " << upperArmLinkLen << std::endl;
    std::cerr << "  下臂长度 lowerArmLinkLen: " << lowerArmLinkLen << std::endl;
    std::cerr << "  最大可达距离 (upperArmLinkLen + lowerArmLinkLen): " << (upperArmLinkLen + lowerArmLinkLen)
              << std::endl;
    std::cerr << "  超出距离: " << (swLen - upperArmLinkLen - lowerArmLinkLen) << std::endl;
    return false;  // 目标不可达
  }
  if (swLen < std::abs(upperArmLinkLen - lowerArmLinkLen) - epsilon) {  // 目标太近，无法折叠到达
    std::cerr << "[updateElbowPosition] 目标不可达: 肩腕距离过近" << std::endl;
    std::cerr << "  肩腕距离 swLen: " << std::fixed << std::setprecision(6) << swLen << std::endl;
    std::cerr << "  上臂长度 upperArmLinkLen: " << upperArmLinkLen << std::endl;
    std::cerr << "  下臂长度 lowerArmLinkLen: " << lowerArmLinkLen << std::endl;
    std::cerr << "  最小可达距离 |upperArmLinkLen - lowerArmLinkLen|: " << std::abs(upperArmLinkLen - lowerArmLinkLen)
              << std::endl;
    std::cerr << "  不足距离: " << (std::abs(upperArmLinkLen - lowerArmLinkLen) - swLen) << std::endl;
    return false;  // 目标不可达
  }

  // 处理肩腕重合的特殊情况
  if (swLen < epsilon) {
    // 当肩腕重合时，意味着swLen=0。
    // 可达性检查保证了 |upperArmLinkLen - lowerArmLinkLen| 必须也约等于0。
    if (std::abs(upperArmLinkLen - lowerArmLinkLen) > epsilon) {
      std::cerr << "[updateElbowPosition] 目标不可达: 肩腕重合但上下臂长度不相等（物理上不可能的配置）" << std::endl;
      std::cerr << "  肩腕距离 swLen: " << std::fixed << std::setprecision(6) << swLen << std::endl;
      std::cerr << "  上臂长度 upperArmLinkLen: " << upperArmLinkLen << std::endl;
      std::cerr << "  下臂长度 lowerArmLinkLen: " << lowerArmLinkLen << std::endl;
      std::cerr << "  长度差 |upperArmLinkLen - lowerArmLinkLen|: " << std::abs(upperArmLinkLen - lowerArmLinkLen)
                << std::endl;
      std::cerr << "  肩膀位置: (" << shoulderPos.x() << ", " << shoulderPos.y() << ", " << shoulderPos.z() << ")"
                << std::endl;
      std::cerr << "  手腕位置: (" << wristPos.x() << ", " << wristPos.y() << ", " << wristPos.z() << ")" << std::endl;
      return false;  // 物理上不可能的配置
    }
    // 此时肘部的解是一个以wristPos为球心，lowerArmLinkLen为半径的球面。
    // 我们直接使用人体朝向来确定肘部位置。
    elbowPos = wristPos + humanForearmDirection.normalized() * lowerArmLinkLen;
    return true;
  }

  // 步骤 3: 计算肘部运动圆的几何属性
  // 这个圆是两个球面的交集，其平面法线是 shoulder-wrist 向量
  Eigen::Vector3d swDir = swVec.normalized();

  // 使用余弦定理的变形公式计算圆心在 shoulder-wrist 连线上的位置。
  // d 是圆心距离 shoulderPos 的投影长度。
  // d = (L_upper^2 - L_lower^2 + D_sw^2) / (2 * D_sw)
  double d = (upperArmLinkLen * upperArmLinkLen - lowerArmLinkLen * lowerArmLinkLen + swLen * swLen) / (2.0 * swLen);

  // 计算圆的半径
  double r_squared = upperArmLinkLen * upperArmLinkLen - d * d;
  // 由于浮点精度问题，r_squared可能为极小的负数，这里取max(0,...)来避免sqrt错误
  double r = std::sqrt(std::max(0.0, r_squared));

  // 计算圆心坐标
  Eigen::Vector3d circleCenter = shoulderPos + d * swDir;

  // 步骤 4: 在圆上找到满足优先级2的最佳点
  // 我们需要找到一个方向，使得(elbowPos - wristPos)与humanForearmDirection夹角最小。
  // 这可以通过将 humanForearmDirection 投影到圆所在平面来实现。

  // 将 humanForearmDirection 投影到垂直于 swDir 的平面上
  Eigen::Vector3d projDir = humanForearmDirection - humanForearmDirection.dot(swDir) * swDir;

  Eigen::Vector3d optimalDir;

  // 处理共线问题：如果投影向量长度过小，说明 humanForearmDirection 与 swDir 近乎平行
  if (projDir.norm() < epsilon) {
    // 在这种情况下，圆上任何一点都同样“好”，需要一个确定的备用策略。
    // 我们通过与世界坐标系的一个轴做叉乘，来生成一个与swDir垂直的稳定方向。
    Eigen::Vector3d worldAxis = Eigen::Vector3d::UnitZ();
    if (std::abs(swDir.dot(worldAxis)) > 0.99) {  // 如果 swDir 与 Z 轴近乎平行
      worldAxis = Eigen::Vector3d::UnitY();       // 则换用 Y 轴
    }
    optimalDir = swDir.cross(worldAxis).normalized();
  } else {
    // 正常情况下，使用归一化的投影向量作为最优方向
    optimalDir = projDir.normalized();
  }

  // 步骤 5: 计算最终的肘部位置
  // 最终位置 = 圆心 + 半径 * 最优方向
  elbowPos = circleCenter + r * optimalDir;

  return true;  // 成功计算
}

// ====================== 顶层调用接口声明 ======================
inline void fhanStepForward(double& x1,
                            double& x2,
                            const double& x1Ref,
                            const double& r,
                            const double& h,
                            const double& h0);

inline void fsunStepForward(double& x1,
                            double& x2,
                            const double& x1Ref,
                            const double& r,
                            const double& h,
                            const double& h0);

inline void fhanStepForwardWithVelLimit(double& x1,
                                        double& x2,
                                        const double& x1Ref,
                                        const double& r,
                                        const double& h,
                                        const double& h0,
                                        const double& maxVel);

// ====================== 核心加速度计算函数声明 ======================

inline double fhan(const double& x1, const double& x2, const double& r, const double& h0);

inline double fsun(const double& x1, const double& x2, const double& r, const double& h0);

// ====================== 底层基础函数声明 ======================
inline double fsg(const double& x, const double& d);
inline double sign(const double& x);
inline double sat(const double& x, const double& delta);
inline double sat(const double& x, const double& l, const double& u);

// ====================== 函数实现 ======================
inline void genFhanSmoothTrajectory(double& x1,
                                    double& x2,
                                    const double& x10,
                                    const double& x20,
                                    const double& x1Ref,
                                    const double r,
                                    const double h,
                                    const double h0) {
  double fh = fhan(x10 - x1Ref, x20, r, h0);
  x2 = x20 + h * fh;
  x1 = x10 + h * x20;
}

inline void fsunStepForward(double& x1,
                            double& x2,
                            const double& x1Ref,
                            const double& r,
                            const double& h,
                            const double& h0) {
  double fh = fsun((x1 - x1Ref), x2, r, h0);
  x1 = x1 + h * x2;
  x2 = x2 + h * fh;
}

inline void fhanStepForward(double& x1,
                            double& x2,
                            const double& x1Ref,
                            const double& r,
                            const double& h,
                            const double& h0) {
  double fh = fhan((x1 - x1Ref), x2, r, h0);
  x1 = x1 + h * x2;
  x2 = x2 + h * fh;
}

inline void fhanStepForwardWithVelLimit(double& x1,
                                        double& x2,
                                        const double& x1Ref,
                                        const double& r,
                                        const double& h,
                                        const double& h0,
                                        const double& maxVel) {
  // 保存滤波前的位置，用于速度限制后重新计算位置
  const double x1_prev = x1;

  // 使用fhanStepForward进行滤波，限制加速度
  double fh = fhan((x1 - x1Ref), x2, r, h0);
  x1 = x1 + h * x2;
  x2 = x2 + h * fh;

  // 限制速度：对滤波后的速度进行限幅
  x2 = sat(x2, maxVel);

  // 使用限制后的速度重新计算位置（覆盖fhanStepForward计算的位置）
  x1 = x1_prev + h * x2;
}

inline double fhan(const double& x1, const double& x2, const double& r, const double& h0) {
  double d = r * h0 * h0;
  double a0 = h0 * x2;
  double y = x1 + a0;
  double a1 = sqrt(d * (d + 8.0 * fabs(y)));
  double a2 = a0 + sign(y) * (a1 - d) / 2;
  double a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
  return (0 - r) * (a / d) * fsg(a, d) - r * sign(a) * (1 - fsg(a, d));
}

inline double fsun(const double& x1, const double& x2, const double& r, const double& h0) {
  double y = 2.0 * x1 + x2 * h0;
  double s = sign(y);
  double k0 = 0.5 * (sqrt(1 + 4.0 * fabs(y) / (h0 * h0 * r)) - 1);
  double k = floor(k0) + 1.0;
  return -1.0 * sat(x2 / h0 + 0.5 * (k - 1) * r * s + 0.5 * y / (2 * k * h0 * h0), r);
}

// ====================== 底层基础函数实现 ======================
inline double fsg(const double& x, const double& d) { return (sign(x + d) - sign(x - d)) / 2; }
inline double sign(const double& x) {  // 1e-6 为可修改项， 根据实际工程精度确定
  return (x - 1e-6 < 0 && x + 1e-6 > 0) ? 0.0 : (x >= 1e-6 ? 1.0 : -1.0);
}

inline double sat(const double& x, const double& delta) {
  return (x - delta < 0 && x + delta > 0) ? x : (x >= delta ? 1.0 * delta : -1.0 * delta);
}

inline double sat(const double& x, const double& l, const double& u) {
  double tmp = x <= l ? l : x;
  return (tmp >= u ? u : tmp);
}

}  // namespace leju_utils