#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace ocs2 {
namespace mobile_manipulator {

/**
 * @class ContinuousEulerAnglesFromMatrix
 * @brief 使用旋转矩阵增量法计算连续无跳变的ZYX欧拉角
 * 
 * 这个类通过跟踪旋转矩阵的变化，计算增量欧拉角并累加，
 * 从而避免传统欧拉角在±π附近的跳变问题。
 * 特别适用于需要连续角度输出的机器人控制和动画应用。
 */
class ContinuousEulerAnglesFromMatrix {
private:
    Eigen::Matrix3d prev_rotation_;      ///< 上一时刻的旋转矩阵
    Eigen::Vector3d accumulated_euler_;   ///< 累加的欧拉角 [yaw, pitch, roll] (ZYX顺序)
    bool initialized_;                    ///< 是否已初始化

    /**
     * @brief 旋转矩阵转ZYX欧拉角
     * @param R 3x3旋转矩阵
     * @return ZYX欧拉角 [yaw, pitch, roll] (单位：弧度)
     * 
     * 注意：这个转换在pitch = ±π/2时存在奇异性（万向节锁）
     * 但在增量计算中通常不会遇到，因为增量通常很小
     */
    static Eigen::Vector3d rotationMatrixToZyx(const Eigen::Matrix3d& R) {
        // 计算yaw（绕Z轴旋转）
        double yaw = std::atan2(R(1,0), R(0,0));
        
        // 计算pitch（绕Y轴旋转）
        double pitch = std::atan2(-R(2,0), std::sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
        
        // 计算roll（绕X轴旋转）
        double roll = std::atan2(R(2,1), R(2,2));
        
        return Eigen::Vector3d(yaw, pitch, roll);
    }

    /**
     * @brief 规范化欧拉角到[-π, π]范围
     * @param euler 待规范化的欧拉角 [yaw, pitch, roll]
     * @return 规范化后的欧拉角 [yaw, pitch, roll]
     */
    static Eigen::Vector3d normalizeEulerAngles(const Eigen::Vector3d& euler) {
        Eigen::Vector3d normalized = euler;
        const double two_pi = 2.0 * M_PI;
        
        for (int i = 0; i < 3; i++) {
            // 使用fmod确保角度在[-π, π]范围内
            normalized(i) = std::fmod(normalized(i) + M_PI, two_pi);
            if (normalized(i) < 0) {
                normalized(i) += two_pi;
            }
            normalized(i) -= M_PI;
        }
        
        return normalized;
    }

public:
    /**
     * @brief 构造函数
     */
    ContinuousEulerAnglesFromMatrix() : initialized_(false) {
        accumulated_euler_ = Eigen::Vector3d::Zero();  // [yaw, pitch, roll]
        prev_rotation_ = Eigen::Matrix3d::Identity();
    }

    /**
     * @brief 从旋转矩阵更新无跳变的ZYX欧拉角
     * @param current_rotation 当前时刻的旋转矩阵
     * @return 连续无跳变的ZYX欧拉角 [yaw, pitch, roll]
     * 
     * 算法步骤：
     * 1. 如果是第一帧，直接转换并保存
     * 2. 计算相对于上一帧的旋转矩阵：ΔR = R_prev⁻¹ * R_curr
     * 3. 将ΔR转换为增量欧拉角
     * 4. 累加到总欧拉角中
     * 5. 规范化结果到[-π, π]范围
     */
    Eigen::Vector3d update(const Eigen::Matrix3d& current_rotation) {
        if (!initialized_) {
            // 第一帧：直接转换并保存
            accumulated_euler_ = rotationMatrixToZyx(current_rotation);
            prev_rotation_ = current_rotation;
            initialized_ = true;
            return accumulated_euler_;
        }
        
        // 计算相对旋转：ΔR = R_prev⁻¹ * R_curr
        Eigen::Matrix3d delta_rotation = prev_rotation_.transpose() * current_rotation;
        
        // 使用SVD确保ΔR是有效的旋转矩阵（防止数值误差）
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(delta_rotation, 
                                              Eigen::ComputeFullU | Eigen::ComputeFullV);
        delta_rotation = svd.matrixU() * svd.matrixV().transpose();
        
        // 将ΔR转换为增量欧拉角
        Eigen::Vector3d delta_euler = rotationMatrixToZyx(delta_rotation);
        
        // 累加增量角度
        accumulated_euler_ += delta_euler;
        
        // 规范化到[-π, π]范围
        // accumulated_euler_ = normalizeEulerAngles(accumulated_euler_);
        
        // 更新保存的旋转矩阵
        prev_rotation_ = current_rotation;
        
        return accumulated_euler_;
    }

    /**
     * @brief 从四元数更新无跳变的欧拉角
     * @param quat 当前时刻的四元数
     * @return 连续无跳变的欧拉角 [yaw, pitch, roll]
     */
    Eigen::Vector3d update(const Eigen::Quaterniond& quat) {
        return update(quat.toRotationMatrix());
    }

    /**
     * @brief 从四元数系数更新无跳变的欧拉角
     * @param qw, qx, qy, qz 四元数的四个分量
     * @return 连续无跳变的欧拉角 [yaw, pitch, roll]
     */
    Eigen::Vector3d update(double qw, double qx, double qy, double qz) {
        Eigen::Quaterniond quat(qw, qx, qy, qz);
        quat.normalize();  // 确保是单位四元数
        return update(quat);
    }

    /**
     * @brief 从ZYX欧拉角更新（用于重新同步或重置后的初始化）
     * @param euler_angles ZYX欧拉角 [yaw, pitch, roll]
     * 
     * 注意：这个函数假设输入的角度已经处理了连续性，直接设置内部状态
     */
    void updateFromEuler(const Eigen::Vector3d& euler_angles) {
        // 将欧拉角转换为旋转矩阵（ZYX顺序）
        Eigen::AngleAxisd rollAngle(euler_angles(2), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(euler_angles(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(euler_angles(0), Eigen::Vector3d::UnitZ());
        
        // ZYX顺序：先绕Z轴(yaw)，再绕Y轴(pitch)，最后绕X轴(roll)
        Eigen::Quaterniond quat = rollAngle * pitchAngle * yawAngle;
        Eigen::Matrix3d rotation = quat.toRotationMatrix();
        
        // 更新状态
        accumulated_euler_ = euler_angles;
        prev_rotation_ = rotation;
        initialized_ = true;
    }

    /**
     * @brief 获取当前累加的欧拉角（不更新内部状态）
     * @return 当前欧拉角 [yaw, pitch, roll]
     */
    Eigen::Vector3d getCurrentEulerAngles() const {
        return accumulated_euler_;
    }

    /**
     * @brief 获取上一帧的旋转矩阵
     * @return 上一帧的旋转矩阵
     */
    Eigen::Matrix3d getPreviousRotation() const {
        return prev_rotation_;
    }

    /**
     * @brief 检查是否已初始化
     * @return true如果已初始化，false否则
     */
    bool isInitialized() const {
        return initialized_;
    }

    /**
     * @brief 重置状态
     * 
     * 将内部状态重置为初始状态，可以重新开始跟踪
     */
    void reset() {
        initialized_ = false;
        accumulated_euler_ = Eigen::Vector3d::Zero();
        prev_rotation_ = Eigen::Matrix3d::Identity();
    }

    /**
     * @brief 重置并指定初始值
     * @param initial_rotation 初始旋转矩阵
     * @param initial_euler 初始欧拉角（可选，如果提供会用于一致性检查）
     */
    void reset(const Eigen::Matrix3d& initial_rotation, 
               const Eigen::Vector3d& initial_euler = Eigen::Vector3d::Zero()) {
        prev_rotation_ = initial_rotation;
        
        if (initial_euler.norm() > 1e-6) {
            // 如果提供了初始欧拉角，使用它
            // accumulated_euler_ = normalizeEulerAngles(initial_euler);
            accumulated_euler_ = initial_euler;
        } else {
            // 否则从旋转矩阵计算
            accumulated_euler_ = rotationMatrixToZyx(initial_rotation);
        }
        
        initialized_ = true;
    }

    /**
     * @brief 获取旋转矩阵形式的当前累计旋转
     * @return 累计的旋转矩阵
     */
    Eigen::Matrix3d getAccumulatedRotationMatrix() const {
        // 将累加的欧拉角转换回旋转矩阵（ZYX顺序）
        Eigen::AngleAxisd rollAngle(accumulated_euler_(2), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(accumulated_euler_(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(accumulated_euler_(0), Eigen::Vector3d::UnitZ());
        
        // ZYX顺序：先绕Z轴(yaw)，再绕Y轴(pitch)，最后绕X轴(roll)
        Eigen::Quaterniond quat = rollAngle * pitchAngle * yawAngle;
        return quat.toRotationMatrix();
    }

    /**
     * @brief 计算两帧之间的角度变化量
     * @param rotation1 第一帧的旋转矩阵
     * @param rotation2 第二帧的旋转矩阵
     * @return 从rotation1到rotation2的欧拉角变化量
     */
    static Eigen::Vector3d computeEulerAngleDifference(const Eigen::Matrix3d& rotation1,
                                                       const Eigen::Matrix3d& rotation2) {
        Eigen::Matrix3d delta_rotation = rotation1.transpose() * rotation2;
        return rotationMatrixToZyx(delta_rotation);
    }
};

}  // namespace mobile_manipulator
}  // namespace ocs2