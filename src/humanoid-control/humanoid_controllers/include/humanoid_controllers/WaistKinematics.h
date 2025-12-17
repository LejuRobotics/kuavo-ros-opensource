#pragma once

#include <Eigen/Dense>
#include "humanoid_interface/common/Types.h"
#include <ros/ros.h>
#include <chrono>
#include <memory>

// 只包含 PinocchioInterface 类型定义，不包含算法头文件，避免 Boost 冲突
namespace ocs2 {
    template <typename SCALAR> class PinocchioInterfaceTpl;
    using PinocchioInterface = PinocchioInterfaceTpl<double>;
}

namespace humanoid_controller 
{

using namespace ocs2;
using vector3_t = Eigen::Vector3d;
using vector4_t = Eigen::Vector4d;
using matrix3_t = Eigen::Matrix3d;
using quaternion_t = Eigen::Quaterniond;

class WaistKinematics {
public:
    WaistKinematics() = default;
    ~WaistKinematics() = default;

    /**
     * @brief 设置 Pinocchio 接口指针，用于基于模型的IK计算
     * @param pinocchioInterfacePtr Pinocchio 接口的共享指针
     * @param waistFrameName 腰部末端frame名称（默认为 "waist_yaw_link"）
     */
    void setPinocchioInterface(std::shared_ptr<PinocchioInterface> pinocchioInterfacePtr,
                               const std::string& waistFrameName = "waist_yaw_link");

    /**
     * @brief 使用 Pinocchio 计算腰部逆运动学（新方法）
     * @param basePose 基座位姿 [x,y,z,qw,qx,qy,qz]
     * @param targetPose 目标位姿 [x,y,z,qw,qx,qy,qz]
     * @param currentJoints 当前关节角度（完整的机器人状态）
     * @param is_check_error 是否检查误差
     * @return 关节角度解 [q0,q1,q2,q3]
     */
    vector_t computeWaistInverseKinematicsWithPinocchio(const vector_t& basePose, 
                                                         const vector_t& targetPose, 
                                                         const vector_t& currentJoints,
                                                         bool is_check_error = false);

    /**
     * @brief 对比两种IK方法的性能和精度
     * @param basePose 基座位姿
     * @param targetPose 目标位姿
     * @param currentJoints 当前关节角度
     */
    void compareIKMethods(const vector_t& basePose, 
                         const vector_t& targetPose, 
                         const vector_t& currentJoints);
    
    /**
     * @brief 计算腰部逆运动学
     * @param basePose 基座位姿 [x,y,z,qw,qx,qy,qz]
     * @param targetPose 目标位姿 [x,y,z,qw,qx,qy,qz]
     * @param currentJoints 当前关节角度
     * @param is_check_error 是否检查误差
     * @return 关节角度解 [q0,q1,q2,q3]
     */
    vector_t computeFastWaistInverseKinematics(const vector_t& basePose, 
                                                     const vector_t& targetPose, 
                                                     const vector_t& currentJoints, 
                                                     bool is_check_error = false) {
        // 初始化关节角（使用当前关节角作为初始解）
        vector_t q = currentJoints.head(4);
        if (q.size() < 4) q = vector_t::Zero(4);
        
        // 迭代参数（保证速度的关键）
        const int MAX_ITER = 10;          // 最大迭代次数
        const double POS_TOL = 1e-3;      // 位置收敛容差
        const double ROT_TOL = 1e-3;      // 姿态收敛容差
        const double LAMBDA = 0.1;        // 阻尼系数
        
        // 提取目标位姿（与正解输出格式严格一致）
        vector3_t targetPos = targetPose.head<3>();
        quaternion_t targetQuat(targetPose(3), targetPose(4), targetPose(5), targetPose(6));
        targetQuat.normalize();  // 确保单位四元数
        
        // 存储当前位姿的变量
        vector_t fkResult;
        vector3_t currentPos;
        quaternion_t currentQuat;
        
        // 迭代求解
        for (int iter = 0; iter < MAX_ITER; ++iter) {
            // 计算当前正解（直接使用你的正解函数，保证一致性）
            fkResult = computeWaistForwardKinematics(basePose, q);
            currentPos = fkResult.head<3>();
            currentQuat = quaternion_t(fkResult(3), fkResult(4), fkResult(5), fkResult(6));
            currentQuat.normalize();
            
            // 计算位置误差
            vector3_t posError = targetPos - currentPos;
            double posNorm = posError.norm();
            
            // 计算姿态误差（四元数处理与正解完全匹配）
            quaternion_t errorQuat = targetQuat * currentQuat.inverse();
            errorQuat.normalize();
            
            // 确保最短路径（处理四元数双重覆盖性）
            if (errorQuat.w() < 0) {
                errorQuat.coeffs() *= -1;
            }
            
            // 转换为旋转向量误差（与正解旋转矩阵转换逻辑一致）
            vector3_t rotError;
            if (fabs(errorQuat.w()) < 1.0) {
                double theta = 2.0 * acos(errorQuat.w());
                double s = sin(theta / 2.0);
                if (s > 1e-6) {
                    rotError = theta * errorQuat.vec() / s;
                } else {
                    rotError = 2.0 * errorQuat.vec();  // 小角度近似
                }
            } else {
                rotError = vector3_t::Zero();  // 无旋转误差
            }
            double rotNorm = rotError.norm();
            
            // 检查收敛
            if (posNorm < POS_TOL && rotNorm < ROT_TOL) {
                break;
            }
            
            // 计算雅可比矩阵（解析解，与正解几何参数一致）
            Eigen::Matrix<double, 6, 4> J = computeJacobian(basePose, q);
            
            // 构造误差向量（位置 + 姿态）
            vector_t error(6);
            error.head<3>() = posError;
            error.tail<3>() = rotError;
            
            // 阻尼最小二乘求解
            Eigen::Matrix<double, 4, 4> JtJ = J.transpose() * J;
            JtJ.diagonal() += LAMBDA * vector4_t::Ones();  // 添加阻尼
            
            // 快速求解线性方程组（LDLT分解适合对称正定矩阵）
            vector_t deltaQ = JtJ.ldlt().solve(J.transpose() * error);
            
            // 更新关节角
            q += deltaQ;
            
            // 关节限位处理
            q(0) = std::max(Q0_MIN, std::min(Q0_MAX, q(0)));
            q(1) = std::max(Q1_MIN, std::min(Q1_MAX, q(1)));
            q(2) = std::max(Q2_MIN, std::min(Q2_MAX, q(2)));
            q(3) = std::max(Q3_MIN, std::min(Q3_MAX, q(3)));
        }
        
        if(is_check_error) {
            ROS_INFO("check ik error.");
            // 最后一次迭代的误差检查
            fkResult = computeWaistForwardKinematics(basePose, q);
            currentPos = fkResult.head<3>();
            currentQuat = quaternion_t(fkResult(3), fkResult(4), fkResult(5), fkResult(6));
            currentQuat.normalize();
            
            // 计算位置误差
            double posError = (targetPos - currentPos).norm();
            
            // 计算姿态误差（使用四元数点积，更简单且数值稳定）
            double dotProduct = std::abs(targetQuat.w() * currentQuat.w() + 
                                     targetQuat.x() * currentQuat.x() + 
                                     targetQuat.y() * currentQuat.y() + 
                                     targetQuat.z() * currentQuat.z());
            double rotError = 2.0 * acos(std::min(1.0, dotProduct));
            
            // 检查是否收敛，如果没收敛返回全0向量
            if (posError >= POS_TOL || rotError >= ROT_TOL) {
                ROS_WARN("IK did not converge: pos_error=%.6f (tol=%.6f), rot_error=%.6f (tol=%.6f)", 
                       posError, POS_TOL, rotError, ROT_TOL);
                return vector_t::Zero(4);
            }
        }
        return q;
    }

    /**
     * @brief 计算腰部正运动学
     * @param basePose 基座位姿 [x,y,z,qw,qx,qy,qz]
     * @param jointAngles 关节角度 [q0,q1,q2,q3]
     * @return 末端位姿 [x,y,z,qw,qx,qy,qz]
     */
    vector_t computeWaistForwardKinematics(const vector_t& basePose, 
                                                 const vector_t& jointAngles) {
        // 返回7维vector_t：前3个元素是位置(x,y,z)，后4个元素是四元数(w,x,y,z)
        vector_t result = vector_t::Zero(7);
        
        // 从basePose中提取基座位置和姿态
        vector3_t basePosition = basePose.head<3>();
        double w = basePose(3);
        double x = basePose(4);
        double y = basePose(5);
        double z = basePose(6);
        quaternion_t baseOrientation(w, x, y, z);  // 正确：参数顺序(w,x,y,z)
        
        // 基座旋转矩阵
        matrix3_t baseRotation = baseOrientation.toRotationMatrix();
        
        // 1. 计算knee_joint位置（关节1）
        vector3_t kneeLocal(0, 0, KNEE_OFFSET_Z);
        vector3_t kneeGlobal = basePosition + baseRotation * kneeLocal;
        
        // 2. 计算leg_joint位置（关节2）
        // 关节1的旋转：knee_joint
        matrix3_t kneeRotation = rotationMatrixFromRPY(0, jointAngles[0], 0);
        vector3_t legLocal(LEG_OFFSET_X, 0, LEG_OFFSET_Z);
        vector3_t legGlobal = kneeGlobal + baseRotation * kneeRotation * legLocal;
        
        // 3. 计算waist_pitch_joint位置（关节3）
        // 关节2的旋转：leg_joint
        matrix3_t legRotation = rotationMatrixFromRPY(0, jointAngles[1], 0);
        vector3_t waistLocal(WAIST_OFFSET_X, 0, WAIST_OFFSET_Z);
        vector3_t waistGlobal = legGlobal + baseRotation * kneeRotation * legRotation * waistLocal;
        
        // 4. 计算waist_yaw_link位置（关节4）
        // 关节3的旋转：waist_pitch_joint
        matrix3_t waistPitchRotation = rotationMatrixFromRPY(0, jointAngles[2], 0);
        vector3_t waistYawLocal(WAIST_YAW_OFFSET_X, WAIST_YAW_OFFSET_Y, WAIST_YAW_OFFSET_Z);
        vector3_t waistYawGlobal = waistGlobal + baseRotation * kneeRotation * legRotation * waistPitchRotation * waistYawLocal;
        
        // 5. 计算waist_yaw_link的姿态
        // 组合所有关节的旋转
        matrix3_t totalRotation = baseRotation * kneeRotation * legRotation * waistPitchRotation;
        
        // 添加关节4的旋转：waist_yaw_joint
        matrix3_t waistYawRotation = rotationMatrixFromRPY(0, 0, jointAngles[3]);
        totalRotation = totalRotation * waistYawRotation;
        
        // 转换为四元数
        quaternion_t waistOrientation = rotationMatrixToQuaternion(totalRotation);
        
        // 填充结果vector_t：前3个是位置，后4个是四元数 [x, y, z, w, x, y, z]
        result.head<3>() = waistYawGlobal;
        result.segment<4>(3) << waistOrientation.w(), waistOrientation.x(), waistOrientation.y(), waistOrientation.z();
        
        return result;
    }

    /**
     * @brief 计算雅可比矩阵
     * @param basePose 基座位姿 [x,y,z,qw,qx,qy,qz]
     * @param jointAngles 关节角度 [q0,q1,q2,q3]
     * @return 6x4的雅可比矩阵
     */
    Eigen::Matrix<double, 6, 4> computeJacobian(const vector_t& basePose, 
                                                      const vector_t& jointAngles) {
        Eigen::Matrix<double, 6, 4> J = Eigen::Matrix<double, 6, 4>::Zero();
        
        // 提取关节角
        double q0 = jointAngles(0);
        double q1 = jointAngles(1);
        double q2 = jointAngles(2);
        double q3 = jointAngles(3);
        
        // 从basePose获取基座信息（与正解处理一致）
        vector3_t basePosition = basePose.head<3>();
        quaternion_t baseOrientation(basePose(3), basePose(4), basePose(5), basePose(6));
        baseOrientation.normalize();
        matrix3_t baseRotation = baseOrientation.toRotationMatrix();
        
        // 计算各关节旋转矩阵（与正解旋转顺序一致）
        matrix3_t kneeRotation = rotationMatrixFromRPY(0, q0, 0);
        matrix3_t legRotation = rotationMatrixFromRPY(0, q1, 0);
        matrix3_t waistPitchRotation = rotationMatrixFromRPY(0, q2, 0);
        
        // 计算各关节位置（全局坐标系）
        vector3_t kneeLocal(0, 0, KNEE_OFFSET_Z);
        vector3_t kneeGlobal = basePosition + baseRotation * kneeLocal;
        
        vector3_t legLocal(LEG_OFFSET_X, 0, LEG_OFFSET_Z);
        vector3_t legGlobal = kneeGlobal + baseRotation * kneeRotation * legLocal;
        
        vector3_t waistLocal(WAIST_OFFSET_X, 0, WAIST_OFFSET_Z);
        vector3_t waistGlobal = legGlobal + baseRotation * kneeRotation * legRotation * waistLocal;
        
        vector3_t waistYawLocal(WAIST_YAW_OFFSET_X, WAIST_YAW_OFFSET_Y, WAIST_YAW_OFFSET_Z);
        vector3_t endEffector = waistGlobal + baseRotation * kneeRotation * legRotation * waistPitchRotation * waistYawLocal;
        
        // 关节轴（全局坐标系，与正解旋转轴一致）
        vector3_t yAxis(0, 1, 0);  // knee, leg, waist_pitch关节轴
        vector3_t zAxis(0, 0, 1);  // waist_yaw关节轴
        
        vector3_t axis0 = baseRotation * yAxis;  // q0轴
        vector3_t axis1 = baseRotation * kneeRotation * yAxis;  // q1轴
        vector3_t axis2 = baseRotation * kneeRotation * legRotation * yAxis;  // q2轴
        vector3_t axis3 = baseRotation * kneeRotation * legRotation * waistPitchRotation * zAxis;  // q3轴
        
        // 计算雅可比矩阵各列
        // 位置部分：轴 × (末端 - 关节位置)
        // 姿态部分：关节轴
        J.col(0).head<3>() = axis0.cross(endEffector - kneeGlobal);
        J.col(0).tail<3>() = axis0;
        
        J.col(1).head<3>() = axis1.cross(endEffector - legGlobal);
        J.col(1).tail<3>() = axis1;
        
        J.col(2).head<3>() = axis2.cross(endEffector - waistGlobal);
        J.col(2).tail<3>() = axis2;
        
        J.col(3).head<3>() = axis3.cross(endEffector - endEffector);  // q3绕自身旋转，位置雅可比为0
        J.col(3).tail<3>() = axis3;
        
        return J;
    }

    /**
     * @brief 将相对位姿变换应用到当前位姿上
     * @param currentPose 当前位姿 [x,y,z,qw,qx,qy,qz]
     * @param relativePose 相对位姿变换 [x,y,z,qw,qx,qy,qz]
     * @return 变换后的位姿 [x,y,z,qw,qx,qy,qz]
     */
    vector_t transformPoseWithRelativeOffset(const vector_t& currentPose, 
                                                  const vector_t& relativePose) {
        // 将基于躯干的相对变化量应用到当前躯干位姿上
        // 1. 位置变化：在躯干坐标系下进行旋转，然后加到当前位置
        vector3_t current_pos = currentPose.head<3>();
        quaternion_t current_quat(currentPose(3), currentPose(4), currentPose(5), currentPose(6));
        current_quat.normalize();
        matrix3_t current_rotation = current_quat.toRotationMatrix();
        
        // 将vr_torso_pose的位置变化量从躯干坐标系转换到世界坐标系
        vector3_t pos_delta(relativePose[0], relativePose[1], relativePose[2]);
        vector3_t world_pos_delta = current_rotation * pos_delta;
        
        // 计算变化后的躯干位置
        vector3_t new_pos = current_pos + world_pos_delta;
        
        // 2. 姿态变化：将vr_torso_pose的姿态变化量应用到当前姿态
        quaternion_t quat_delta(relativePose[3], relativePose[4], relativePose[5], relativePose[6]);
        quat_delta.normalize();
        
        // 组合姿态变化：新姿态 = 当前姿态 * 变化量
        quaternion_t new_quat = current_quat * quat_delta;
        new_quat.normalize();
        
        // 构建最终的目标位姿
        vector_t target_pose = vector_t::Zero(7);
        target_pose.head<3>() = new_pos;
        target_pose.segment<4>(3) << new_quat.w(), new_quat.x(), new_quat.y(), new_quat.z();
        
        return target_pose;
    }

private:
    // Pinocchio 相关成员变量
    std::shared_ptr<PinocchioInterface> pinocchio_interface_ptr_;
    size_t waist_frame_id_ = 0;
    bool use_pinocchio_ = false;

    /**
     * @brief 从RPY角度计算旋转矩阵
     */
    matrix3_t rotationMatrixFromRPY(double roll, double pitch, double yaw) {
        // 从RPY角度计算旋转矩阵
        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        
        double cr = cos(roll), sr = sin(roll);
        double cp = cos(pitch), sp = sin(pitch);
        double cy = cos(yaw), sy = sin(yaw);
        
        matrix3_t R;
        R << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
             sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
             -sp, cp*sr, cp*cr;
        
        return R;
    }

    /**
     * @brief 从旋转矩阵转换为四元数
     */
    quaternion_t rotationMatrixToQuaternion(const matrix3_t& R) {
        // 从旋转矩阵转换为四元数
        quaternion_t q;
        q = R;  // Eigen提供了从旋转矩阵到四元数的直接转换
        return q;
    }

    // 机器人几何参数（从URDF中提取）
    static constexpr double KNEE_OFFSET_Z = 0.0805;
    static constexpr double LEG_OFFSET_X = -0.48618;
    static constexpr double LEG_OFFSET_Z = 0.11672;
    static constexpr double WAIST_OFFSET_X = 0.49384;
    static constexpr double WAIST_OFFSET_Z = 0.078217;
    static constexpr double WAIST_YAW_OFFSET_X = 0.0;
    static constexpr double WAIST_YAW_OFFSET_Y = -0.0165;
    static constexpr double WAIST_YAW_OFFSET_Z = 0.209;

    // 关节限位参数
    static constexpr double Q0_MIN = 0.0;
    static constexpr double Q0_MAX = 1.448;
    static constexpr double Q1_MIN = -2.81;
    static constexpr double Q1_MAX = 0.0;
    static constexpr double Q2_MIN = -0.157;
    static constexpr double Q2_MAX = 2.984;
    static constexpr double Q3_MIN = -M_PI;
    static constexpr double Q3_MAX = M_PI;
};

} // namespace humanoid_controller
