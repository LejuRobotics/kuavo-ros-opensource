// 必须在所有头文件之前包含 pinocchio/fwd.hpp
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/WaistKinematics.h"
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>  // 提供 log3 函数

namespace humanoid_controller {

void WaistKinematics::setPinocchioInterface(std::shared_ptr<PinocchioInterface> pinocchioInterfacePtr, const std::string& waistFrameName) 
{
    pinocchio_interface_ptr_ = pinocchioInterfacePtr;
    if(!pinocchio_interface_ptr_)
    {
        ROS_ERROR("[WaistKinematics] Pinocchio interface not available! Returning empty result.");
        return;
    }
    
    const auto& model = pinocchio_interface_ptr_->getModel();

    // 打印关节数量和维度信息
    ROS_INFO_STREAM("\033[32m" << "Model dimensions:" << "\033[0m");
    ROS_INFO_STREAM("\033[32m" << " - Number of joints (njoints): " << model.njoints << "\033[0m");
    ROS_INFO_STREAM("\033[32m" << " - Configuration vector size (nq): " << model.nq << "\033[0m");
    ROS_INFO_STREAM("\033[32m" << " - Velocity vector size (nv): " << model.nv << "\033[0m");

    // 打印位置向量的组成
    ROS_INFO_STREAM("\033[32m" << "\nConfiguration vector (q) components:" << "\033[0m");
    size_t q_idx = 0;
    for (size_t i = 0; i < model.njoints; ++i) {
        const auto& joint = model.joints[i];
        ROS_INFO_STREAM("\033[32m" << " - Joint '" << model.names[i] << "': " 
                       << joint.nq() << " DOF, q[" << q_idx << ":" 
                       << q_idx + joint.nq() - 1 << "]" << "\033[0m");
        q_idx += joint.nq();
    }

    // 打印速度向量的组成
    ROS_INFO_STREAM("\033[32m" << "\nVelocity vector (v) components:" << "\033[0m");
    size_t v_idx = 0;
    for (size_t i = 0; i < model.njoints; ++i) {
        const auto& joint = model.joints[i];
        ROS_INFO_STREAM("\033[32m" << " - Joint '" << model.names[i] << "': " 
                       << joint.nv() << " DOF, v[" << v_idx << ":" 
                       << v_idx + joint.nv() - 1 << "]" << "\033[0m");
        v_idx += joint.nv();
    }

    ROS_INFO_STREAM("\033[32m" << "\nAvailable joints in model with limits:" << "\033[0m");
    for (size_t i = 1; i < model.njoints; ++i) 
    {  
        // 从1开始，跳过universe/root关节
        ROS_INFO_STREAM("\033[32m" << " - Joint " << i << ": " << model.names[i] 
                       << " [" << model.lowerPositionLimit[i] << ", " 
                       << model.upperPositionLimit[i] << "]" << "\033[0m");
    }
    // 查找腰部frame ID
    if (model.existFrame(waistFrameName)) 
    {
        waist_frame_id_ = model.getFrameId(waistFrameName);
        use_pinocchio_ = true;
        ROS_INFO("[WaistKinematics] Pinocchio interface set successfully. Frame '%s' ID: %ld", 
                 waistFrameName.c_str(), waist_frame_id_);
    } 
    else 
    {
        use_pinocchio_ = false;
        ROS_ERROR("[WaistKinematics] Frame '%s' not found in Pinocchio model! Available frames:", 
                  waistFrameName.c_str());
        for (size_t i = 0; i < model.frames.size(); ++i) {
            ROS_ERROR("  [%zu] %s", i, model.frames[i].name.c_str());
        }
    }
    
}

vector_t WaistKinematics::computeWaistInverseKinematicsWithPinocchio(
    const vector_t& basePose,      // [x, y, z, qw, qx, qy, qz] - 只使用x, y和yaw (从四元数计算)
    const vector_t& targetPose,    // [x, y, z, qw, qx, qy, qz] - 目标空间位姿
    const vector_t& currentJoints, // 当前4个腰部关节角度 [knee, leg, waist_pitch, waist_yaw]
    bool is_check_error) 
{
    // 1. 参数检查
    if (!use_pinocchio_ || !pinocchio_interface_ptr_) {
        ROS_ERROR_THROTTLE(1.0, "[WaistKinematics] Pinocchio interface not available! Returning empty result.");
        return vector_t::Zero(4);
    }

    if (basePose.size() != 7 || targetPose.size() != 7 || currentJoints.size() != 4) {
        ROS_ERROR("[WaistKinematics] Invalid input dimensions: base=%ld, target=%ld, joints=%ld",
                  basePose.size(), targetPose.size(), currentJoints.size());
        return vector_t::Zero(4);
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    const auto& model = pinocchio_interface_ptr_->getModel();
    auto& data = pinocchio_interface_ptr_->getData();

    // 2. 构建完整的构型向量
    vector_t q = vector_t::Zero(model.nq);  // 21维
    
    // 设置base位姿（平面位姿：x, y, yaw）
    q[1] = basePose[0];  // x
    q[2] = basePose[1];  // y
    
    // 从四元数提取yaw角
    Eigen::Quaterniond quat(basePose[3], basePose[4], basePose[5], basePose[6]);
    Eigen::Matrix3d rot = quat.toRotationMatrix();
    q[3] = std::atan2(rot(1,0), rot(0,0));  // yaw = atan2(r21, r11)
    // 注：base只能平面运动，所以z和roll/pitch保持为0
    
    // 设置腰部运动学链的关节状态（从knee到waist_yaw）
    const int KNEE_START_IDX = 4;  // knee_joint的索引，q[4:7]对应knee,leg,waist_pitch,waist_yaw
    q.segment<4>(KNEE_START_IDX) = currentJoints;  // [knee, leg, waist_pitch, waist_yaw]

    // 3. 提取目标位姿
    vector3_t targetPos = targetPose.head<3>();
    quaternion_t targetQuat(targetPose(3), targetPose(4), targetPose(5), targetPose(6));
    targetQuat.normalize();

    // 4. 迭代参数
    const int MAX_ITER = 10;
    const double POS_TOL = 1e-3;
    const double ROT_TOL = 1e-3;
    const double LAMBDA = 0.1;  // 阻尼系数
    bool converged = false;

    // 5. 迭代求解
    for (int iter = 0; iter < MAX_ITER; ++iter) {
        // 5.1 计算当前位姿
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        const auto& oMf = data.oMf[waist_frame_id_];

        // 5.2 计算位置和姿态误差
        vector3_t posError = targetPos - oMf.translation();
        matrix3_t targetRot = targetQuat.toRotationMatrix();
        vector3_t rotError = pinocchio::log3(targetRot.transpose() * oMf.rotation());

        double posNorm = posError.norm();
        double rotNorm = rotError.norm();

        // 5.3 检查收敛
        if (posNorm < POS_TOL && rotNorm < ROT_TOL) {
            converged = true;
            if (iter > 0) {
                ROS_DEBUG("[WaistKinematics] Pinocchio IK converged in %d iterations", iter);
            }
            break;
        }

        // 5.4 计算雅可比矩阵
        pinocchio::computeJointJacobians(model, data, q);
        matrix_t J_full = matrix_t::Zero(6, model.nv);
        pinocchio::getFrameJacobian(model, data, waist_frame_id_, 
                                   pinocchio::LOCAL_WORLD_ALIGNED, J_full);

        // 5.5 提取腰部运动学链对应的雅可比子矩阵（从knee到waist_yaw）
        Eigen::Matrix<double, 6, 4> J_waist = J_full.block<6, 4>(0, KNEE_START_IDX);

        // 5.6 构造误差向量并求解
        vector_t error(6);
        error.head<3>() = posError;
        error.tail<3>() = rotError;

        // 阻尼最小二乘求解
        Eigen::Matrix<double, 4, 4> JtJ = J_waist.transpose() * J_waist;
        JtJ.diagonal() += LAMBDA * vector4_t::Ones();
        vector4_t deltaQ = JtJ.ldlt().solve(J_waist.transpose() * error);

        // 5.7 更新关节角并处理限位（从knee到waist_yaw）
    vector4_t new_q = q.segment<4>(KNEE_START_IDX) + deltaQ;

    //TODO 当前pinocchio加载的关节限位有问题
    // for (int i = 0; i < 4; ++i) {
    //     new_q[i] = std::clamp(new_q[i],
    //                          model.lowerPositionLimit(KNEE_START_IDX + i),
    //                          model.upperPositionLimit(KNEE_START_IDX + i));
    // }

    // 使用预定义的关节限位
    new_q[0] = std::clamp(new_q[0], Q0_MIN, Q0_MAX);   // leg_joint
    new_q[1] = std::clamp(new_q[1], Q1_MIN, Q1_MAX);   // waist_pitch_joint
    new_q[2] = std::clamp(new_q[2], Q2_MIN, Q2_MAX);   // waist_roll_joint
    new_q[3] = std::clamp(new_q[3], Q3_MIN, Q3_MAX);   // waist_yaw_joint
    
    q.segment<4>(KNEE_START_IDX) = new_q;
    }

    // 6. 结果检查和日志
    if (is_check_error) {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start_time);

        if (!converged) {
            ROS_WARN("[WaistKinematics] Pinocchio IK did not converge in %d iterations", MAX_ITER);
            return vector_t::Zero(4);
        }

        // 最终误差检查
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        
        const auto& oMf = data.oMf[waist_frame_id_];
        double posError = (targetPos - oMf.translation()).norm();
        double rotError = pinocchio::log3(targetQuat.toRotationMatrix().transpose() * 
                                        oMf.rotation()).norm();

        ROS_INFO("[WaistKinematics] IK solved - Time: %.3f ms, Pos Error: %.6f m, Rot Error: %.6f rad",
                 duration.count() / 1000.0, posError, rotError);
    }

    // 返回腰部运动学链的关节角度（从knee到waist_yaw）
    return q.segment<4>(KNEE_START_IDX);
}

void WaistKinematics::compareIKMethods(const vector_t& basePose, 
                                       const vector_t& targetPose, 
                                       const vector_t& currentJoints) {
    ROS_INFO("\n========== IK Methods Comparison ==========");

    // 1. 测试原始快速方法
    auto start_fast = std::chrono::high_resolution_clock::now();
    vector_t result_fast = computeFastWaistInverseKinematics(basePose, targetPose, currentJoints, false);
    auto end_fast = std::chrono::high_resolution_clock::now();
    auto duration_fast = std::chrono::duration_cast<std::chrono::microseconds>(end_fast - start_fast);

    // 计算快速方法的最终误差
    vector_t fk_fast = computeWaistForwardKinematics(basePose, result_fast);
    double pos_error_fast = (targetPose.head<3>() - fk_fast.head<3>()).norm();
    quaternion_t target_q(targetPose(3), targetPose(4), targetPose(5), targetPose(6));
    quaternion_t result_q_fast(fk_fast(3), fk_fast(4), fk_fast(5), fk_fast(6));
    matrix3_t targetRot = target_q.toRotationMatrix();
    matrix3_t resultRot_fast = result_q_fast.toRotationMatrix();
    double rot_error_fast = pinocchio::log3(targetRot.transpose() * resultRot_fast).norm();

    ROS_INFO("[Fast IK] Time: %.3f ms, Pos Error: %.6f m, Rot Error: %.6f rad",
             duration_fast.count() / 1000.0, pos_error_fast, rot_error_fast);

    // 2. 测试 Pinocchio 方法
    if (use_pinocchio_) {
        auto start_pin = std::chrono::high_resolution_clock::now();
        vector_t result_pin = computeWaistInverseKinematicsWithPinocchio(basePose, targetPose, currentJoints, false);
        auto end_pin = std::chrono::high_resolution_clock::now();
        auto duration_pin = std::chrono::duration_cast<std::chrono::microseconds>(end_pin - start_pin);

        // 使用自定义FK验证Pinocchio IK结果
        vector_t fk_pin = computeWaistForwardKinematics(basePose, result_pin);
        double pos_error_pin = (targetPose.head<3>() - fk_pin.head<3>()).norm();
        quaternion_t result_q_pin(fk_pin(3), fk_pin(4), fk_pin(5), fk_pin(6));
        matrix3_t resultRot_pin = result_q_pin.toRotationMatrix();
        double rot_error_pin = pinocchio::log3(targetRot.transpose() * resultRot_pin).norm();

        ROS_INFO("[Pinocchio IK] Time: %.3f ms, Pos Error: %.6f m, Rot Error: %.6f rad",
                 duration_pin.count() / 1000.0, pos_error_pin, rot_error_pin);

        // 3. 对比结果
        ROS_INFO("[Comparison] Speed ratio: %.2fx, Pos accuracy: %.2fx, Rot accuracy: %.2fx",
                 (double)duration_fast.count() / duration_pin.count(),
                 pos_error_fast / (pos_error_pin + 1e-10),
                 rot_error_fast / (rot_error_pin + 1e-10));
    } else {
        ROS_WARN("[Pinocchio IK] Not available for comparison");
    }

    ROS_INFO("===========================================\n");
}

} // namespace humanoid_controller

