#ifndef __HEIMAN_HAND_HPP__
#define __HEIMAN_HAND_HPP__
#include <atomic>
#include <memory>
#include <string>
#include <array>
#include <map>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <mujoco/mujoco.h>
#include "../joint_address.hpp"
#include "mujoco_hand_base.hpp"

namespace mujoco_node {

// 黑漫 heiman SG100 五指灵巧手（11-DOF）：thumb 3 + index 3 + middle 2 + little 3。
// 命令/状态直接读写 11 个关节的 qpos/ctrl（弧度），不做曲度抽象。
// 关节顺序与 SG100HandCommand 的 left/right_hand_positions 11 维一致：
//   l_thumb_j1 j2 j3, l_index_j1 j2 j3, l_middle_j1 j2, l_little_j1 j2 j3
class HeimanHand : public MujocoHandBase {
public:
    static constexpr int JOINT_COUNT = 11;
    // 与真机 sg100_hand_driver.cpp loadSg100HardcodedDefaults() rest 一致（弧度）
    static constexpr std::array<double, JOINT_COUNT> kLeftRestPose{
        0.00, -1.0, 1.57, 0.0, 1.57, 1.57, 1.57, 1.57, 0.0, 1.57, 1.57};
    static constexpr std::array<double, JOINT_COUNT> kRightRestPose{
        0.00, -1.0, 1.57, 0.0, 1.57, 1.57, 1.57, 1.57, 0.0, 1.57, 1.57};

    HeimanHand(const mjModel* model, const JointGroupAddress& jga,
               const std::array<double, JOINT_COUNT>& rest = kLeftRestPose)
        : jga_(jga), target_positions_(rest) {
        if (!jga_.ctrladr().invalid()) {
            for (auto iter = jga_.ctrladr().begin(); iter != jga_.ctrladr().end(); ++iter) {
                auto actuator_id = *iter;
                if (model->actuator_ctrllimited[actuator_id]) {
                    double min_val = model->actuator_ctrlrange[2 * actuator_id];
                    double max_val = model->actuator_ctrlrange[2 * actuator_id + 1];
                    ctrllimited_map_[actuator_id] = std::array<double, 2>{min_val, max_val};
                }
            }
        }
    }

    // 读 11 个关节的 qpos（弧度）/ qvel / qfrc_actuator
    void readCallback(const mjData *d) override {
        if (jga_.qposadr().invalid()) return;
        int i = 0;
        auto iter_qpos = jga_.qposadr().begin();
        auto iter_qdof = jga_.qdofadr().begin();
        for (; iter_qpos != jga_.qposadr().end() && iter_qdof != jga_.qdofadr().end()
               && i < JOINT_COUNT; ++iter_qpos, ++iter_qdof, ++i) {
            joint_positions_[i] = d->qpos[*iter_qpos];
            joint_velocities_[i] = d->qvel[*iter_qdof];
            joint_torques_[i] = d->qfrc_actuator[*iter_qdof];
        }
    }

    // 与真机 sg100_hand_driver 一致：先用当前 qpos 做滤波初值，再以 50 Hz、alpha=0.2
    // 低通插值到 target（启动时为 rest）。仿真步长是 1 ms，不能每步都滤，否则会瞬间到位。
    void writeCallback(mjData *d) override {
        if (jga_.ctrladr().invalid() || jga_.qposadr().invalid()) return;

        constexpr double kFilterPeriod = 0.02;
        constexpr double kAlpha = 0.2;
        const bool reset = !filter_initialized_ || d->time + 1e-9 < last_filter_time_;
        if (reset) {
            int i = 0;
            for (auto iter = jga_.qposadr().begin();
                 iter != jga_.qposadr().end() && i < JOINT_COUNT; ++iter, ++i) {
                filtered_positions_[i] = d->qpos[*iter];
            }
            filter_initialized_ = true;
            last_filter_time_ = d->time - kFilterPeriod;
        }

        if (d->time - last_filter_time_ >= kFilterPeriod - 1e-9) {
            for (int i = 0; i < JOINT_COUNT; ++i) {
                filtered_positions_[i] = kAlpha * target_positions_[i]
                    + (1.0 - kAlpha) * filtered_positions_[i];
            }
            last_filter_time_ = d->time;
        }

        int i = 0;
        for (auto iter = jga_.ctrladr().begin(); iter != jga_.ctrladr().end() && i < JOINT_COUNT; ++iter, ++i) {
            d->ctrl[*iter] = filtered_positions_[i];
        }
    }

    // 设置 11 个关节目标位置（弧度），供 heiman 命令回调调用
    void setJointPositionsRadians(const std::array<double, JOINT_COUNT>& positions) {
        for (int i = 0; i < JOINT_COUNT; ++i) {
            target_positions_[i] = positions[i];
        }
        ctrl_updated_ = true;
    }

    // 读取 11 个关节当前弧度
    std::array<double, JOINT_COUNT> getJointPositions() const {
        return joint_positions_;
    }

    // —— 基类 6 维曲度接口：heiman 不使用，提供退化实现 ——
    void setFingerPositions(const UnsignedFingerArray &positions) override {
        (void)positions; // 不支持：heiman 走 setJointPositionsRadians
    }
    void setFingerSpeeds(const FingerArray &speeds) override {
        (void)speeds;    // 不支持
    }
    FingerStatusPtr getFingerStatus() override {
        return std::make_shared<FingerStatus>(finger_status_);
    }

private:
    JointGroupAddress jga_;
    std::map<int, std::array<double, 2>> ctrllimited_map_;
    FingerStatus finger_status_;   // 未使用，仅为满足基类接口
    std::atomic<bool> ctrl_updated_{false};
    std::array<double, JOINT_COUNT> target_positions_{};
    std::array<double, JOINT_COUNT> filtered_positions_{};
    bool filter_initialized_{false};
    double last_filter_time_{0.0};
    std::array<double, JOINT_COUNT> joint_positions_{};
    std::array<double, JOINT_COUNT> joint_velocities_{};
    std::array<double, JOINT_COUNT> joint_torques_{};
};

using HeimanHandPtr = std::shared_ptr<mujoco_node::HeimanHand>;

} // namespace mujoco_node
#endif
