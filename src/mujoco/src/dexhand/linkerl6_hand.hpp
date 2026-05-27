#ifndef __LINKERL6_HAND_HPP__
#define __LINKERL6_HAND_HPP__
#include <atomic>
#include <memory>
#include <string>
#include <array>
#include <map>
#include <vector>
#include <algorithm>
#include <iostream>
#include <mujoco/mujoco.h>
#include "../joint_address.hpp"
#include "mujoco_hand_base.hpp"
namespace mujoco_node {
// 新的LinkerL6灵巧手实现，范围0-255
class LinkerL6Hand : public MujocoHandBase {
public:
    LinkerL6Hand(const mjModel* model, const JointGroupAddress& jga) : jga_(jga) {
        if(!jga_.ctrladr().invalid()) {
            for (auto iter = jga_.ctrladr().begin(); iter != jga_.ctrladr().end(); ++iter) {
                auto actuator_id = *iter;
                if(model->actuator_ctrllimited[actuator_id]) {
                    double min_val = model->actuator_ctrlrange[2 * actuator_id];      // 最小值
                    double max_val = model->actuator_ctrlrange[2 * actuator_id + 1];  // 最大值
                    ctrllimited_map_[actuator_id] = std::array<double, 2>{min_val, max_val};
                }
            }
        }
    }
    /**
     * @brief read sensor data from mjData
     * @note callback for read thread.
     * @param d
     */
    void readCallback(const mjData *d) override {
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> tau;
        auto iter0 = jga_.qposadr().begin();
        auto iter1 = jga_.qdofadr().begin();
        for(; iter0 != jga_.qposadr().end() && iter1 != jga_.qdofadr().end(); ++iter0, ++iter1) {
            pos.push_back(d->qpos[*iter0]);
            vel.push_back(d->qvel[*iter1] * (180.0 / M_PI));
            tau.push_back(d->qfrc_actuator[*iter1] * 1000);
        }
        // thumb - LinkerL6手真实关节范围从URDF读取
        auto iter = jga_.ctrladr().begin();
        // pos[0]对应l_thumb_cmc_yaw关节，范围[-0.087266, 1.256637]弧度
        finger_status_.positions[1] = Joints2Curl(pos[0], {-0.087266, 1.256637}, pos[0], {-0.087266, 1.256637});
        finger_status_.speeds[1] = vel[0];
        finger_status_.currents[1] = tau[0];
        iter++;
        // pos[1]对应l_thumb_cmc_pitch关节，pos[2]对应l_thumb_ip关节，和其他手指一样合并计算曲度
        int actuator_pitch = *iter;
        ++iter;
        int actuator_dip = *iter;
        ++iter;
        finger_status_.positions[0] = Joints2Curl(pos[1], ctrllimited_map_[actuator_pitch], pos[2], ctrllimited_map_[actuator_dip]);
        finger_status_.speeds[0] = (vel[1] + vel[2]) / 2.0;
        finger_status_.currents[0] = (tau[1] + tau[2]) / 2.0;
        // index, middle, ring, little
        for (int i = 1; i < 5; i++) {
            auto joint0 = pos[i*2 + 1]; // 偏移1位，跳过新增的thumb_dip关节
            auto joint1 = pos[i*2 + 2];
            auto &range0 = ctrllimited_map_[*iter];
            iter ++;
            auto &range1 = ctrllimited_map_[*iter];
            iter ++;
            finger_status_.positions[i+1] = Joints2Curl(joint0, range0, joint1, range1);
            finger_status_.speeds[i+1] = (vel[i*2 + 1] + vel[i*2 + 2]) / 2.0;
            finger_status_.currents[i+1] = (tau[i*2 + 1] + tau[i*2 + 2]) / 2.0;
        }
    }
    /**
     * @brief write command to mjData
     * @note callback for write/control thread.
     * @param d
     */
    void writeCallback(mjData *d) override {
        std::vector<double> ctrl_cmd;
        auto iter = jga_.ctrladr().begin();
        // thumb yaw: 单关节无耦合，直接映射（和其他手指的单控制逻辑对齐）
        double roll_norm = ctrl_cmd_[1] / 255.0;
        auto &roll_range = ctrllimited_map_[*iter];
        double roll_cmd = roll_range[0] + roll_norm * (roll_range[1] - roll_range[0]);
        ctrl_cmd.push_back(roll_cmd);
        iter++;
        // thumb pitch + ip: 耦合控制，和其他手指逻辑完全一致，使用拇指专属mimic比例1.226495
        auto &pitch_range = ctrllimited_map_[*iter];
        iter++;
        auto &dip_range = ctrllimited_map_[*iter];
        iter++;
        auto thumb_joint_cmd = Curl2Joints(ctrl_cmd_[0], pitch_range, dip_range, true);
        ctrl_cmd.push_back(thumb_joint_cmd[0]);
        ctrl_cmd.push_back(thumb_joint_cmd[1]);
        // index, middle, ring, little
        for (int i = 2; i < 6; i++) {
            auto &range1 = ctrllimited_map_[*iter];
            iter ++;
            auto &range2 = ctrllimited_map_[*iter];
            iter ++;
            auto joint_cmd = Curl2Joints(ctrl_cmd_[i], range1, range2);
            ctrl_cmd.push_back(joint_cmd[0]);
            ctrl_cmd.push_back(joint_cmd[1]);
        }
        int i = 0;
        for(auto iter = jga_.ctrladr().begin(); iter != jga_.ctrladr().end(); ++iter) {
            d->ctrl[*iter] = ctrl_cmd[i++];
        }
    }
    /**
     * @brief Set the Finger Positions.
     *
     * @param positions range: 0~255, 0 for open, 255 for close.（内部自动反转，与O6处理方式一致）
     */
    void setFingerPositions(const UnsignedFingerArray &positions) override {
        constexpr int finger_count = std::tuple_size<UnsignedFingerArray>::value;
        for (int i = 0; i < finger_count; i++) {
            // 指令反转：输入position=255→张开（控制值0），position=0→闭合（控制值255），与O6处理方式一致
            ctrl_cmd_[i] = static_cast<FingerArray::value_type>(255 - positions[i]);
        }
        ctrl_updated_ = true;
    }
    /**
     * @brief Set the Finger Speeds object
     *
     * @param speeds range: -100~100
     * @note The fingers will move at the set speed values until they stall.
     *       The value range is -100 to 100. Positive values indicate flexion, negative values indicate extension.
     */
    void setFingerSpeeds(const FingerArray &speeds) override {
        // TODO: implement speed control for LinkerL6 hand
    }
    /**
     * @brief Get the Finger Status object
     *
     * @return FingerStatusPtr
     */
    FingerStatusPtr getFingerStatus() override {
        return std::make_shared<FingerStatus>(finger_status_);
    }
private:
    JointGroupAddress jga_;
    std::map<int, std::array<double, 2>> ctrllimited_map_;
    FingerStatus finger_status_;
    // 控制指令更新标志
    std::atomic<bool> ctrl_updated_{false};
    // 控制指令缓存，0-255范围
    // 初始值对应：大拇指弯曲输入155（半弯），其余输入255（完全张开），与O6逻辑完全一致
    FingerArray ctrl_cmd_{100, 100, 0, 0, 0, 0};
    /**
     * @brief 将关节弧度转换为0-255的弯曲值
     * @param j1_val 近指关节角度
     * @param j1_range 近指关节范围 [min, max]
     * @param j2_val 远指关节角度
     * @param j2_range 远指关节范围 [min, max]
     * @return 归一化后的弯曲值 0-255
     */
    double Joints2Curl(
        double j1_val, const std::array<double, 2>& j1_range,
        double j2_val, const std::array<double, 2>& j2_range)
    {
        // 计算并限制关节归一化位置
        double norm1 = (j1_val - j1_range[0]) / (j1_range[1] - j1_range[0]);
        norm1 = std::clamp(norm1, 0.0, 1.0);
        double norm2 = (j2_val - j2_range[0]) / (j2_range[1] - j2_range[0]);
        norm2 = std::clamp(norm2, 0.0, 1.0);
        // 计算平均曲度，LinkerL6手范围0-255
        return (norm1 + norm2) / 2.0 * 255.0;
    }
    /**
     * @brief 将0-255的弯曲值转换为关节控制弧度
     * @param curl 0-255的弯曲值
     * @param j1_range 近指关节范围 [min, max]
     * @param j2_range 远指关节范围 [min, max]
     * @param is_thumb 是否是拇指关节，使用不同的耦合比例
     * @return [近指关节指令, 远指关节指令]
     */
    std::array<double, 2> Curl2Joints(
        double curl,
        const std::array<double, 2>& j1_range,
        const std::array<double, 2>& j2_range,
        bool is_thumb = false)
    {
        // 限制曲度输入范围，LinkerL6手是0-255
        curl = std::clamp(curl, 0.0, 255.0);
        // 计算归一化曲度，LinkerL6手除以255
        double norm = curl / 255.0;
        // 近指关节用原本的映射方式
        double j1_command = j1_range[0] + norm * (j1_range[1] - j1_range[0]);
        // 远指关节采用真实机械mimic比例计算
        // 非大拇指手指mimic比例1.125676，大拇指mimic比例1.226495
        double mimic_ratio = is_thumb ? 1.226495 : 1.125676;
        // 远指关节角度 = 近指关节角度 * mimic比例，offset为0
        double j2_command = j1_command * mimic_ratio;
        // 限制远指关节在自身运动范围内
        j2_command = std::clamp(j2_command, j2_range[0], j2_range[1]);
        return {j1_command, j2_command};
    }
};
using LinkerL6HandPtr = std::shared_ptr<mujoco_node::LinkerL6Hand>;
} // namespace mujoco_node
#endif