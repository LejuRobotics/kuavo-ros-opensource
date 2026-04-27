#ifndef __LINKERO6_HAND_HPP__
#define __LINKERO6_HAND_HPP__
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

// LinkerO6灵巧手实现，控制范围0-255（0=完全张开，255=完全握紧）
// 控制通道定义：[拇指横摆, 拇指弯曲, 食指弯曲, 中指弯曲, 无名指弯曲, 小指弯曲]
class LinkerO6Hand : public MujocoHandBase {
public:
    LinkerO6Hand(const mjModel* model, const JointGroupAddress& jga) : jga_(jga) {
        // 读取所有执行器的控制范围
        if (!jga_.ctrladr().invalid()) {
            for (auto iter = jga_.ctrladr().begin(); iter != jga_.ctrladr().end(); ++iter) {
                auto actuator_id = *iter;
                if (model->actuator_ctrllimited[actuator_id]) {
                    double min_val = model->actuator_ctrlrange[2 * actuator_id];
                    double max_val = model->actuator_ctrlrange[2 * actuator_id + 1];
                    ctrllimited_map_[actuator_id] = {min_val, max_val};
                }
            }
        }
    }
    ~LinkerO6Hand() override = default;

    /**
     * @brief 从mjData读取传感器和关节状态
     * @note 读线程回调
     * @param d mujoco数据
     */
    void readCallback(const mjData* d) override {
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> tau;

        // 读取所有关节的位置、速度、力矩
        auto iter_qpos = jga_.qposadr().begin();
        auto iter_qvel = jga_.qdofadr().begin();
        for (; iter_qpos != jga_.qposadr().end() && iter_qvel != jga_.qdofadr().end(); ++iter_qpos, ++iter_qvel) {
            pos.push_back(d->qpos[*iter_qpos]);
            vel.push_back(d->qvel[*iter_qvel] * (180.0 / M_PI)); // 转换为角度/秒
            tau.push_back(d->qfrc_actuator[*iter_qvel] * 1000); // 转换为mNm
        }

        auto iter_ctrl = jga_.ctrladr().begin();

        // 拇指处理
        // pos[0]是thumb_cmc_yaw，范围[0, 1.3]弧度，单独控制无耦合
        finger_status_.positions[1] = Joints2Curl(pos[0], {0, 1.3}, pos[0], {0, 1.3});
        finger_status_.speeds[1] = vel[0];
        finger_status_.currents[1] = tau[0];
        iter_ctrl++; // 跳过yaw执行器

        // pos[1]是thumb_cmc_pitch，pos[2]是thumb_ip，两个关节耦合
        finger_status_.positions[0] = Joints2Curl(pos[1], {0, 0.58}, pos[2], {0, 1.08});
        finger_status_.speeds[0] = (vel[1] + vel[2]) / 2.0;
        finger_status_.currents[0] = (tau[1] + tau[2]) / 2.0;
        ++iter_ctrl; // 跳过pitch执行器
        ++iter_ctrl; // 跳过ip执行器

        // 其他手指处理：index(2), middle(3), ring(4), pinky(5)
        for (int i = 2; i < 6; i++) {
            int pos_idx = 2 + (i - 2) * 2; // 食指从pos[3]开始
            auto joint1_pos = pos[pos_idx];
            auto joint2_pos = pos[pos_idx + 1];
            auto& range1 = ctrllimited_map_[*iter_ctrl];
            iter_ctrl++;
            auto& range2 = ctrllimited_map_[*iter_ctrl];
            iter_ctrl++;

            finger_status_.positions[i] = Joints2Curl(joint1_pos, range1, joint2_pos, range2);
            finger_status_.speeds[i] = (vel[pos_idx] + vel[pos_idx + 1]) / 2.0;
            finger_status_.currents[i] = (tau[pos_idx] + tau[pos_idx + 1]) / 2.0;
        }
    }

    /**
     * @brief 写入控制指令到mjData
     * @note 写/控制线程回调
     * @param d mujoco数据
     */
    void writeCallback(mjData* d) override {
        std::vector<double> ctrl_cmd;

        auto iter = jga_.ctrladr().begin();

        // 拇指横摆：ctrl_cmd_[1]，单独控制无耦合
        double yaw_curl = ctrl_cmd_[1];
        auto yaw_range = ctrllimited_map_[*iter];
        double yaw_command = yaw_range[0] + (yaw_curl / 255.0) * (yaw_range[1] - yaw_range[0]);
        yaw_command = std::clamp(yaw_command, yaw_range[0], yaw_range[1]);
        ctrl_cmd.push_back(yaw_command);
        iter++;

        // 拇指弯曲：ctrl_cmd_[0]，控制pitch和ip关节，耦合比例1.86
        auto iter_next = iter;
        ++iter_next;
        auto thumb_cmd = Curl2Joints(ctrl_cmd_[0], ctrllimited_map_[*iter], ctrllimited_map_[*iter_next], true);
        ctrl_cmd.push_back(thumb_cmd[0]);
        ctrl_cmd.push_back(thumb_cmd[1]);
        ++iter;
        ++iter;

        // 其他手指：index(2), middle(3), ring(4), pinky(5)，耦合比例0.89
        for (int i = 2; i < 6; i++) {
            auto& range1 = ctrllimited_map_[*iter];
            iter++;
            auto& range2 = ctrllimited_map_[*iter];
            iter++;
            auto joint_cmd = Curl2Joints(ctrl_cmd_[i], range1, range2);
            ctrl_cmd.push_back(joint_cmd[0]);
            ctrl_cmd.push_back(joint_cmd[1]);
        }

        // 写入控制指令到mjData
        int i = 0;
        for (auto iter_ctrl = jga_.ctrladr().begin(); iter_ctrl != jga_.ctrladr().end(); ++iter_ctrl) {
            d->ctrl[*iter_ctrl] = ctrl_cmd[i++];
        }
    }

    /**
     * @brief 设置手指位置
     * @param positions 数组范围0~255，索引0-5分别对应：
     * 0: 拇指弯曲, 1: 拇指横摆, 2: 食指, 3: 中指, 4: 无名指, 5: 小指
     */
    void setFingerPositions(const UnsignedFingerArray& positions) override {
        constexpr int finger_count = std::tuple_size<UnsignedFingerArray>::value;
        for (int i = 0; i < finger_count; i++) {
            // 指令反转：输入position=255→张开（控制值0），position=0→闭合（控制值255）
            ctrl_cmd_[i] = static_cast<FingerArray::value_type>(255 - positions[i]);
        }
        ctrl_updated_ = true;
    }

    /**
     * @brief 设置手指速度（暂未实现）
     * @param speeds 范围-100~100
     */
    void setFingerSpeeds(const FingerArray& speeds) override {
        // TODO: 实现O6手的速度控制
        (void)speeds;
    }

    /**
     * @brief 获取手指状态
     * @return 手指状态指针，包含位置、速度、电流
     */
    FingerStatusPtr getFingerStatus() override {
        return std::make_shared<FingerStatus>(finger_status_);
    }

private:
    JointGroupAddress jga_;
    // 存储每个执行器的控制范围 [min, max]
    std::map<int, std::array<double, 2>> ctrllimited_map_;
    // 手指状态
    FingerStatus finger_status_;
    // 控制指令更新标志
    std::atomic<bool> ctrl_updated_{false};
    // 控制指令缓存，0-255范围
    // 初始值对应：大拇指弯曲输入155（半弯），其余输入255（完全张开）
    FingerArray ctrl_cmd_{100, 100, 0, 0, 0, 0};

    /**
     * @brief 将关节弧度转换为0-255的弯曲值
     * @param j1_val 近指关节角度
     * @param j1_range 近指关节范围 [min, max]
     * @param j2_val 远指关节角度
     * @param j2_range 远指关节范围 [min, max]
     * @return 归一化后的弯曲值 0-255
     */
    double Joints2Curl(double j1_val, const std::array<double, 2>& j1_range,
                      double j2_val, const std::array<double, 2>& j2_range) {
        // 计算近指关节的归一化位置
        double norm1 = (j1_val - j1_range[0]) / (j1_range[1] - j1_range[0]);
        norm1 = std::clamp(norm1, 0.0, 1.0);

        // 计算远指关节的归一化位置
        double norm2 = (j2_val - j2_range[0]) / (j2_range[1] - j2_range[0]);
        norm2 = std::clamp(norm2, 0.0, 1.0);

        // 平均后转换为0-255范围
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
    std::array<double, 2> Curl2Joints(double curl, const std::array<double, 2>& j1_range,
                                    const std::array<double, 2>& j2_range, bool is_thumb = false) {
        // 限制输入范围0-255
        curl = std::clamp(curl, 0.0, 255.0);

        // 转换为归一化值 0-1
        double norm = curl / 255.0;

        // 近指关节直接映射到范围
        double j1_command = j1_range[0] + norm * (j1_range[1] - j1_range[0]);

        // 远指关节使用耦合比例：拇指1.86，其他手指0.89
        double mimic_ratio = is_thumb ? 1.86 : 0.89;
        double j2_command = j1_command * mimic_ratio;

        // 限制远指关节在自身范围内
        j2_command = std::clamp(j2_command, j2_range[0], j2_range[1]);

        return {j1_command, j2_command};
    }
};

using LinkerO6HandPtr = std::shared_ptr<mujoco_node::LinkerO6Hand>;

} // namespace mujoco_node

#endif // __LINKERO6_HAND_HPP__
