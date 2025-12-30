#ifndef __MUJOCO_DEXHAND_HPP__
#define __MUJOCO_DEXHAND_HPP__
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

namespace mujoco_node {
using FingerArray = std::array<int16_t, 6>;
using UnsignedFingerArray = std::array<uint16_t, 6>;
struct FingerStatus {
    UnsignedFingerArray positions;
    FingerArray speeds;
    FingerArray currents;
    UnsignedFingerArray states;

    FingerStatus() : positions{}, speeds{}, currents{}, states{} {}
    friend std::ostream& operator<<(std::ostream& os, const FingerStatus& status) {
        os << "Finger Positions: ";
        for (const auto& pos : status.positions) {
            os << pos << " ";
        }
        os << "\nFinger Speeds: ";
        for (const auto& speed : status.speeds) {
            os << speed << " ";
        }
        os << "\nFinger Currents: ";
        for (const auto& current : status.currents) {
            os << current << " ";
        }
        os << "\nFinger States: ";
        for (const auto& state : status.states) {
            os << state << " ";
        }
        return os;
    }
};
using DualHandsArray = std::array<FingerArray, 2>;
using UnsignedDualHandsArray = std::array<UnsignedFingerArray, 2>;
using FingerStatusPtr = std::shared_ptr<FingerStatus>;
using FingerStatusPtrArray = std::array<FingerStatusPtr, 2>;

class MujocoDexHand;
using MujocoDexHandPtr = std::shared_ptr<mujoco_node::MujocoDexHand>;

class MujocoDexHand {
public:
    MujocoDexHand(const mjModel* model, const JointGroupAddress& jga): jga_(jga) {
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
    void readCallback(const mjData *d) {
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> tau;
        auto iter0 = jga_.qposadr().begin();
        auto iter1 = jga_.qdofadr().begin();
        for(; iter0 != jga_.qposadr().end() && iter1 != jga_.qdofadr().end(); ++iter0, ++iter1) {
            pos.push_back(d->qpos[*iter0]);
            // std::cout << "pos:" << *iter0 << " " << d->qpos[*iter0] << std::endl;
            vel.push_back(d->qvel[*iter1] * (180.0 / M_PI));
            tau.push_back(d->qfrc_actuator[*iter1] * 1000);
        }

        // thumb
        auto iter = jga_.ctrladr().begin();
        finger_status_.positions[0] = Joints2Curl(pos[0], {0.23, 1.36}, pos[0], {0.23, 1.36});
        // finger_status_.positions[0] = Joints2Curl(pos[0], ctrllimited_map_[*iter], pos[0], ctrllimited_map_[*iter]);
        finger_status_.speeds[0] = vel[0];
        finger_status_.currents[0] = tau[0];            
        iter++; 

        finger_status_.positions[1] = Joints2Curl(pos[1], {0.16, 0.75}, pos[1], {0.16, 0.75});
        // finger_status_.positions[1] = Joints2Curl(pos[1], ctrllimited_map_[*iter], pos[1], ctrllimited_map_[*iter]);
        finger_status_.speeds[1] = vel[1]; 
        finger_status_.currents[1] = tau[1];        
        
        // index, middle, ring, little
        iter++;
        for (int i = 1; i < 5; i++) {
            auto joint0 = pos[i*2];
            auto joint1 = pos[i*2+1];
            auto &range0 = ctrllimited_map_[*iter];
            iter ++;
            auto &range1 = ctrllimited_map_[*iter];
            iter ++;
            finger_status_.positions[i+1] = Joints2Curl(joint0, range0, joint1, range1);
            finger_status_.speeds[i+1] = (vel[i*2] + vel[i*2+1])/2;
            finger_status_.currents[i+1] = (tau[i*2] + tau[i*2+1])/2;
        }
    }

    double Joints2Curl(
        double j1_val, const std::array<double, 2>& j1_range,
        double j2_val, const std::array<double, 2>& j2_range)
    {
        // 计算并限制关节归一化位置
        double norm1 = (j1_val - j1_range[0]) / (j1_range[1] - j1_range[0]);
        norm1 = std::clamp(norm1, 0.0, 1.0);
        
        double norm2 = (j2_val - j2_range[0]) / (j2_range[1] - j2_range[0]);
        norm2 = std::clamp(norm2, 0.0, 1.0);

        // 计算平均曲度
        return (norm1 + norm2) / 2.0 * 100.0;
    }

    // 将曲度转换为关节控制指令
    std::array<double, 2> Curl2Joints(
        double curl, 
        const std::array<double, 2>& j1_range,
        const std::array<double, 2>& j2_range)
    {
        // 限制曲度输入范围
        curl = std::clamp(curl, 0.0, 100.0);
        
        // 计算归一化曲度
        double norm = curl / 100.0;
        
        // 生成关节控制指令
        double j1_command = j1_range[0] + norm * (j1_range[1] - j1_range[0]);
        double j2_command = j2_range[0] + norm * (j2_range[1] - j2_range[0]);
        return {j1_command, j2_command};
    }

    /**
     * @brief write command to mjData
     * @note callback for write/control thread. 
     * @param d 
     */
    void writeCallback(mjData *d) {
        // if(!ctrl_updated_) return;
        std::vector<double> ctrl_cmd; 
        
        auto iter = jga_.ctrladr().begin();
        // thumb
        auto joint_cmd0 = Curl2Joints(ctrl_cmd_[0], ctrllimited_map_[*iter], ctrllimited_map_[*iter]);
        iter++;
        auto joint_cmd1 = Curl2Joints(ctrl_cmd_[1], ctrllimited_map_[*iter], ctrllimited_map_[*iter]);
        iter++;
        ctrl_cmd.push_back(joint_cmd0[0]);
        ctrl_cmd.push_back(joint_cmd1[0]);
        
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
     * @param positions range: 0~100, 0 for open, 100 for close.
     */
    void setFingerPositions(const UnsignedFingerArray &positions){
        constexpr int finger_count = std::tuple_size<UnsignedFingerArray>::value; 
        for (int i = 0; i < finger_count; i++) {
            ctrl_cmd_[i] = static_cast<FingerArray::value_type>(positions[i]);
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
    void setFingerSpeeds(const FingerArray &speeds) {
        // TODO: implement
    }

    /**
     * @brief Get the Finger Status object
     * 
     * @return FingerStatusPtr 
     */
    FingerStatusPtr getFingerStatus() {
        return std::make_shared<FingerStatus>(finger_status_);
    }
private:
    JointGroupAddress jga_;
    std::map<int, std::array<double, 2>> ctrllimited_map_;

    FingerStatus finger_status_;
    std::atomic<bool> ctrl_updated_{false};
    FingerArray  ctrl_cmd_{0, 0, 0, 0, 0, 0};
};
} // namespace mujoco_node

#endif