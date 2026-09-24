#ifndef KUAVO_MUJOCO_BLACK_MAN_11_HAND_HPP_
#define KUAVO_MUJOCO_BLACK_MAN_11_HAND_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace mujoco_node {

class BlackMan11Hand {
public:
    static constexpr std::size_t kJointCount = 11;

    BlackMan11Hand(const mjModel* model, const std::string& side_prefix) {
        const std::array<std::string, kJointCount> suffixes = {
            "thumb_j1", "thumb_j2", "thumb_j3",
            "index_j1", "index_j2", "index_j3",
            "middle_j1", "middle_j2",
            "little_j1", "little_j2", "little_j3",
        };

        for (std::size_t index = 0; index < kJointCount; ++index) {
            names_[index] = side_prefix + "_" + suffixes[index];
            const int joint_id = mj_name2id(model, mjOBJ_JOINT, names_[index].c_str());
            const std::string actuator_name = names_[index] + "_motor";
            const int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str());
            if (joint_id < 0 || actuator_id < 0) {
                throw std::runtime_error(
                    "BLACK_MAN_11 missing joint or actuator: " + names_[index]);
            }

            const int qpos_address = model->jnt_qposadr[joint_id];
            const int dof_address = model->jnt_dofadr[joint_id];
            if (qpos_address < 0 || dof_address < 0) {
                throw std::runtime_error(
                    "BLACK_MAN_11 invalid qpos/dof address: " + names_[index]);
            }

            qpos_addresses_[index] = qpos_address;
            dof_addresses_[index] = dof_address;
            actuator_ids_[index] = actuator_id;
            name_to_index_.emplace(names_[index], index);
            if (model->actuator_ctrllimited[actuator_id]) {
                control_min_[index] = model->actuator_ctrlrange[2 * actuator_id];
                control_max_[index] = model->actuator_ctrlrange[2 * actuator_id + 1];
            } else {
                control_min_[index] = -mjMAXVAL;
                control_max_[index] = mjMAXVAL;
            }
            ROS_INFO("[DexHandMujoco] BLACK_MAN_11 %s joint=%d qpos=%d dof=%d actuator=%d",
                     names_[index].c_str(), joint_id, qpos_address, dof_address, actuator_id);
        }
    }

    bool setCommand(const sensor_msgs::JointState& command, std::string* error) {
        if (command.name.size() != kJointCount || command.position.size() != kJointCount) {
            if (error) *error = "expected exactly 11 joint names and 11 positions";
            return false;
        }

        std::array<double, kJointCount> next_command{};
        std::array<bool, kJointCount> assigned{};
        for (std::size_t input = 0; input < kJointCount; ++input) {
            if (!std::isfinite(command.position[input])) {
                if (error) *error = "non-finite position for joint: " + command.name[input];
                return false;
            }
            const auto entry = name_to_index_.find(command.name[input]);
            if (entry == name_to_index_.end()) {
                if (error) *error = "unknown joint name: " + command.name[input];
                return false;
            }
            const std::size_t index = entry->second;
            if (assigned[index]) {
                if (error) *error = "duplicate joint name: " + command.name[input];
                return false;
            }
            assigned[index] = true;
            next_command[index] = std::clamp(
                command.position[input], control_min_[index], control_max_[index]);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        command_ = next_command;
        return true;
    }

    void readCallback(const mjData* data) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t index = 0; index < kJointCount; ++index) {
            position_[index] = data->qpos[qpos_addresses_[index]];
            velocity_[index] = data->qvel[dof_addresses_[index]];
            effort_[index] = data->actuator_force[actuator_ids_[index]];
        }
    }

    void writeCallback(mjData* data) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t index = 0; index < kJointCount; ++index) {
            data->ctrl[actuator_ids_[index]] = command_[index];
        }
    }

    sensor_msgs::JointState state() const {
        std::lock_guard<std::mutex> lock(mutex_);
        sensor_msgs::JointState message;
        message.name.assign(names_.begin(), names_.end());
        message.position.assign(position_.begin(), position_.end());
        message.velocity.assign(velocity_.begin(), velocity_.end());
        message.effort.assign(effort_.begin(), effort_.end());
        return message;
    }

private:
    std::array<std::string, kJointCount> names_{};
    std::array<int, kJointCount> qpos_addresses_{};
    std::array<int, kJointCount> dof_addresses_{};
    std::array<int, kJointCount> actuator_ids_{};
    std::array<double, kJointCount> control_min_{};
    std::array<double, kJointCount> control_max_{};
    std::unordered_map<std::string, std::size_t> name_to_index_;

    mutable std::mutex mutex_;
    std::array<double, kJointCount> command_{};
    std::array<double, kJointCount> position_{};
    std::array<double, kJointCount> velocity_{};
    std::array<double, kJointCount> effort_{};
};

using BlackMan11HandPtr = std::shared_ptr<BlackMan11Hand>;

}  // namespace mujoco_node

#endif
