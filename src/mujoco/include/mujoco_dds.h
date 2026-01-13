#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <functional>
#include <mutex>

// DDS includes
#include <dds/dds.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/pub/ddspub.hpp>

// Unitree SDK DDS types
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"

// Leju DDS types
#include "leju/idl/JointCmd.hpp"
#include "leju/idl/SensorsData.hpp"
#include "leju/idl/ImuData.hpp"
#include "leju/idl/JointData.hpp"
#include "leju/idl/EndEffectorData.hpp"

// Type aliases for convenience
using LejuJointCmd = leju::msgs::JointCmd;
using LejuSensorsData = leju::msgs::SensorsData;
using LejuImuData = leju::msgs::ImuData;
using LejuJointData = leju::msgs::JointData;
using LejuEndEffectorData = leju::msgs::EndEffectorData;

// MuJoCo includes
#include <mujoco/mujoco.h>

// Eigen includes (for free_acc calculation)
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace org::eclipse::cyclonedds;

// Forward declarations
// template<typename CmdMessageType>
// class MujocoDdsLowCmdListener;
// template<typename CmdType, typename StateType>
// class MujocoDdsClient;

// Constants
constexpr size_t KUAVO_JOINT_COUNT = 28;
constexpr size_t DDS_MOTOR_COUNT = 35;
constexpr uint32_t CRC_POLYNOMIAL = 0x04c11db7;

// Topics (same as hardware plant)
static const std::string DDS_CMD_TOPIC = "rt/lowcmd";
static const std::string DDS_STATE_TOPIC = "rt/lowstate";

// CRC calculation utility (from Unitree SDK)
inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = CRC_POLYNOMIAL;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else
                CRC32 <<= 1;
            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

// MuJoCo low command listener
template<typename CmdMessageType>
class MujocoDdsLowCmdListener : public dds::sub::NoOpDataReaderListener<CmdMessageType> {
public:
    explicit MujocoDdsLowCmdListener(std::function<void(const CmdMessageType&)> callback);
    void on_data_available(dds::sub::DataReader<CmdMessageType>& reader) override;

    uint64_t getMessageCount() const { return message_count_.load(); }

private:
    std::function<void(const CmdMessageType&)> callback_;
    std::atomic<uint64_t> message_count_;
};

/**
 * MujocoDdsClient - DDS communication for MuJoCo simulation
 *
 * This class handles DDS communication for MuJoCo simulation.
 * It receives LowCmd messages and publishes LowState messages.
 */
template<typename CmdType, typename StateType>
class MujocoDdsClient {
public:
    MujocoDdsClient();
    ~MujocoDdsClient();

    void start();
    void stop();

    // Low command subscription
    void setLowCmdCallback(std::function<void(const CmdType&)> callback);

    // Low state publishing API
    void publishLowState(const StateType& state);

    
    // Statistics
    uint64_t getCommandCount() const { return cmd_count_.load(); }
    uint64_t getStateCount() const { return state_count_.load(); }

private:
    void setupCmdListener();
    void onLowCmdReceived(const CmdType& cmd);
    bool isValidDdsLowCommand(const CmdType& cmd) const;

    // DDS entities
    dds::domain::DomainParticipant participant_;
    dds::topic::Topic<CmdType> cmd_topic_;
    dds::topic::Topic<StateType> state_topic_;
    dds::sub::DataReader<CmdType> cmd_reader_;
    dds::pub::DataWriter<StateType> state_writer_;

    // Command handling
    CmdType last_cmd_;
    std::atomic<bool> has_new_cmd_;
    using MujocoDdsLowCmdListenerType = MujocoDdsLowCmdListener<CmdType>;
    std::unique_ptr<MujocoDdsLowCmdListenerType> cmd_listener_;
    mutable std::mutex command_mutex_;

    // External callback for commands
    std::function<void(const CmdType&)> ext_cmd_callback_;

    // Threading
    std::atomic<bool> running_;

    // Performance counters
    std::atomic<uint64_t> cmd_count_;
    std::atomic<uint64_t> state_count_;
};

// Template function for converting MuJoCo data to DDS LowState
template<typename StateType>
void ConvertMujocoToDdsState(
    const std::vector<double>& joint_q,
    const std::vector<double>& joint_v,
    const std::vector<double>& joint_vd,
    const std::vector<double>& joint_torque,
    const Eigen::Vector3d& acc_eigen,
    const Eigen::Vector3d& angVel,
    const Eigen::Vector3d& free_acc,
    const Eigen::Vector4d& ori,
    StateType& dds_state);

// Specialization for LowState_
template<>
void ConvertMujocoToDdsState<unitree_hg::msg::dds_::LowState_>(
    const std::vector<double>& joint_q,
    const std::vector<double>& joint_v,
    const std::vector<double>& joint_vd,
    const std::vector<double>& joint_torque,
    const Eigen::Vector3d& acc_eigen,
    const Eigen::Vector3d& angVel,
    const Eigen::Vector3d& free_acc,
    const Eigen::Vector4d& ori,
    unitree_hg::msg::dds_::LowState_& dds_state);

// Specialization for Leju DDS SensorsData (for publishing sensor data)
template<>
void ConvertMujocoToDdsState<leju::msgs::SensorsData>(
    const std::vector<double>& joint_q,
    const std::vector<double>& joint_v,
    const std::vector<double>& joint_vd,
    const std::vector<double>& joint_torque,
    const Eigen::Vector3d& acc_eigen,
    const Eigen::Vector3d& angVel,
    const Eigen::Vector3d& free_acc,
    const Eigen::Vector4d& ori,
    leju::msgs::SensorsData& leju_sensors_data); 