#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <signal.h>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <iomanip>
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

using namespace org::eclipse::cyclonedds;


// Constants
constexpr size_t KUAVO_JOINT_COUNT = 28;
constexpr size_t DDS_MOTOR_COUNT = 35;
constexpr uint32_t CRC_POLYNOMIAL = 0x04c11db7;

// Topics
extern const std::string DDS_CMD_TOPIC;
extern const std::string DDS_STATE_TOPIC;

// CRC calculation utility
uint32_t Crc32Core(uint32_t *ptr, uint32_t len);

// DdsLowStateListener class declaration
template<typename StateMessageType>
class DdsLowStateListener : public dds::sub::NoOpDataReaderListener<StateMessageType> {
public:
    DdsLowStateListener();

    void on_data_available(dds::sub::DataReader<StateMessageType>& reader) override;

    uint64_t getMessageCount() const;
    StateMessageType getLatestData() const;
    void setLowdstateCallback(std::function<void(const StateMessageType&)> callback);

private:
    std::atomic<uint64_t> message_count_;
    mutable std::mutex data_mutex_;
    StateMessageType latest_state_data_;
    std::function<void(const StateMessageType&)> ext_lowdstate_callback_;
};

// HumanoidControllerDDSClient class declaration
template<typename CmdType, typename StateType>
class HumanoidControllerDDSClient {
public:
    HumanoidControllerDDSClient();
    ~HumanoidControllerDDSClient();

    void start();
    void stop();

    // Low command publishing API
    void publishLowCmd(const CmdType& cmd);

    std::unique_ptr<DdsLowStateListener<StateType>> state_listener_;

private:
    void setupStateListener();

    // DDS entities
    dds::domain::DomainParticipant participant_;
    dds::topic::Topic<CmdType> cmd_topic_;
    dds::topic::Topic<StateType> state_topic_;
    dds::pub::DataWriter<CmdType> cmd_writer_;
    dds::sub::DataReader<StateType> state_reader_;

    // Threading
    std::thread publish_thread_;
    std::atomic<bool> running_;

    // Performance counters
    std::atomic<uint64_t> publish_count_;
}; 