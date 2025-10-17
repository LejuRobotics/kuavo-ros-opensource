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

using namespace org::eclipse::cyclonedds;

// Forward declarations
class DdsLowStateListener;
class HumanoidControllerDDSClient;

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
class DdsLowStateListener : public dds::sub::NoOpDataReaderListener<unitree_hg::msg::dds_::LowState_> {
public:
    DdsLowStateListener();
    
    void on_data_available(dds::sub::DataReader<unitree_hg::msg::dds_::LowState_>& reader) override;
    
    uint64_t getMessageCount() const;
    unitree_hg::msg::dds_::LowState_ getLatestData() const;
    void setLowdstateCallback(std::function<void(const unitree_hg::msg::dds_::LowState_&)> callback);
    
private:
    std::atomic<uint64_t> message_count_;
    mutable std::mutex data_mutex_;
    unitree_hg::msg::dds_::LowState_ latest_state_data_;
    std::function<void(const unitree_hg::msg::dds_::LowState_&)> ext_lowdstate_callback_;
};

// HumanoidControllerDDSClient class declaration
class HumanoidControllerDDSClient {
public:
    HumanoidControllerDDSClient();
    ~HumanoidControllerDDSClient();
    
    void start();
    void stop();
    
    // Low command publishing API
    void publishLowCmd(const unitree_hg::msg::dds_::LowCmd_& cmd);
    
    std::unique_ptr<DdsLowStateListener> state_listener_;
    
private:
    void setupStateListener();
    
    // DDS entities
    dds::domain::DomainParticipant participant_;
    dds::topic::Topic<unitree_hg::msg::dds_::LowCmd_> cmd_topic_;
    dds::topic::Topic<unitree_hg::msg::dds_::LowState_> state_topic_;
    dds::pub::DataWriter<unitree_hg::msg::dds_::LowCmd_> cmd_writer_;
    dds::sub::DataReader<unitree_hg::msg::dds_::LowState_> state_reader_;
    
    // Threading
    std::thread publish_thread_;
    std::atomic<bool> running_;
    
    // Performance counters
    std::atomic<uint64_t> publish_count_;
}; 