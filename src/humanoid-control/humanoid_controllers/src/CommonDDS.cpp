#include "humanoid_controllers/CommonDDS.h"

// Unitree SDK DDS types
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"

using namespace org::eclipse::cyclonedds;

// Topics
const std::string DDS_CMD_TOPIC = "rt/lowcmd";
const std::string DDS_STATE_TOPIC = "rt/lowstate";

// CRC calculation utility (from Unitree SDK)
uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = CRC_POLYNOMIAL;
    for (uint32_t i = 0; i < len; i++) {
        uint32_t xbit = 1U << 31;
        uint32_t data = ptr[i];
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

// DdsLowStateListener implementation
template<typename StateMessageType>
DdsLowStateListener<StateMessageType>::DdsLowStateListener() : message_count_(0) {}

template<typename StateMessageType>
void DdsLowStateListener<StateMessageType>::on_data_available(dds::sub::DataReader<StateMessageType>& reader) {
    auto samples = reader.read();
    for (const auto& sample : samples) {
        if (sample.info().valid()) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_state_data_ = sample.data();
            }
            
            // Call external callback directly if set
            if (ext_lowdstate_callback_) {
                ext_lowdstate_callback_(sample.data());
            } else {
                std::cout << "Received DDS state data " << message_count_.load() << std::endl;
            }
            
            message_count_++;
        }
    }
}
    
template<typename StateMessageType>
uint64_t DdsLowStateListener<StateMessageType>::getMessageCount() const { return message_count_.load(); }
    
template<typename StateMessageType>
StateMessageType DdsLowStateListener<StateMessageType>::getLatestData() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_state_data_;
}
    
template<typename StateMessageType>
void DdsLowStateListener<StateMessageType>::setLowdstateCallback(std::function<void(const StateMessageType&)> callback) {
        ext_lowdstate_callback_ = callback;
}

// HumanoidControllerDDSClient implementation
template<typename CmdType, typename StateType>
HumanoidControllerDDSClient<CmdType, StateType>::HumanoidControllerDDSClient()
    : participant_(org::eclipse::cyclonedds::domain::default_id())
    , cmd_topic_(participant_, DDS_CMD_TOPIC)
    , state_topic_(participant_, DDS_STATE_TOPIC)
    , cmd_writer_(dds::pub::Publisher(participant_), cmd_topic_)
    , state_reader_(dds::sub::Subscriber(participant_), state_topic_)
    , running_(false)
    , publish_count_(0)
{
    setupStateListener();
    std::cout << "Humanoid Controller DDS Client initialized" << std::endl;
}

template<typename CmdType, typename StateType>
HumanoidControllerDDSClient<CmdType, StateType>::~HumanoidControllerDDSClient() {
    stop();
}

template<typename CmdType, typename StateType>
void HumanoidControllerDDSClient<CmdType, StateType>::start() {
    if (running_.exchange(true)) {
        std::cerr << "Client already running" << std::endl;
        return;
    }
}

template<typename CmdType, typename StateType>
void HumanoidControllerDDSClient<CmdType, StateType>::stop() {
    if (!running_.exchange(false)) {
        return;
    }
}


template<typename CmdType, typename StateType>
void HumanoidControllerDDSClient<CmdType, StateType>::publishLowCmd(const CmdType& cmd) {
    cmd_writer_.write(cmd);
}

template<typename CmdType, typename StateType>
void HumanoidControllerDDSClient<CmdType, StateType>::setupStateListener() {
    state_listener_ = std::make_unique<DdsLowStateListener<StateType>>();
    state_reader_.listener(state_listener_.get(),
                          dds::core::status::StatusMask::data_available());
}

// Explicit template instantiation for the specific message types
template class DdsLowStateListener<unitree_hg::msg::dds_::LowState_>;
template class HumanoidControllerDDSClient<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>;

// Explicit template instantiation for Leju DDS types
template class DdsLowStateListener<leju::msgs::SensorsData>;
template class HumanoidControllerDDSClient<leju::msgs::JointCmd, leju::msgs::SensorsData>; 