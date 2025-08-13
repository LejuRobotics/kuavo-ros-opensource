#include "humanoid_controllers/CommonDDS.h"

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
DdsLowStateListener::DdsLowStateListener() : message_count_(0) {}

void DdsLowStateListener::on_data_available(dds::sub::DataReader<unitree_hg::msg::dds_::LowState_>& reader) {
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
    
uint64_t DdsLowStateListener::getMessageCount() const { return message_count_.load(); }
    
unitree_hg::msg::dds_::LowState_ DdsLowStateListener::getLatestData() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_state_data_;
}
    
void DdsLowStateListener::setLowdstateCallback(std::function<void(const unitree_hg::msg::dds_::LowState_&)> callback) {
        ext_lowdstate_callback_ = callback;
}

// HumanoidControllerDDSClient implementation
HumanoidControllerDDSClient::HumanoidControllerDDSClient() 
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

HumanoidControllerDDSClient::~HumanoidControllerDDSClient() {
    stop();
}

void HumanoidControllerDDSClient::start() {
    if (running_.exchange(true)) {
        std::cerr << "Client already running" << std::endl;
        return;
    }
}

void HumanoidControllerDDSClient::stop() {
    if (!running_.exchange(false)) {
        return;
    }
}


void HumanoidControllerDDSClient::publishLowCmd(const unitree_hg::msg::dds_::LowCmd_& cmd) {
    cmd_writer_.write(cmd);
}

void HumanoidControllerDDSClient::setupStateListener() {
    state_listener_ = std::make_unique<DdsLowStateListener>();
    state_reader_.listener(state_listener_.get(), 
                          dds::core::status::StatusMask::data_available());
} 