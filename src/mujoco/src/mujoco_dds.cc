#include "mujoco_dds.h"

using namespace org::eclipse::cyclonedds;

// MujocoDdsLowCmdListener implementation
MujocoDdsLowCmdListener::MujocoDdsLowCmdListener(
    std::function<void(const unitree_hg::msg::dds_::LowCmd_&)> callback)
    : callback_(callback), message_count_(0) {}

void MujocoDdsLowCmdListener::on_data_available(
    dds::sub::DataReader<unitree_hg::msg::dds_::LowCmd_>& reader) {
    auto samples = reader.read();
    for (const auto& sample : samples) {
        if (sample.info().valid()) {
            callback_(sample.data());
            message_count_++;
        }
    }
}

// MujocoDdsClient implementation
MujocoDdsClient::MujocoDdsClient() 
    : participant_(org::eclipse::cyclonedds::domain::default_id())
    , cmd_topic_(participant_, DDS_CMD_TOPIC)
    , state_topic_(participant_, DDS_STATE_TOPIC)
    , cmd_reader_(dds::sub::Subscriber(participant_), cmd_topic_)
    , state_writer_(dds::pub::Publisher(participant_), state_topic_)
    , running_(false)
    , has_new_cmd_(false)
    , cmd_count_(0)
    , state_count_(0)
{
    setupCmdListener();
    std::cout << "[MuJoCo DDS] DDS Client initialized" << std::endl;
}

MujocoDdsClient::~MujocoDdsClient() {
    stop();
}

void MujocoDdsClient::start() {
    if (running_.exchange(true)) {
        std::cerr << "[MuJoCo DDS] Client already running" << std::endl;
        return;
    }
    std::cout << "[MuJoCo DDS] DDS communication started" << std::endl;
}

void MujocoDdsClient::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    std::cout << "[MuJoCo DDS] DDS communication stopped" << std::endl;
}

void MujocoDdsClient::setLowCmdCallback(
    std::function<void(const unitree_hg::msg::dds_::LowCmd_&)> callback) {
    ext_cmd_callback_ = callback;
}

void MujocoDdsClient::publishLowState(const unitree_hg::msg::dds_::LowState_& state) {
    if (!running_.load()) {
        return;
    }
    
    state_writer_.write(state);
    state_count_.fetch_add(1, std::memory_order_relaxed);
}

void MujocoDdsClient::setupCmdListener() {
    auto callback = [this](const unitree_hg::msg::dds_::LowCmd_& cmd) {
        this->onLowCmdReceived(cmd);
    };
    
    cmd_listener_ = std::make_unique<MujocoDdsLowCmdListener>(callback);
    cmd_reader_.listener(cmd_listener_.get(), 
                        dds::core::status::StatusMask::data_available());
}

void MujocoDdsClient::onLowCmdReceived(const unitree_hg::msg::dds_::LowCmd_& cmd) {
    if (!isValidDdsLowCommand(cmd)) {
        std::cerr << "[MuJoCo DDS] Invalid DDS low command received" << std::endl;
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_cmd_ = cmd;
    }
    has_new_cmd_.store(true, std::memory_order_release);
    cmd_count_.fetch_add(1, std::memory_order_relaxed);
    
    // Call external callback if set
    if (ext_cmd_callback_) {
        ext_cmd_callback_(cmd);
    }
}

bool MujocoDdsClient::isValidDdsLowCommand(const unitree_hg::msg::dds_::LowCmd_& cmd) const {
    // Validate CRC
    uint32_t calculated_crc = Crc32Core((uint32_t*)&cmd, (sizeof(cmd) >> 2) - 1);
    if (cmd.crc() != calculated_crc) {
        std::cerr << "[MuJoCo DDS] Command CRC validation failed" << std::endl;
        return false;
    }
    
    // Check motor command array size
    if (cmd.motor_cmd().size() != DDS_MOTOR_COUNT) {
        std::cerr << "[MuJoCo DDS] Invalid motor command array size" << std::endl;
        return false;
    }
    
    return true;
}

void MujocoDdsClient::convertMujocoToDdsState(const std::vector<double>& joint_q, const std::vector<double>& joint_v, const std::vector<double>& joint_vd, const std::vector<double>& joint_torque, const Eigen::Vector3d& acc_eigen, const Eigen::Vector3d& angVel, const Eigen::Vector3d& free_acc, const Eigen::Vector4d& ori,
                                                        unitree_hg::msg::dds_::LowState_& dds_state) {
    // Set timestamp using reserve fields to avoid uint32_t overflow
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
    
    // Use reserve fields for timestamp: [0]=seconds, [1]=nanoseconds
    dds_state.reserve()[0] = static_cast<uint32_t>(seconds.count());
    dds_state.reserve()[1] = static_cast<uint32_t>(nanoseconds.count());
    
    // Keep tick as a simple counter or relative timestamp
    static uint32_t tick_counter = 0;
    dds_state.tick(++tick_counter);
    
    // Initialize ALL 35 motor states with zeros first
    for (size_t motor_idx = 0; motor_idx < DDS_MOTOR_COUNT; ++motor_idx) {
        auto& motor_state = dds_state.motor_state()[motor_idx];
        motor_state.mode(0);
        motor_state.q(0.0f);
        motor_state.dq(0.0f);
        motor_state.ddq(0.0f);
        motor_state.tau_est(0.0f);
        motor_state.temperature({{0, 0}});
        motor_state.vol(0.0f);
        motor_state.sensor({{0, 0}});
        motor_state.motorstate(0);
    }
    
    // Fill motor data with same logic as ROS joint_data
    // Use the same joint mapping and data as publish_ros_data function
    size_t joint_count = std::min(joint_q.size(), KUAVO_JOINT_COUNT);
    
    for (size_t i = 0; i < joint_count; ++i) {
        auto& motor_state = dds_state.motor_state()[i];
        
        // Same data mapping as ROS joint_data
        motor_state.mode(1);  
        motor_state.q(static_cast<float>(joint_q[i]));
        motor_state.dq(static_cast<float>(joint_v[i]));
        motor_state.ddq(static_cast<float>(joint_vd[i]));
        motor_state.tau_est(static_cast<float>(joint_torque[i]));
        motor_state.temperature({{0, 0}});  // Not available in MuJoCo
        motor_state.vol(0.0f);              // Not available in MuJoCo
        motor_state.sensor({{0, 0}});       // Not available in MuJoCo
        motor_state.motorstate(1);
    }
    
    
    // Set IMU data (same as ROS) - only use fields that actually exist in IMUState_
    dds_state.imu_state().accelerometer()[0] = static_cast<float>(acc_eigen[0]);
    dds_state.imu_state().accelerometer()[1] = static_cast<float>(acc_eigen[1]);
    dds_state.imu_state().accelerometer()[2] = static_cast<float>(acc_eigen[2]);
    
    dds_state.imu_state().gyroscope()[0] = static_cast<float>(angVel[0]);
    dds_state.imu_state().gyroscope()[1] = static_cast<float>(angVel[1]);
    dds_state.imu_state().gyroscope()[2] = static_cast<float>(angVel[2]);
    
    dds_state.imu_state().quaternion()[0] = static_cast<float>(ori[0]);  // w
    dds_state.imu_state().quaternion()[1] = static_cast<float>(ori[1]);  // x
    dds_state.imu_state().quaternion()[2] = static_cast<float>(ori[2]);  // y
    dds_state.imu_state().quaternion()[3] = static_cast<float>(ori[3]);  // z
    
    // Map free_acc to rpy field (as requested)
    dds_state.imu_state().rpy()[0] = static_cast<float>(free_acc[0]);
    dds_state.imu_state().rpy()[1] = static_cast<float>(free_acc[1]);
    dds_state.imu_state().rpy()[2] = static_cast<float>(free_acc[2]);
    
    dds_state.imu_state().temperature(20);  // Mock temperature
    
    // Note: LowState_ does NOT have base_state field, only motor_state and imu_state
    // Base position/velocity info is not part of DDS LowState protocol
} 