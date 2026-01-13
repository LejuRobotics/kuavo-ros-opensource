#include "mujoco_dds.h"

using namespace org::eclipse::cyclonedds;

// MujocoDdsLowCmdListener implementation
template<typename CmdMessageType>
MujocoDdsLowCmdListener<CmdMessageType>::MujocoDdsLowCmdListener(
    std::function<void(const CmdMessageType&)> callback)
    : callback_(callback), message_count_(0) {}

template<typename CmdMessageType>
void MujocoDdsLowCmdListener<CmdMessageType>::on_data_available(
    dds::sub::DataReader<CmdMessageType>& reader) {
    auto samples = reader.read();
    for (const auto& sample : samples) {
        if (sample.info().valid()) {
            callback_(sample.data());
            message_count_++;
        }
    }
}

// MujocoDdsClient implementation
template<typename CmdType, typename StateType>
MujocoDdsClient<CmdType, StateType>::MujocoDdsClient()
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
    std::cout << "\033[33m[MuJoCo DDS] DDS Client initialized\033[0m" << std::endl;
}

template<typename CmdType, typename StateType>
MujocoDdsClient<CmdType, StateType>::~MujocoDdsClient() {
    stop();
}

template<typename CmdType, typename StateType>
void MujocoDdsClient<CmdType, StateType>::start() {
    if (running_.exchange(true)) {
        std::cerr << "\033[33m[MuJoCo DDS] Client already running\033[0m" << std::endl;
        return;
    }
    std::cout << "\033[33m[MuJoCo DDS] MujocoDdsClient started\033[0m" << std::endl;
}

template<typename CmdType, typename StateType>
void MujocoDdsClient<CmdType, StateType>::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    std::cout << "\033[33m[MuJoCo DDS] DDS communication stopped\033[0m" << std::endl;
}

template<typename CmdType, typename StateType>
void MujocoDdsClient<CmdType, StateType>::setLowCmdCallback(
    std::function<void(const CmdType&)> callback) {
    ext_cmd_callback_ = callback;
}

template<typename CmdType, typename StateType>
void MujocoDdsClient<CmdType, StateType>::publishLowState(const StateType& state) {
    if (!running_.load()) {
        return;
    }

    state_writer_.write(state);
    state_count_.fetch_add(1, std::memory_order_relaxed);
}

template<typename CmdType, typename StateType>
void MujocoDdsClient<CmdType, StateType>::setupCmdListener() {
    auto callback = [this](const CmdType& cmd) {
        this->onLowCmdReceived(cmd);
    };

    cmd_listener_ = std::make_unique<MujocoDdsLowCmdListenerType>(callback);
    cmd_reader_.listener(cmd_listener_.get(),
                        dds::core::status::StatusMask::data_available());
}

template<typename CmdType, typename StateType>
void MujocoDdsClient<CmdType, StateType>::onLowCmdReceived(const CmdType& cmd) {
    if (!isValidDdsLowCommand(cmd)) {
        std::cerr << "\033[33m[MuJoCo DDS] Invalid DDS low command received\033[0m" << std::endl;
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

template<typename CmdType, typename StateType>
bool MujocoDdsClient<CmdType, StateType>::isValidDdsLowCommand(const CmdType& cmd) const {
    // Validate CRC
    uint32_t calculated_crc = Crc32Core((uint32_t*)&cmd, (sizeof(cmd) >> 2) - 1);
    if (cmd.crc() != calculated_crc) {
        std::cerr << "\033[33m[MuJoCo DDS] Command CRC validation failed\033[0m" << std::endl;
        return false;
    }
    
    // Check motor command array size
    if (cmd.motor_cmd().size() != DDS_MOTOR_COUNT) {
        std::cerr << "\033[33m[MuJoCo DDS] Invalid motor command array size\033[0m" << std::endl;
        return false;
    }
    
    return true;
}

// Template function specialization for LowState_
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


// Template function specialization for SensorsData
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
    leju::msgs::SensorsData& dds_state) {
    // Set timestamp
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

    dds_state.header_sec(static_cast<int32_t>(seconds.count()));
    dds_state.header_nanosec(static_cast<uint32_t>(nanoseconds.count()));
    dds_state.sensor_time(now.time_since_epoch().count());

    // Fill joint data
    size_t joint_count = std::min(joint_q.size(), KUAVO_JOINT_COUNT);

    // Resize joint data sequences
    dds_state.joint_data().joint_q().resize(joint_count);
    dds_state.joint_data().joint_v().resize(joint_count);
    dds_state.joint_data().joint_vd().resize(joint_count);
    dds_state.joint_data().joint_torque().resize(joint_count);

    // Fill joint data
    for (size_t i = 0; i < joint_count; ++i) {
        dds_state.joint_data().joint_q()[i] = joint_q[i];
        dds_state.joint_data().joint_v()[i] = joint_v[i];
        dds_state.joint_data().joint_vd()[i] = joint_vd[i];
        dds_state.joint_data().joint_torque()[i] = joint_torque[i];
    }

    // Fill IMU data
    // Gyroscope data (angular velocity)
    dds_state.imu_data().gyro()[0] = angVel[0];
    dds_state.imu_data().gyro()[1] = angVel[1];
    dds_state.imu_data().gyro()[2] = angVel[2];

    // Accelerometer data
    dds_state.imu_data().acc()[0] = acc_eigen[0];
    dds_state.imu_data().acc()[1] = acc_eigen[1];
    dds_state.imu_data().acc()[2] = acc_eigen[2];

    // Free acceleration data
    dds_state.imu_data().free_acc()[0] = free_acc[0];
    dds_state.imu_data().free_acc()[1] = free_acc[1];
    dds_state.imu_data().free_acc()[2] = free_acc[2];

    // Quaternion data (orientation)
    dds_state.imu_data().quat()[0] = ori[0];  // w
    dds_state.imu_data().quat()[1] = ori[1];  // x
    dds_state.imu_data().quat()[2] = ori[2];  // y
    dds_state.imu_data().quat()[3] = ori[3];  // z

    // End effector data - leave as default/empty since MuJoCo doesn't provide this
    // The end_effector_data field will be initialized with default values

    // std::cout << "ConvertMujocoToDdsState  leju DDS \n";
}

// Template specialization for isValidDdsLowCommand with leju::msgs::JointCmd
template<>
bool MujocoDdsClient<leju::msgs::JointCmd, leju::msgs::SensorsData>::isValidDdsLowCommand(const leju::msgs::JointCmd& cmd) const {
    // For leju::msgs::JointCmd, validate joint command arrays
    size_t expected_joint_count = KUAVO_JOINT_COUNT;

    // Check joint array sizes
    if (cmd.joint_q().size() != expected_joint_count) {
        std::cerr << "\033[33m[MuJoCo DDS] Invalid joint_q array size: expected "
                  << expected_joint_count << ", got " << cmd.joint_q().size() << "\033[0m" << std::endl;
        return false;
    }

    if (cmd.joint_v().size() != expected_joint_count) {
        std::cerr << "\033[33m[MuJoCo DDS] Invalid joint_v array size: expected "
                  << expected_joint_count << ", got " << cmd.joint_v().size() << "\033[0m" << std::endl;
        return false;
    }

    if (cmd.tau().size() != expected_joint_count) {
        std::cerr << "\033[33m[MuJoCo DDS] Invalid tau array size: expected "
                  << expected_joint_count << ", got " << cmd.tau().size() << "\033[0m" << std::endl;
        return false;
    }

    // Check control modes array size
    if (cmd.control_modes().size() != expected_joint_count) {
        std::cerr << "\033[33m[MuJoCo DDS] Invalid control_modes array size: expected "
                  << expected_joint_count << ", got " << cmd.control_modes().size() << "\033[0m" << std::endl;
        return false;
    }

    // Validate timestamp (basic check)
    if (cmd.header_sec() < 0 || cmd.header_nanosec() >= 1000000000) {
        std::cerr << "\033[33m[MuJoCo DDS] Invalid timestamp in JointCmd\033[0m" << std::endl;
        return false;
    }

    return true;
}

// Explicit template instantiation for the specific message types
template class MujocoDdsLowCmdListener<unitree_hg::msg::dds_::LowCmd_>;
template class MujocoDdsClient<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>;


// Explicit template instantiation for Leju DDS types
template class MujocoDdsLowCmdListener<leju::msgs::JointCmd>;
template class MujocoDdsClient<leju::msgs::JointCmd, leju::msgs::SensorsData>; 