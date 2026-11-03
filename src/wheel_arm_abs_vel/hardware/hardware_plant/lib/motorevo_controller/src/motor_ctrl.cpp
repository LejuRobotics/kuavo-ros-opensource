#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_log.h"
#include "canbus_sdk/canbus_sdk.h"
#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <unordered_set>

namespace motorevo {

static constexpr uint8_t kEnterMotorStateFrame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static constexpr uint8_t kEnterResetStateFrame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
static constexpr uint8_t kSetZeroPositionFrame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
static constexpr uint8_t kMultiTurnZeroFrame[8] = {0x67, 0x06, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x76}; // 多圈编码器清零

// 零点阈值常量
static constexpr float kZeroThresholdRadians = 0.5f * M_PI / 180.0f;  // 0.5度 = 0.0087弧度

// 检查位置是否存在异常跳变
static bool hasPositionJump(float previous_pos, float current_pos) {
    // 根据Motorevo电机特性，检查位置是否存在异常跳变
    // 设置安全的跳变阈值，考虑CAN通讯可能丢几帧的情况

    const float POSITION_JUMP_THRESHOLD = 0.3f;  // 位置跳变阈值（弧度）
    const float EPSILON = 1e-10f;                // 浮点数比较精度

    // 检查输入值的有效性
    if (std::isnan(current_pos) || std::isinf(current_pos) ||
        std::isnan(previous_pos) || std::isinf(previous_pos)) {
        return true;  // 无效输入认为是异常
    }

    // 计算位置变化的绝对值
    float position_change = std::abs(current_pos - previous_pos);

    // 检查是否超过阈值
    return position_change > POSITION_JUMP_THRESHOLD;
}

static bool sendCanMessage(uint8_t canbus_id, uint32_t can_id, const uint8_t* payload) {
    using namespace canbus_sdk;
    CanMessageFrame can_frame{};
    can_frame.id.SID = can_id;
    can_frame.ctrl.tx.dlc = payload_length_to_dlc(8);
    memcpy(can_frame.payload, payload, 8);
    Result<ErrorType> result = CanBusController::getInstance().sendMessage(canbus_id, can_frame);
    if (result.has_value()) {
        return result.value() == 0;
    }
    return false;
}

// RT 直发路径：绕过 ring 队列，由 sender 线程的 RT slot 直接写 CAN
// 注意：在 RT 源回调（rtWrite → controlPTM）内调用，此时 sender 线程已持 blocks_mutex_，
// 因此必须使用不加锁的 sendMessageRtUnlocked（若用 sendMessageRt 会自锁死锁）
static bool sendCanMessageRt(uint8_t canbus_id, uint32_t can_id, const uint8_t* payload) {
    using namespace canbus_sdk;
    CanMessageFrame can_frame{};
    can_frame.id.SID = can_id;
    can_frame.ctrl.tx.dlc = payload_length_to_dlc(8);
    memcpy(can_frame.payload, payload, 8);
    Result<ErrorType> result = CanBusController::getInstance().sendMessageRtUnlocked(canbus_id, can_frame);
    if (result.has_value()) {
        return result.value() == 0;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////
// Class FeedbackFrame
///////////////////////////////////////////////////////////////////////////////
/**
* See Details:《Motorevo Driver User Guide v0.2.3》- 6.5.1 反馈帧格式
* 无论在什么状态和模式下，驱动板在收到报文标识符为 CAN COM ID 的报文时都会进行一
* 次应答，反馈当前电机的位置(rad)，速度(rad/s)，扭矩(N·m)。应答的报文标识符恒定本机
* CAN ID，波特率为 1Mbps，DLC 为 8 字节。
* +---------+---------+---------+---------+---------+---------+---------+---------+---------+
* | Byte0   | Byte1   | Byte2   | Byte3   | Byte4_H |Byte4_L  | Byte5   | Byte6_H | Byte7   |
* +---------+---------+---------+---------+---------+---------+---------+---------+---------+
* | Motor   |position |position |velocity |velocity | torque  | torque  | errcode | tempe-  |
* | Id      | High    | Low     | High    | Low     | High    | Low     |         | rature  |
* | 8bits   | 8bits   | 8bits   | 8bits   | 4bits   | 4bits   | 8bits   |         |         |
* +---------+---------+---------+---------+---------+---------+---------+---------+---------+
*/
FeedbackFrame::FeedbackFrame() {
    memset(payload_, 0, 8);
}

FeedbackFrame::FeedbackFrame(const uint8_t payload[8])
{
    memcpy(payload_, payload, 8);
}

FeedbackFrame::FeedbackFrame(const FeedbackFrame& other)
{
    std::memcpy(payload_, other.payload_, sizeof(payload_));
}

FeedbackFrame& FeedbackFrame::operator=(const FeedbackFrame& other)
{
    if (this != &other) {
        std::memcpy(payload_, other.payload_, sizeof(payload_));
    }
    return *this;
}

void FeedbackFrame::reset()
{
    memset(payload_, 0, 8);
}
void FeedbackFrame::set_payload(const uint8_t payload[8])
{
    memcpy(payload_, payload, 8);
}

float FeedbackFrame::position() const
{
    uint32_t pos = (payload_[1] << 8) | payload_[2];
    return int_to_float<16>(pos, kCAN_COM_THETA_MIN, kCAN_COM_THETA_MAX);
}

float FeedbackFrame::velocity() const
{
    uint32_t vel = (payload_[3] << 4) | (payload_[4] >> 4);
    return int_to_float<12>(vel, kCAN_COM_VELOCITY_MIN, kCAN_COM_VELOCITY_MAX);
}

float FeedbackFrame::torque() const
{
   uint32_t torque = ((payload_[4] & 0x0F) << 8) | payload_[5];
   return int_to_float<12>(torque, kCAN_COM_TORQUE_MIN, kCAN_COM_TORQUE_MAX);
}
uint8_t FeedbackFrame::temperature() const
{
    return payload_[7];
}

MotorErrCode FeedbackFrame::errcode() const
{
    return static_cast<MotorErrCode>(payload_[6]);
}

MotorId FeedbackFrame::motor_id() const
{
    return static_cast<MotorId>(payload_[0]);
}

///////////////////////////////////////////////////////////////////////////////
// Class RevoMotor
///////////////////////////////////////////////////////////////////////////////

struct RevoMotor::data_t {
    std::atomic<float> position_raw;    // 位置(rad)，原始数据
    std::atomic<float> velocity_raw;    // 速度(rad/s)，原始数据
    std::atomic<float> torque_raw;      // 扭矩(N·m)，原始数据
    std::atomic<uint8_t> temperature;   // 温度(℃)
    std::atomic<MotorErrCode> errcode;  // 故障码

    data_t(): position_raw(0.0), velocity_raw(0.0), torque_raw(0.0), temperature(0), errcode(MotorErrCode::NO_FAULT) {}
    data_t(const data_t& other) :
        position_raw(other.position_raw.load()),
        velocity_raw(other.velocity_raw.load()),
        torque_raw(other.torque_raw.load()),
        temperature(other.temperature.load()),
        errcode(other.errcode.load()) {}

    data_t& operator=(const data_t& other) {
        if (this != &other) {
            position_raw.store(other.position_raw.load());
            velocity_raw.store(other.velocity_raw.load());
            torque_raw.store(other.torque_raw.load());
            temperature.store(other.temperature.load());
            errcode.store(other.errcode.load());
        }
        return *this;
    }
};

RevoMotor::RevoMotor(uint8_t canbus_id, MotorId id, MotorMode mode)
    : state_(MotorState::None), ctrl_mode_(mode),
    canbus_id_(canbus_id), id_(id), data_(new data_t()) {
}

void RevoMotor::setRtSend(bool enable) {
    rt_send_.store(enable, std::memory_order_release);
}

RevoMotor::RevoMotor(const RevoMotor& other)
    : state_(other.state_),
      ctrl_mode_(other.ctrl_mode_),
      data_(new data_t(*other.data_)),
      canbus_id_(other.canbus_id_),
      id_(other.id_),
      rt_send_(other.rt_send_.load(std::memory_order_acquire)) {
}

RevoMotor::RevoMotor(RevoMotor&& other) noexcept
    : state_(other.state_),
      ctrl_mode_(other.ctrl_mode_),
      data_(other.data_),
      canbus_id_(other.canbus_id_),
      id_(other.id_),
      rt_send_(other.rt_send_.load(std::memory_order_acquire)) {
    other.data_ = nullptr;
}

RevoMotor& RevoMotor::operator=(const RevoMotor& other) {
    if (this != &other) {
        state_ = other.state_;
        ctrl_mode_ = other.ctrl_mode_;
        if (data_ != nullptr) {
            delete data_;
        }
        data_ = new data_t(*other.data_);
        canbus_id_ = other.canbus_id_;
        id_ = other.id_;
        rt_send_.store(other.rt_send_.load(std::memory_order_acquire), std::memory_order_release);
    }
    return *this;
}

RevoMotor& RevoMotor::operator=(RevoMotor&& other) noexcept {
    if (this != &other) {
        state_ = other.state_;
        ctrl_mode_ = other.ctrl_mode_;
        if (data_ != nullptr) {
            delete data_;  // 删除当前对象的数据
        }
        data_ = other.data_;  // 接管源对象的数据
        canbus_id_ = other.canbus_id_;
        id_ = other.id_;
        rt_send_.store(other.rt_send_.load(std::memory_order_acquire), std::memory_order_release);
        other.data_ = nullptr;  // 将源对象指针设为 nullptr
    }
    return *this;
}

uint32_t RevoMotor::getId() const {
    return id_;
}

MotorState RevoMotor::getState() const {
    return state_;
}

MotorMode RevoMotor::getControlMode() const {
    return ctrl_mode_;
}

float RevoMotor::position() const {
    return data_->position_raw.load();
}

float RevoMotor::torque() const {
    return data_->torque_raw.load();
}

float RevoMotor::velocity() const {
    return data_->velocity_raw.load();
}

uint8_t RevoMotor::temperature() const {
    return data_->temperature.load();
}

MotorErrCode RevoMotor::getErrorCode() const {
    return data_->errcode.load();
}

bool RevoMotor::receiveFeedback(const FeedbackFrame& frame) {
    /**
    * See Details:《Motorevo Driver User Guide v0.2.3》- 6.5.1 反馈帧格式
    * 无论在什么状态和模式下，驱动板在收到报文标识符为 CAN COM ID 的报文时都会进行一
    * 次应答，反馈当前电机的位置(rad)，速度(rad/s)，扭矩(N·m)。应答的报文标识符恒定本机
    * CAN ID，波特率为 1Mbps，DLC 为 8 字节。
    * +---------+---------+---------+---------+---------+---------+---------+---------+---------+
    * | Byte0   | Byte1   | Byte2   | Byte3   | Byte4_H |Byte4_L  | Byte5   | Byte6_H | Byte7   |
    * +---------+---------+---------+---------+---------+---------+---------+---------+---------+
    * | Motor   |position |position |velocity |velocity | torque  | torque  | errcode | tempe-  |
    * | Id      | High    | Low     | High    | Low     | High    | Low     |         | rature  |
    * | 8bits   | 8bits   | 8bits   | 8bits   | 4bits   | 4bits   | 8bits   |         |         |
    * +---------+---------+---------+---------+---------+---------+---------+---------+---------+
    */
    // 双重判断:报文标识符和电机ID匹配 (CAN_ID + MOTOR_ID)
    if(frame.motor_id() != id_) {
        printf("Motor feedback frame id mismatch: 0x%02X != 0x%02X\n", frame.motor_id(), id_);
        return false;
    }

    data_->position_raw.store(frame.position());
    data_->velocity_raw.store(frame.velocity());
    data_->torque_raw.store(frame.torque());
    data_->temperature.store(frame.temperature());
    data_->errcode.store(frame.errcode());

    return true;
}

RevoMotor::~RevoMotor() {
    if (data_ != nullptr) {
        delete data_;
        data_ = nullptr;
    }
}

bool RevoMotor::enterMotorState() {
    return  sendCanMessage(canbus_id_, id_, kEnterMotorStateFrame);
}

bool RevoMotor::enterRestState() {
    return  sendCanMessage(canbus_id_, id_, kEnterResetStateFrame);
}


bool RevoMotor::setZeroPosition() {
    return  sendCanMessage(canbus_id_, id_, kSetZeroPositionFrame);
}

bool RevoMotor::multiTurnZero() {
    uint32_t can_id = id_ + 0x600;
    return  sendCanMessage(canbus_id_, can_id, kMultiTurnZeroFrame);
}

/**
 * See Details:《Motorevo Driver User Guide v0.2.3》- 6.3.2 使用 CAN 控制电机在力位混合模式（P-T-M Mode）运行
 *
 * 控制帧为 CAN 标准帧，帧 ID 为 Motor ID，波特率为 1Mhz，控制帧和控制框图的映射关系定义如下:
 *
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 * | Byte0   | Byte1   | Byte2   | Byte3_H | Byte3_L | Byte4   | Byte5   | Byte6_H | Byte6_L | Byte7   |
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 * | θ_ref   | θ_ref   | V_ref   | V_ref   | Kp      | Kp      | Kd      | Kd      | T_ref   | T_ref   |
 * | High    | Low     | High    | Low     | High    | Low     | High    | Low     | High    | Low     |
 * | 8bits   | 8bits   | 8bits   | 4bits   | 4bits   | 8bits   | 8bits   | 4bits   | 4bits   | 8bits   |
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 */
bool RevoMotor::controlPTM(float pos, float vel, float torque, float pos_kp, float pos_kd,
                         kuavo_e2e::Trace<32>::Row* observation) {
    if (ctrl_mode_ != MotorMode::TorquePositionMixControlMode) {
        RWLOG_E("Motor is not in P-T-M mode");
        return false;
    }

    uint8_t payload[8]{};

    uint16_t theta_ref = float_to_int<16>(pos, kCAN_COM_THETA_MIN, kCAN_COM_THETA_MAX);
    uint16_t V_ref = float_to_int<12>(vel, kCAN_COM_VELOCITY_MIN, kCAN_COM_VELOCITY_MAX);
    uint16_t Kp    = float_to_int<12>(pos_kp, kCAN_COM_POS_KP_MIN, kCAN_COM_POS_KP_MAX);
    uint16_t Kd    = float_to_int<12>(pos_kd, kCAN_COM_POS_KD_MIN, kCAN_COM_POS_KD_MAX);
    uint16_t T_ref = float_to_int<12>(torque, kCAN_COM_TORQUE_MIN, kCAN_COM_TORQUE_MAX);
    if(observation) {
        auto& r=*observation;r[6]=pos;r[7]=vel;r[8]=torque;r[9]=pos_kp;r[10]=pos_kd;
        r[11]=theta_ref;r[12]=V_ref;r[13]=T_ref;r[14]=Kp;r[15]=Kd;
        r[16]=int_to_float<12>(Kp,kCAN_COM_POS_KP_MIN,kCAN_COM_POS_KP_MAX);
        r[17]=int_to_float<12>(Kd,kCAN_COM_POS_KD_MIN,kCAN_COM_POS_KD_MAX);
        r[18]=canbus_id_;r[19]=id_;r[20]=1; // PTM encoding reached; not a motor ACK.
    }

    // θ_ref: Byte 0, Byte1
    payload[0] = (theta_ref >> 8) & 0xFF;
    payload[1] = theta_ref & 0xFF;
    // V_ref: Byte2, Byte3_H
    payload[2] = (V_ref >> 4) & 0xFF;
    payload[3] = ((V_ref & 0x0F) << 4) | ((Kp >> 8) & 0x0F);
    // Kp: Byte3_L, Byte4
    payload[4] = Kp & 0xFF;
    // Kd: Byte5, Byte6_H
    payload[5] = (Kd >> 4) & 0xFF;
    payload[6] = ((Kd & 0x0F) << 4) | ((T_ref >> 8) & 0x0F);
    // T_ref: Byte6_L, Byte7
    payload[7] = T_ref & 0xFF;

    if(observation) (*observation)[30]=kuavo_e2e::ns();
    const bool sent = rt_send_.load(std::memory_order_acquire)
        ? sendCanMessageRt(canbus_id_, id_, payload)
        : sendCanMessage(canbus_id_, id_, payload);
    if(observation) (*observation)[31]=kuavo_e2e::ns();
    return sent;
}

// Retain the original exported symbol for existing non-observing callers,
// including the bound gripper library. Encoding/send remain a single path.
bool RevoMotor::controlPTM(float pos, float vel, float torque, float pos_kp, float pos_kd) {
    return controlPTM(pos, vel, torque, pos_kp, pos_kd, nullptr);
}

/**
 * See Details:《Motorevo Driver User Guide v0.2.3》 - 6.3.3 使用 CAN 控制电机在速度模式（Velocity Mode）运行
 *
 * 控制帧为 CAN 标准帧，帧 ID 为 Motor ID，波特率为 1Mhz，控制帧和控制框图的映射关系定义如下:
 *
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 * | Byte0   | Byte1   | Byte2   | Byte3_H | Byte3_L | Byte4   | Byte5   | Byte6_H | Byte6_L | Byte7   |
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 * | V_ref   | V_ref   | Kp      | Kp      | Kd      | Kd      | Ki      | Ki      | unused  | 0xAC    |
 * | High    | Low     | High    | Low     | High    | Low     | High    | Low     |         | Identi~ |
 * | 8bits   | 8bits   | 8bits   | 4bits   | 4bits   | 8bits   | 8bits   | 4bits   |         |         |
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 */
bool RevoMotor::controlVelocity(float vel, float vel_kp, float vel_ki, float vel_kd) {
    if(ctrl_mode_ != MotorMode::VelocityMode) {
        RWLOG_E("Motor is not in Velocity Mode");
    }

    uint8_t payload[8]{};

    constexpr float kCAN_COM_VEL_KP_MIN = 0.0f;
    constexpr float kCAN_COM_VEL_KP_MAX = 250.0f;
    constexpr float kCAN_COM_VEL_KD_MIN = 0.0f;
    constexpr float kCAN_COM_VEL_KD_MAX = 50.0f;
    constexpr float kCAN_COM_VEL_KI_MIN = 0.0f;
    constexpr float kCAN_COM_VEL_KI_MAX = 0.05f;

    uint16_t V_ref = float_to_int<16>(vel, kCAN_COM_VELOCITY_MIN, kCAN_COM_VELOCITY_MAX);
    uint16_t Kp    = float_to_int<12>(vel_kp, kCAN_COM_VEL_KP_MIN, kCAN_COM_VEL_KP_MAX);
    uint16_t Kd    = float_to_int<12>(vel_kd, kCAN_COM_VEL_KD_MIN, kCAN_COM_VEL_KD_MAX);
    uint16_t Ki    = float_to_int<12>(vel_ki, kCAN_COM_VEL_KI_MIN, kCAN_COM_VEL_KI_MAX);

    payload[0] = (V_ref >> 8) & 0xFF;
    payload[1] = V_ref & 0xFF;
    payload[2] = (Kp >> 4) & 0xFF;
    payload[3] = ((Kp & 0x0F) << 4) | ((Kd >> 8) & 0x0F);
    payload[4] = Kd & 0xFF;
    payload[5] = (Ki >> 4) & 0xFF;
    payload[6] = (Ki & 0x0F) << 4;
    payload[7] = 0xAC;

    return sendCanMessage(canbus_id_, id_, payload);
}

/**
 * See Details:《Motorevo Driver User Guide v0.2.3》 - 使用 CAN 控制电机在力矩模式（Torque Mode）运行
 *
 * 控制帧为 CAN 标准帧，帧 ID 为 Motor ID，波特率为 1Mhz，控制帧和控制框图的映射关系定义如下:
 *
 * +--------+--------+--------+--------+--------+--------+--------+----------------+
 * | Byte0  | Byte1  | Byte2  | Byte3  | Byte4  | Byte5  | Byte6  | Byte7          |
 * +--------+--------+--------+--------+--------+--------+--------+----------------+
 * | T_ref  | T_ref  | unused | unused | unused | unused | unused | 0xAB           |
 * | High   | Low    |        |        |        |        |        | Identification |
 * +--------+--------+--------+--------+--------+--------+--------+----------------+
 */
bool RevoMotor::controlTorque(float torque) {
    if(ctrl_mode_ != MotorMode::TorqueMode) {
        RWLOG_E("Motor is not in Torque Mode");
        return false;
    }
    uint8_t payload[8]{};

    uint16_t T_ref = float_to_int<16>(torque, kCAN_COM_TORQUE_MIN, kCAN_COM_TORQUE_MAX);

    payload[0] = (T_ref >> 8) & 0xFF;
    payload[1] = T_ref & 0xFF;
    payload[7] = 0xAB;

    return sendCanMessage(canbus_id_, id_, payload);
}


///////////////////////////////////////////////////////////////////////////////
// RevoMotorControl
///////////////////////////////////////////////////////////////////////////////

RevoMotorControl::RevoMotorControl(const std::string& canbus_name) : canbus_name_(canbus_name),
    command_trace_(("motor_boundary_"+canbus_name).c_str()), zero_torque_mode_(false) {
}

bool RevoMotorControl::initializeCanBusAndDevices(const std::vector<RevoMotorConfig_t>& motor_configs) {
    // 获取CAN总线控制器实例
    auto& canbus_controller = canbus_sdk::CanBusController::getInstance();

    // 获取总线ID
    auto bus_id_result = canbus_controller.getBusIdByName(canbus_name_);
    if (!bus_id_result.has_value()) {
        RWLOG_E("Failed to get bus ID for CAN bus %s", canbus_name_.c_str());
        return false;
    }

    uint8_t bus_id = bus_id_result.value();
    RWLOG_I("Using CAN bus %s with bus_id: %d", canbus_name_.c_str(), bus_id);

    // 初始化回调上下文
    msg_callback_context_.userdata = this;
    tef_callback_context_.userdata = this;

    // 注册CAN消息回调
    canbus_sdk::CallbackParams callback_params{
        .msg_callback = internalMessageCallback,
        .msg_cb_ctx = &msg_callback_context_,
        .tef_callback = internalTefEventCallback,
        .tef_cb_ctx = &tef_callback_context_
    };

    // 遍历电机配置，创建电机实例并注册设备
    for (const auto& config : motor_configs) {
        // 注册设备到CAN总线
        if (!config.ignore) {
            auto register_result = canbus_controller.registerDevice(
                canbus_sdk::DeviceInfo(config.name.c_str(), canbus_sdk::DeviceType::MOTOR, static_cast<canbus_sdk::DeviceId>(config.id), nullptr),
                canbus_name_,
                callback_params
            );

            if (!register_result.has_value()) {
                RWLOG_E("Failed to register device %s (ID: %d): %s",
                         config.name.c_str(), config.id,
                         canbus_sdk::errorToString(register_result.error()));
                return false;
            }

            RWLOG_I("Device registered successfully: %s (ID: %d)", config.name.c_str(), config.id);
        }

        // 创建电机实例，使用默认模式，后续可以根据配置设置具体模式
        RevoMotor motor(bus_id, config.id, MotorMode::TorquePositionMixControlMode);

        // 初始化电机控制数据
        motor_ctrl_datas_.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(config.id),
            std::forward_as_tuple(false, std::move(motor), config)
        );
    }

    return true;
}

bool RevoMotorControl::init(const std::vector<RevoMotorConfig_t> &motor_configs, bool calibrate,
                            bool skip_move_to_zero) {
    // 初始化CAN总线并注册电机设备
    if (!initializeCanBusAndDevices(motor_configs)) {
        return false;
    }
    // Roll back partial enable/calibration/feedback failures, including exceptions.
    struct InitRollback {
        RevoMotorControl* control;
        bool complete{false};
        ~InitRollback() {
            if (!complete) {
                try { control->disableAll(); }
                catch (...) { RWLOG_FAILURE("Initialization rollback failed; physical E-stop required"); }
            }
        }
    } rollback{this};

    // 零点校准: 先发送多圈编码器清零
    if (calibrate) {
        multiTurnZeroAll();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    /*
     * 不管有没有故障码都执行 Enter Reset State  清除一下
     * from: 《Motorevo Driver User Guide v0.2.3》- 6.5.1 反馈帧格式 - 故障码
     * notes: "一旦驱动板检测到故障，将会从 Motor State 自动切回 Rest State 以保护驱动器和电机，同
     *        时将返回故障码，提示用户电机运行异常。此时您应仔细检查电机运行工况是否正常，在排
     *        除异常情况后，发送 Enter Rest State 命令清除故障码"
     */
    if(!disableAll()) {
        RWLOG_FAILURE("disableAll() 失败--->无法正常初始运行");
        return false;
    }
    
    // 使能所有电机
    if(!enableAll()) {
        RWLOG_FAILURE("使能所有电机失败--->无法正常初始运行");
        return false;
    }

    // 等待所有电机第一个反馈到达
    if (!waitForAllMotorsFeedback(1500)) {
        RWLOG_FAILURE("Initial feedback missing; aborting initialization without zero movement");
        for (const auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore && !data.feedback_received) {
                RWLOG_WARNING("电机 ID %u (%s) 没有收到初始反馈", id, data.config.name.c_str());
            }
        }
        return false;
    }

    RWLOG_I("==================== MotorRevo Calibration: %s ====================", calibrate ? "True" : "False");
    if (calibrate) {
        // 校准零点 --> 把当前电机的位置设置为零点
        if(!calibrateMotors()) return false;
    }
    else {
        // 检查并更新带有magic number的零点偏移
        // 先获取所有电机的当前位置
        auto raw_positions = getRawPositions();
        // 遍历判断并更新offset
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (data.isEnabled() && !data.config.ignore && std::abs(data.config.zero_offset - kOFFSET_MAGIC_NUMBER) < 1e-6f) {
                // 使用当前位置作为零点偏移
                auto it = raw_positions.find(id);
                if (it != raw_positions.end()) {
                    float current_raw_position = it->second;
                    data.config.zero_offset = data.config.negtive? -current_raw_position : current_raw_position;
                    RWLOG_WARNING("Saving motor ID %u zero offset: %.6f", id, current_raw_position);
                }
            }
        }
    }

    // 打印电机原始位置
    auto raw_positions = getRawPositions();
    std::cout << "\033[32m电机 IDs: \033[0m\n";
    for (const auto& [id, pos] : raw_positions) {
        std::cout << id << " ";
    }
    std::cout << std::endl;

    std::cout << "\033[32m电机原始位置: \033[0m\n";
    for (const auto& [id, pos] : raw_positions) {
        std::cout << std::fixed << std::setprecision(4) << pos << " ";
    }
    std::cout << std::endl;

    // 打印电机+上零点后的位置
    auto positions = getPositions();
    std::cout << "\033[32m电机加上零点后的位置: \033[0m\n";
    for (const auto& [id, pos] : positions) {
        std::cout << std::fixed << std::setprecision(4) << pos << " ";
    }
    std::cout << std::endl;

    if (!skip_move_to_zero) {
        const auto zero_status = moveToZero();
        if (zero_status != OperationStatus::SUCCESS && zero_status != OperationStatus::TIMEOUT) {
            RWLOG_FAILURE("Zero movement failed; aborting initialization");
            return false;
        }
        // A settling timeout must not hide a motor fault or lost enable state.
        for (const auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore && !data.isEnabled()) {
                RWLOG_FAILURE("Motor ID %u is not enabled/healthy after boot positioning", id);
                return false;
            }
        }
        if (zero_status == OperationStatus::TIMEOUT) {
            RWLOG_WARNING("Boot positioning has not settled; retaining the original startup flow. "
                          "This is NOT an all-motors-at-target acknowledgement");
        }
    }

    // 打印电机归零后的位置
    auto final_positions = getPositions();

    std::cout << "\033[32m电机回零后的位置(误差大于0.5度报红):\033[0m\n";
    for (const auto& [id, pos] : final_positions) {
        if (std::abs(pos) > kZeroThresholdRadians) {
            std::cout << "\033[31m" << std::fixed << std::setprecision(4) << pos << "\033[0m ";
        } else {
            std::cout << std::fixed << std::setprecision(4) << pos << " ";
        }
    }
    std::cout << std::endl;

    rollback.complete = true;
    return true;
}

int RevoMotorControl::enableMotor(MotorId id, int timeout_ms) {
    /**
     * @brief 使能指定电机
     * @param id 电机ID
     * @param timeout_ms 超时时间(毫秒)
     * @return 返回值含义：
     *         - 0: 使能成功
     *         - 1: 操作超时
     *         - 2: 发送消息失败
     *         - 3: 电机不存在
     *         - 负数: 电机故障码的负数
     */
    // 查找电机控制数据
    auto data_it = motor_ctrl_datas_.find(id);
    if (data_it == motor_ctrl_datas_.end()) {
        RWLOG_W("Cannot enable unknown motor ID: 0x%02X", id);
        return 3; // 电机不存在
    }

    // 设置状态为发送中
    data_it->second.startOperation(Operation::ENABLE);

    // 发送enable指令
    if (!data_it->second.motor.enterMotorState()) {
        RWLOG_E("Failed to send enable command to motor 0x%02X", id);
        data_it->second.updateOperationStatus(OperationStatus::FAILED);
        return 2; // 发送消息失败
    }

    // RWLOG_I("Enable command sent to motor 0x%02X, waiting for confirmation...", id);

    // 等待操作完成，waitForOperationStatus会检查故障码和超时
    int result = waitForOperationStatus(id, Operation::ENABLE, OperationStatus::SUCCESS, timeout_ms, "Enable");
    if (result != 0) {
        return result; // 返回具体的错误码 (1=超时, 负数=故障码)
    }

    return 0; // 成功
}

void RevoMotorControl::setZeroTorqueMode(bool enable) {
    zero_torque_mode_ = enable;
    RWLOG_I("Zero torque mode %s", enable ? "enabled" : "disabled");
}

void RevoMotorControl::setInitialTargetPositions(const std::map<MotorId, float>& target_positions) {
    std::lock_guard<std::mutex> lock(initial_target_positions_mutex_);
    initial_target_positions_ = target_positions;
    RWLOG_I("Set initial target positions for %zu motors", target_positions.size());
    for (const auto& [id, pos] : target_positions) {
        RWLOG_I("Motor %u initial target position: %.6f rad (%.2f deg)", id, pos, pos * 180.0f / M_PI);
    }
}

int RevoMotorControl::disableMotor(MotorId id, int timeout_ms) {
    /**
     * @brief 失能指定电机
     * @param id 电机ID
     * @param timeout_ms 超时时间(毫秒)
     * @return 返回值含义：
     *         - 0: 失能成功
     *         - 1: 操作超时
     *         - 2: 发送消息失败
     *         - 3: 电机不存在
     *         - 负数: 电机故障码的负数
     */
    // 查找电机控制数据
    auto data_it = motor_ctrl_datas_.find(id);
    if (data_it == motor_ctrl_datas_.end()) {
        RWLOG_W("Cannot disable unknown motor ID: 0x%02X", id);
        return 3; // 电机不存在
    }

    // 设置状态为发送中, 会在feedback和TEF回调中更新
    data_it->second.startOperation(Operation::DISABLE);

    // 发送 Enter Reset State 指令
    if (!data_it->second.motor.enterRestState()) {
        RWLOG_E("Failed to send disable command to motor 0x%02X", id);
        data_it->second.updateOperationStatus(OperationStatus::FAILED);
        return 2; // 发送消息失败
    }

    // RWLOG_I("Disable command sent to motor 0x%02X, waiting for confirmation...", id);

    // 等待操作完成，waitForOperationStatus会检查故障码和超时
    int result = waitForOperationStatus(id, Operation::DISABLE, OperationStatus::SUCCESS, timeout_ms, "Disable");
    if (result != 0) {
        return result; // 返回具体的错误码 (1=超时, 负数=故障码)
    }

    return 0; // 成功
}

bool RevoMotorControl::multiTurnZero(MotorId id, int timeout_ms) {
    // 查找电机控制数据
    auto data_it = motor_ctrl_datas_.find(id);
    if (data_it == motor_ctrl_datas_.end()) {
        RWLOG_W("Cannot do multi-turn zero for unknown motor ID: 0x%02X", id);
        return false;
    }

    // 设置状态为发送中
    data_it->second.startOperation(Operation::MULTI_TURN_ZERO);

    // 发送多圈编码器清零指令
    if (!data_it->second.motor.multiTurnZero()) {
        RWLOG_E("Failed to send multi-turn zero command to motor 0x%02X", id);
        data_it->second.updateOperationStatus(OperationStatus::FAILED);
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int result = waitForOperationStatus(id, Operation::MULTI_TURN_ZERO, OperationStatus::PENDING, timeout_ms, "Multi-turn zero");
    return (result == 0); // multiTurnZero 返回 bool，所以需要转换
}

bool RevoMotorControl::setMotorConfig(MotorId id, const RevoMotorConfig_t& config) {
    // 查找电机控制数据
    auto data_it = motor_ctrl_datas_.find(id);
    if (data_it == motor_ctrl_datas_.end()) {
        RWLOG_W("Cannot set config for unknown motor ID: 0x%02X", id);
        return false;
    }

    // 零点变化时同步补偿缓存命令位置，保持物理目标不变：
    // write(): raw = ±(cmd.pos + zero_offset)
    // getPositions(): logical = ±raw - zero_offset
    // 因此 zero_offset += delta 时，cmd.pos 必须 -= delta，否则下一拍 write 会跳变
    const float old_zero_offset = data_it->second.config.zero_offset;
    const float zero_delta = config.zero_offset - old_zero_offset;
    if (std::abs(zero_delta) > 1e-8f) {
        std::lock_guard<std::mutex> lock(batch_mutex_);
        RevoMotorCmd_t cmd = data_it->second.getCmd();
        cmd.pos -= static_cast<double>(zero_delta);
        data_it->second.setCmd(cmd);
        RWLOG_I("Motor 0x%02X zero_offset %.6f -> %.6f, compensate cmd.pos by %.6f",
                id, old_zero_offset, config.zero_offset, -zero_delta);
    }

    data_it->second.config = config;
    target_updated_ = true;

    return true;
}

void RevoMotorControl::internalMessageCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context) {
    /**
     * 高频函数最好不好使用 RWLOG_... 等日志函数
     * 
     */
    if(!frame) {
        return;
    }

    if (!context || !context->userdata) {
        if (frame) canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    // 从context获取类实例指针
    RevoMotorControl* controller = static_cast<RevoMotorControl*>(context->userdata);
    if (!controller) {
        canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    // 创建反馈帧并解析
    uint8_t payload[8];
    memcpy(payload, frame->payload, 8);
    FeedbackFrame feedback(payload);
    // !!! 重要！释放CAN帧
    canbus_sdk::freeCanMessageFrame(frame);

    // 查找对应的电机控制数据
    MotorId motor_id = feedback.motor_id();
    auto data_it = controller->motor_ctrl_datas_.find(motor_id);
    if (data_it == controller->motor_ctrl_datas_.end()) {
        printf("Received CAN message for unknown motor ID: 0x%02X\n", motor_id);
        return;
    }

    // 故障码检测
    auto errcode = feedback.errcode();
    if(errcode != MotorErrCode::NO_FAULT) {
        data_it->second.fault_code.store(errcode);  // 更新电机存在故障码
        std::string errcode_str = MotorErrCode2string(errcode);
        printf("\033[31m电机 0x%02X 故障码: 0x%03X, 描述: %s\033[0m\n", motor_id, static_cast<uint8_t>(errcode), errcode_str.c_str());
        // TODO: automatic recovery 自动恢复: 执行enter rest state 清除故障码
        
        // 有故障码也要应该要更新
        // return;
    }

    // 位置跳变检测
    auto prev_pos = data_it->second.motor.position();
    auto curr_pos = feedback.position();
    if(data_it->second.feedback_received.load() && hasPositionJump(prev_pos, curr_pos)) {
        printf("\033[31m电机 0x%02X 位置跳变: %.4f ---> %.4f\033[0m\n", motor_id, prev_pos, curr_pos);
        return;
    }

    // TODO: 温度过高检测

    // 更新电机状态、更新电机控制状态为收到反馈
    if(data_it->second.motor.receiveFeedback(feedback)) {
        data_it->second.feedback_received.store(true); // 更新电机状态为收到反馈
    }
    // printf("Received feedback from motor %d: pos=%.4f, vel=%.4f, torque=%.4f\n",
    //          motor_id, feedback_frame.position(), feedback_frame.velocity(), feedback_frame.torque());
}

void RevoMotorControl::internalTefEventCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context) {
    if(!frame) {
        return;
    }

    if (!context || !context->userdata) {
        if (frame) canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    // 从context获取类实例指针
    RevoMotorControl* controller = static_cast<RevoMotorControl*>(context->userdata);
    if (!controller) {
        canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    MotorId motor_id = static_cast<MotorId>(frame->id.SID); // CAN_ID ---> MotorId

    // 拷贝 payload
    uint8_t payload[8];
    memcpy(payload, frame->payload, 8);

    // 释放CAN帧
    canbus_sdk::freeCanMessageFrame(frame);

    // 查找电机控制数据
    auto data_it = controller->motor_ctrl_datas_.find(motor_id);
    if (data_it == controller->motor_ctrl_datas_.end()) {
        printf("TEF callback for unknown motor ID: 0x%02X\n", motor_id);
        return;
    }

    if (data_it->second.operation_status != OperationStatus::PENDING) {
        // 减少 memcmp 次数
        return;
    }
    
    // 根据payload判断是不是控制指令
    if (memcmp(payload, kEnterMotorStateFrame, 8) == 0) {
        // enable指令发送成功
        if (data_it->second.operation == Operation::ENABLE &&
            data_it->second.operation_status == OperationStatus::PENDING) {
                data_it->second.updateOperationStatus(OperationStatus::SUCCESS);
                printf("Motor 0x%02X enable command sent successfully\n", motor_id);
        }
    } else if (memcmp(payload, kEnterResetStateFrame, 8) == 0) {
        // disable指令发送成功
        if (data_it->second.operation == Operation::DISABLE &&
            data_it->second.operation_status == OperationStatus::PENDING) {
                data_it->second.updateOperationStatus(OperationStatus::SUCCESS);
                printf("Motor 0x%02X disable command sent successfully\n", motor_id);
        }
    } else if (memcmp(payload, kMultiTurnZeroFrame, 8) == 0) {
        // multi-turn zero指令发送成功
        if (data_it->second.operation == Operation::MULTI_TURN_ZERO &&
            data_it->second.operation_status == OperationStatus::PENDING) {
                data_it->second.updateOperationStatus(OperationStatus::SUCCESS);
                printf("Motor 0x%02X multi-turn zero command sent successfully\n", motor_id);
        }
    }
}

static std::vector<std::vector<float>> interpolate_positions_with_speed(
    const std::vector<float>& start_positions,
    const std::vector<float>& target_positions,
    float speed,
    float dt,
    float timeout = 5.0f) {

    if (start_positions.size() != target_positions.size()) {
        RWLOG_E("Start and target positions size mismatch");
        return {};
    }

    std::cout << "\033[32m插值起始位置: \033[0m\n";
    for (const auto& pos : start_positions) {
        std::cout << std::fixed << std::setprecision(4) << pos << " ";
    }
    std::cout << std::endl;
    std::cout << "\033[32m插值目标位置: \033[0m\n";
    for (const auto& pos : target_positions) {
        std::cout << std::fixed << std::setprecision(4) << pos << " ";
    }
    std::cout << std::endl;

    int num_motors = start_positions.size();

    // 计算每个电机的距离，找到最大距离的电机作为基准
    float max_distance = 0.0f;
    for (int i = 0; i < num_motors; ++i) {
        float distance = std::abs(target_positions[i] - start_positions[i]);
        if (distance > max_distance) {
            max_distance = distance;
        }
    }

    // 根据最大距离和速度计算总时间
    float total_time = std::min((max_distance / speed), timeout); // 归零运动不要超timeout秒

    // 根据总时间和时间步长计算插值步数
    int steps = static_cast<int>(total_time / dt);
    if (steps < 2) steps = 2;  // 至少需要2步：起始位置和目标位置

    RWLOG_D("Interpolation: max_distance=%.4f, speed:%.4f, time=%.4fs, steps=%d", max_distance, speed, total_time, steps);

    // 使用线性插值计算每个时间步的电机位置
    std::vector<std::vector<float>> interpolation(steps, std::vector<float>(num_motors));
    for (int step = 0; step < steps; ++step) {
        float t = static_cast<float>(step) / std::max(1, (steps - 1));
        for (int motor = 0; motor < num_motors; ++motor) {
            interpolation[step][motor] = start_positions[motor] + t * (target_positions[motor] - start_positions[motor]);
        }
    }

    return interpolation;
}

void RevoMotorControl::updateMotorCmd(MotorId id, double pos, double vel, double torque, double kp, double kd,
                                    double scalar_before10)
{
    if (motor_ctrl_datas_.find(id) == motor_ctrl_datas_.end()) {
        RWLOG_W("Motor %u not found in updateMotorCmd", id);
        return;
    }

    auto& data = motor_ctrl_datas_.at(id);

    // 整组锁保护所有电机的 cmd（替代 per-motor cmd_mutex）
    std::lock_guard<std::mutex> lock(batch_mutex_);
    data.setCmd(RevoMotorCmd_t{
        .pos = pos,
        .vel = vel,
        .torque = torque,
        .kp = kp,
        .kd = kd,
        .local_receive_seq = ++command_receive_seq_,
        .scalar_before10 = scalar_before10
    });

    // RWLOG_D("Updated motor %u command: pos=%.3f, vel=%.3f, torque=%.3f, kp=%.1f, kd=%.1f",
    //          id, pos, vel, torque, kp, kd);

    target_updated_ = true;
}

int RevoMotorControl::waitForOperationStatus(MotorId id, Operation expected_op, OperationStatus expected_status, int timeout_ms, const char* operation_name) {
    /**
     * @brief 等待电机操作完成
     * @param id 电机ID
     * @param expected_op 期望的操作类型
     * @param expected_status 期望的操作状态
     * @param timeout_ms 超时时间(毫秒)
     * @param operation_name 操作名称(用于日志)
     * @return 返回值含义：
     *         - 0: 操作成功完成
     *         - 1: 操作超时
     *         - 负数: 电机故障码的负数
     */
    auto start_time = std::chrono::steady_clock::now();
    while (true) {
        // 检查是否超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        if (elapsed.count() >= timeout_ms) {
            RWLOG_W("%s motor 0x%02X timeout after %d ms", operation_name, id, timeout_ms);
            auto data_it = motor_ctrl_datas_.find(id);
            if (data_it != motor_ctrl_datas_.end()) {
                data_it->second.updateOperationStatus(OperationStatus::TIMEOUT);
            }
            return 1; // 超时
        }

        // 检查状态是否已更新为成功
        auto data_it = motor_ctrl_datas_.find(id);
        if (data_it != motor_ctrl_datas_.end() &&
            data_it->second.operation == expected_op &&
            data_it->second.operation_status == expected_status) {
            // 多圈清零指令没有反馈帧
            if(expected_op == Operation::MULTI_TURN_ZERO) {
                RWLOG_I("Motor 0x%02X %s completed successfully", id, operation_name);
                return 0;
            }
            
            // 检查电机故障码
            auto errcode = data_it->second.getFaultCode();
            if(errcode != MotorErrCode::NO_FAULT) {  // 没有故障
                data_it->second.updateOperationStatus(OperationStatus::FAILED);
                RWLOG_E("Motor 0x%02X %s failed: %s", id, operation_name, MotorErrCode2string(errcode).c_str());
                return -static_cast<int>(errcode);
            }

            // 有收到反馈帧
            if(true == data_it->second.feedback_received.load()) {
                RWLOG_I("Motor 0x%02X %s completed successfully", id, operation_name);
                return 0;
            }
        } // end if data_it != motor_ctrl_datas_.end()

        // 10ms检查一次
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 1; // timeout
}

bool RevoMotorControl::waitForAllMotorsFeedback(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // 检查是否超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        if (elapsed.count() >= timeout_ms) {
            RWLOG_W("Timeout waiting for feedback from all motors after %d ms", timeout_ms);
            return false;
        }

        // 检查是否所有非忽略电机都收到了反馈
        bool all_feedback_received = true;
        for (const auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore && !data.feedback_received) {
                all_feedback_received = false;
            }
        }

        if (all_feedback_received) {
            RWLOG_SUCCESS("All %d motors have received feedback", motor_ctrl_datas_.size());
            return true;
        }

        // 10ms检查一次
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

RevoMotorControl::OperationStatus RevoMotorControl::moveToZero(float zero_timeout) {
    RWLOG_I(" === Start moving motors to zero position ===");

    // 定义控制参数
    constexpr float control_frequency = 100.0f;
    constexpr float dt = 1.0f / control_frequency;
    constexpr float movement_speed = 25.0f * M_PI / 180.0f; 

    // 获取所有电机的当前位置
    std::vector<float> current_positions;
    std::vector<MotorId> motor_ids;

    auto positions = getPositions();
    for (const auto& [id, pos] : positions) {
        current_positions.push_back(pos);
        motor_ids.push_back(id);
    }

    if (current_positions.empty()) {
        RWLOG_W("No motors available for zero movement");
        return OperationStatus::FAILED;
    }

    // 从initial_target_positions_获取目标位置，如果某个电机ID不在map中，则默认回零到0
    std::vector<float> target_positions(current_positions.size(), 0.0f);
    {
        std::lock_guard<std::mutex> lock(initial_target_positions_mutex_);

        // 打印initial_target_positions_ map的内容
        std::cout << "\033[32mInitial target positions map (size: " << initial_target_positions_.size() << "): \033[0m\n";
        if (initial_target_positions_.empty()) {
            std::cout << "  (empty - all motors will go to 0)\n";
        } else {
            for (const auto& [id, pos] : initial_target_positions_) {
                std::cout << "  Motor " << id << " -> " << std::fixed << std::setprecision(6) 
                          << pos << " rad (" << pos * 180.0f / M_PI << " deg)\n";
            }
        }
        std::cout << std::endl;

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            MotorId id = motor_ids[i];
            auto it = initial_target_positions_.find(id);
            if (it != initial_target_positions_.end()) {
                target_positions[i] = it->second;
                RWLOG_I("Motor %u target position set to %.6f rad (%.2f deg)", id, it->second, it->second * 180.0f / M_PI);
            } else {
                target_positions[i] = 0.0f;  // 默认回零到0
            }
        }
    }

    // 计算插值路径
    auto interpolation_path = interpolate_positions_with_speed(current_positions, target_positions, movement_speed, dt);

    RWLOG_I("Moving %zu motors to zero with %zu interpolation steps", motor_ids.size(), interpolation_path.size());
    // 第一阶段：发送插值结果
    RWLOG_I("Phase 1: 所有电机开始从当前位置插值到零点");
    for (size_t step = 0; step < interpolation_path.size(); ++step) {
        // 发送位置命令到所有电机
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            MotorId id = motor_ids[i];
            float target_pos = interpolation_path[step][i];
            auto& data = motor_ctrl_datas_.at(id);
            updateMotorCmd(id, target_pos, 0.0f, 0.0f, data.config.default_params.kp_pos, data.config.default_params.kd_pos);
        }
        // 按照控制频率等待
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
    
    // 第二阶段：检查是否到达零点，并记录未到达阈值的电机
    // 检查当前位置 - 使用 getPositions() 获取经过零点偏移和方向处理后的位置
    auto current_positions_for_check_map = getPositions();
    std::vector<float> current_positions_for_check;
    std::unordered_set<MotorId> motors_outside_threshold;  // 只记录未到达阈值的电机

    // 获取目标位置映射（用于检查）
    std::map<MotorId, float> target_positions_map;
    {
        std::lock_guard<std::mutex> lock(initial_target_positions_mutex_);
        target_positions_map = initial_target_positions_;
    }

    for (size_t i = 0; i < motor_ids.size(); ++i) {
        MotorId id = motor_ids[i];
        current_positions_for_check.push_back(current_positions_for_check_map.at(id));

        // 检查是否在阈值外，只记录未到达的电机
        float current_pos = current_positions_for_check_map.at(id);
        float target_pos = 0.0f; // 默认目标是零点
        auto it = target_positions_map.find(id);
        if (it != target_positions_map.end()) {
            target_pos = it->second;
        }

        // 检查当前位置是否接近目标位置
        if (std::abs(current_pos - target_pos) > kZeroThresholdRadians) {
            motors_outside_threshold.insert(id);
        }
    }

    if (motors_outside_threshold.empty()) {
        RWLOG_SUCCESS("所有电机已到达目标位置");
        return OperationStatus::SUCCESS;
    }

    // 第三阶段：如果没有到达目标，继续发送命令，但只给未到达阈值的电机施加扭矩
    // 打印当前位置
    std::cout << "\033[33m电机未到达目标位置(阈值相差大于0.5度),当前位置:\033[0m\n";
    for (size_t i = 0; i < current_positions_for_check.size(); ++i) {
        MotorId motor_id = motor_ids[i];
        float target_pos = 0.0f;
        auto it = target_positions_map.find(motor_id);
        if (it != target_positions_map.end()) {
            target_pos = it->second;
        }

        std::cout << "电机" << motor_id << "(";
        if (std::abs(current_positions_for_check[i] - target_pos) > kZeroThresholdRadians) {
            std::cout << "\033[31m目标:" << std::fixed << std::setprecision(1) << target_pos * 180.0f / M_PI << "度\033[0m ";
        } else {
            std::cout << "目标:" << std::fixed << std::setprecision(1) << target_pos * 180.0f / M_PI << "度 ";
        }
        std::cout << "当前:" << std::fixed << std::setprecision(4) << current_positions_for_check[i] << ") ";
    }
    std::cout << std::endl;

    // Lambda函数检查是否所有位置都为零
    auto all_positions_zero = [](const std::vector<float>& positions, float threshold) -> bool {
        for (const auto& pos : positions) {
            if (std::abs(pos) > threshold) {
                return false;
            }
        }
        return true;
    };

    auto start_time = std::chrono::steady_clock::now();
    bool reached_zero = false;

    while (!reached_zero) {
        // 检查是否超时
        auto elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > zero_timeout) {
            RWLOG_W("Move to zero timeout after %.1f seconds", zero_timeout);
            const auto latest_positions = getPositions();
            for (const auto id : motor_ids) {
                const auto target = target_positions_map.find(id);
                const float target_pos = target == target_positions_map.end() ? 0.0f : target->second;
                const float position = latest_positions.at(id);
                if (!std::isfinite(position) || !std::isfinite(target_pos)) {
                    RWLOG_FAILURE("Invalid position during boot positioning: motor ID %u", id);
                    return OperationStatus::FAILED;
                }
                if (std::abs(position - target_pos) > kZeroThresholdRadians) {
                    RWLOG_WARNING("Boot positioning unsettled: motor ID %u, position %.6f rad, "
                                  "target %.6f rad, error %.3f deg", id, position, target_pos,
                                  (position - target_pos) * 180.0f / M_PI);
                }
            }
            return OperationStatus::TIMEOUT;
        }

        // 给未到达阈值的电机施加0.2扭矩
        for (const auto& id : motors_outside_threshold) {
            auto& data = motor_ctrl_datas_.at(id);

            float cmd_pos = 0.0f; // 默认命令位置
            auto it = target_positions_map.find(id);
            if (it != target_positions_map.end()) {
                cmd_pos = it->second;
            }

            updateMotorCmd(id, cmd_pos, 0.0f, 0.2f, data.config.default_params.kp_pos, data.config.default_params.kd_pos);
        }

        // 按照控制频率等待
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));

        // 检查是否所有电机都到达目标位置 - 使用 getPositions() 获取经过零点偏移和方向处理后的位置
        auto current_positions_for_timeout_check_map = getPositions();
        std::vector<float> current_positions_for_timeout_check;
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            MotorId id = motor_ids[i];
            current_positions_for_timeout_check.push_back(current_positions_for_timeout_check_map.at(id));
        }
        // 检查是否到达目标位置
        reached_zero = true;
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            MotorId id = motor_ids[i];
            float current_pos = current_positions_for_timeout_check[i];
            float target_pos = 0.0f;
            auto it = target_positions_map.find(id);
            if (it != target_positions_map.end()) {
                target_pos = it->second;
            }

            if (std::abs(current_pos - target_pos) > kZeroThresholdRadians) {
                reached_zero = false;
                break;
            }
        }
    }
    return OperationStatus::SUCCESS;
}

bool RevoMotorControl::calibrateMotors() {
    RWLOG_I("Waiting for all motors to receive feedback frames...");

    // 等待所有电机接收到feedback帧，超时时间1000ms
    auto start_time = std::chrono::steady_clock::now();
    const int timeout_ms = 1000;

    while (true) {
        // 检查是否所有电机都收到了反馈
        bool all_received = true;
        for (const auto& [id, data] : motor_ctrl_datas_) {
            if (!data.feedback_received && !data.config.ignore) {
                all_received = false;
                break;
            }
        }

        if (all_received) {
            RWLOG_I("All motors received feedback, breaking wait loop");
            break;
        }

        // 检查是否超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        if (elapsed.count() >= timeout_ms) {
            RWLOG_W("Calibration timeout after %d ms", timeout_ms);
            break;
        }

        // 10ms检查一次
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 检查是否所有电机都收到了反馈，因为要需要把反馈的位置记做零点
    for (auto& [id, data] : motor_ctrl_datas_) {
        if (!data.feedback_received && !data.config.ignore) {
            RWLOG_E(" --- 警告！警告！电机 0x%02X 没有收到反馈帧，无法把电机当前位置设置成零点，使用默认值(0.0)", id);
            //  WARN: 电机被标记成 ignore/电机enble失败的情况，只能把零点设置成 MAGIC_NUMBER 了
        }
        else if(data.config.ignore){
            // 忽略电机 => 使用magic_number标记，下次使能时使用当前位置作为offset
            data.config.zero_offset = kOFFSET_MAGIC_NUMBER;
        }
        else {
            // 电机正常使能，将当前反馈的位置设置为零点偏移
            float current_raw_position = data.motor.position();
            data.config.zero_offset = data.config.negtive? -current_raw_position : current_raw_position;

            // 检查 zero_offset 是否在合理范围内: kCAN_COM_THETA_MIN + π ~ kCAN_COM_THETA_MAX - π
            constexpr float min_safe_offset = kCAN_COM_THETA_MIN + M_PI;  // -12.5 + π ≈ -9.35841
            constexpr float max_safe_offset = kCAN_COM_THETA_MAX - M_PI;  // 12.5 - π ≈ 9.35841
            if (data.config.zero_offset < min_safe_offset || data.config.zero_offset > max_safe_offset) {
                RWLOG_FAILURE("电机 0x%02X 零点偏移 %.4f 超出安全范围 [%.4f, %.4f]，可能影响运动范围",
                           id, data.config.zero_offset, min_safe_offset, max_safe_offset);
                return false;
            }
        }
        
        RWLOG_I(" --- Motor 0x%02X received feedback successfully, set as zero offset: %.4f rad", id, data.config.zero_offset);
    }

    RWLOG_I("Calibration completed successfully");
    return true;
}

std::map<MotorId, float> RevoMotorControl::getZeroOffsets() {
    std::map<MotorId, float> zero_offsets;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        zero_offsets[id] = data.config.zero_offset;
    }
    return zero_offsets;
}

std::map<MotorId, float> RevoMotorControl::getRawPositions() {
    std::map<MotorId, float> raw_positions;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        raw_positions[id] = data.motor.position();
    }
    return raw_positions;
}

std::map<MotorId, float> RevoMotorControl::getRawVelocities() {
    std::map<MotorId, float> raw_velocities;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        raw_velocities[id] = data.motor.velocity();
    }
    return raw_velocities;
}

std::map<MotorId, float> RevoMotorControl::getRawTorques() {
    std::map<MotorId, float> raw_torques;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        raw_torques[id] = data.motor.torque();
    }
    return raw_torques;
}

std::map<MotorId, float> RevoMotorControl::getPositions() {
    std::map<MotorId, float> positions;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        float raw_pos = data.motor.position();
        const auto& config = data.config;

        // 应用零点偏移和方向处理
        float adjusted_pos = raw_pos;
        if (config.negtive) {
            adjusted_pos = -adjusted_pos;
        }
        positions[id] = adjusted_pos - config.zero_offset;
        // 电机被标记为忽略或者没有使能成功 返回0.0f
        if(config.ignore || !data.isEnabled()) {
            positions[id] = 0.0f;
        }
    }
    return positions;
}


RevoMotorConfig_t RevoMotorControl::getMotorConfig(MotorId id) const {
    auto it = motor_ctrl_datas_.find(id);
    if (it != motor_ctrl_datas_.end()) {
        return it->second.config;
    }

    // 返回默认配置
    RevoMotorConfig_t default_config;
    default_config.id = id;
    default_config.name = "Unknown";
    default_config.negtive = false;
    default_config.ignore = true;
    default_config.zero_offset = 0.0f;
    default_config.ratio = 1.0f;
    return default_config;
}

std::map<MotorId, float> RevoMotorControl::getVelocities() {
    std::map<MotorId, float> velocities;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        float raw_vel = data.motor.velocity();
        const auto& config = data.config;

        // 应用方向处理
        float adjusted_vel = raw_vel;
        if (config.negtive) {
            adjusted_vel = -adjusted_vel;
        }
        velocities[id] = adjusted_vel;
    }
    return velocities;
}

std::map<MotorId, float> RevoMotorControl::getTorques() {
    std::map<MotorId, float> torques;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        float raw_torque = data.motor.torque();
        const auto& config = data.config;

        // 应用方向处理
        float adjusted_torque = raw_torque;
        if (config.negtive) {
            adjusted_torque = -adjusted_torque;
        }
        torques[id] = adjusted_torque;
    }
    return torques;
}

std::map<MotorId, MotorState> RevoMotorControl::getMotorStates() {
    std::map<MotorId, MotorState> motor_states;

    for (const auto& [id, data] : motor_ctrl_datas_) {
        motor_states[id] = data.isEnabled() ? MotorState::MotorSate : MotorState::RestSate;
    }

    return motor_states;
}

void RevoMotorControl::setTargetPositions(const std::map<MotorId, RevoMotorCmd_t>& targets)
{
    for (const auto& [id, cmd] : targets) {
        if (motor_ctrl_datas_.find(id) == motor_ctrl_datas_.end()) {
            RWLOG_W("Motor %u not found in setTargetPositions, skipping", id);
            continue;
        }

        // 调用updateMotorCmd函数，避免代码重复
        updateMotorCmd(id, cmd.pos, cmd.vel, cmd.torque, cmd.kp, cmd.kd, cmd.scalar_before10);

        // RWLOG_D("Set target for motor %u: pos=%.3f, vel=%.3f, torque=%.3f, kp=%.1f, kd=%.1f",
        //          id, cmd.pos, cmd.vel, cmd.torque, cmd.kp, cmd.kd);
    }
}

void RevoMotorControl::write()
{
    // Consume only the notification that admitted this batch. An update
    // during getCmd()/CAN send must remain pending for the next write().
    if(!target_updated_.exchange(false)) {
        return;
    }

    // 遍历所有电机，发送控制命令
    for (auto& [id, data] : motor_ctrl_datas_) {
        if(data.config.ignore || !data.isEnabled()) {
            continue;
        }

        // 整组锁保护 cmd 读取（替代 per-motor cmd_mutex）
        RevoMotorCmd_t cmd;
        {
            std::lock_guard<std::mutex> lock(batch_mutex_);
            cmd = data.getCmd();
        }

        // 根据电机方向和零点偏移计算原始目标值
        double raw_target_position = cmd.pos;
        double raw_target_velocity = cmd.vel;
        double raw_target_torque = cmd.torque;
        float kp = static_cast<float>(cmd.kp);
        float kd = static_cast<float>(cmd.kd);

        if (zero_torque_mode_) {
            // 0-torque模式：所有控制参数设为0
            raw_target_position = 0.0;
            raw_target_velocity = 0.0;
            raw_target_torque = 0.0;
            kp = 0.0f;
            kd = 0.0f;
        } else {
            // 正常模式：根据电机方向和零点偏移计算原始目标值
            if (data.config.negtive) {
                raw_target_position = -(raw_target_position + data.config.zero_offset);
                raw_target_velocity = -raw_target_velocity;
                raw_target_torque = -raw_target_torque;
            }
            else {
                raw_target_position += data.config.zero_offset;
            }
        }

        // 发送控制命令
        kuavo_e2e::Trace<32>::Row observation{};
        auto* observation_ptr=command_trace_.enabled()?&observation:nullptr;
        if(observation_ptr) {
            observation[0]=1;observation[1]=cmd.local_receive_seq>>32;
            observation[2]=cmd.local_receive_seq&0xffffffffULL;
            observation[3]=kuavo_e2e::ns();observation[21]=cmd.scalar_before10;
            observation[22]=cmd.torque;observation[23]=cmd.pos;observation[24]=cmd.vel;
            observation[25]=cmd.kp;observation[26]=cmd.kd;observation[27]=id;
            observation[28]=command_trace_.dropped();
        }
        const bool sent=data.motor.controlPTM(
            static_cast<float>(raw_target_position),
            static_cast<float>(raw_target_velocity),
            static_cast<float>(raw_target_torque),
            kp,
            kd, observation_ptr
        );
        if(observation_ptr) {
            observation[4]=kuavo_e2e::ns();observation[5]=sent;
            command_trace_.push(observation);
        }

        // RWLOG_D("Motor %u: pos=%.3f->%.3f, vel=%.3f->%.3f, torque=%.3f->%.3f, neg=%d, zero_offset=%.3f",
        //          id, data.cmd.pos, raw_target_position, data.cmd.vel, raw_target_velocity,
        //          data.cmd.torque, raw_target_torque, data.config.negtive, data.config.zero_offset);
    }

    // Do not clear here: that would erase a newer producer notification.
}

void RevoMotorControl::setRtSend(bool enable) {
    for (auto& [id, data] : motor_ctrl_datas_) {
        data.motor.setRtSend(enable);
    }
    RWLOG_I("Set RT send mode=%d for %zu motors", enable ? 1 : 0, motor_ctrl_datas_.size());
}

bool RevoMotorControl::registerRtSource() {
    using namespace canbus_sdk;
    auto& controller = CanBusController::getInstance();

    // 获取本控制器所属的总线 ID
    auto bus_id_result = controller.getBusIdByName(canbus_name_);
    if (!bus_id_result.has_value()) {
        RWLOG_FAILURE("registerRtSource failed: bus '%s' not found", canbus_name_.c_str());
        return false;
    }
    BusId bus_id = bus_id_result.value();
    bus_id_ = bus_id;   // 保存，供析构时按总线注销

    // 先确保所有电机已开启 RT 直发模式（controlPTM 内部走 sendMessageRt）
    setRtSend(true);

    // 将电机编码注册为 RT 源：sender 线程每轮先调用 rtWrite()，再处理 ring 队列
    // 回调捕获 this 指针；注意 RevoMotorControl 析构时必须调用 unregisterRtSource() 注销，
    // 否则 sender 线程继续调用已销毁对象的 rtWrite() 会 use-after-free
    bool ok = controller.registerRtSource(bus_id, [this]() {
        this->rtWrite();
    });
    if (ok) {
        RWLOG_SUCCESS("Registered RT source for bus '%s' (bus_id=%d), %zu motors",
                      canbus_name_.c_str(), static_cast<int>(bus_id), motor_ctrl_datas_.size());
    } else {
        RWLOG_FAILURE("registerRtSource failed for bus '%s'", canbus_name_.c_str());
    }
    return ok;
}

void RevoMotorControl::unregisterRtSource() {
    using namespace canbus_sdk;
    if (bus_id_ != static_cast<BusId>(-1)) {
        CanBusController::getInstance().unregisterRtSource(bus_id_);
        bus_id_ = static_cast<BusId>(-1);
    }
    setRtSend(false);
    RWLOG_I("Unregistered RT source for bus '%s'", canbus_name_.c_str());
}

void RevoMotorControl::rtWrite() {
    // RT 源回调：在 sender 线程内执行，取最新 cmd → 编码所有使能电机 PTM 帧 → 直发
    // 与 write() 逻辑相同；controlPTM 内部根据 rt_send_ 走 sendMessageRt（直发）
    // Limit each bus to 250 Hz without dropping newer producer updates.
    if (!rt_send_limiter_.consume(target_updated_)) {
        return;
    }

    for (auto& [id, data] : motor_ctrl_datas_) {
        if(data.config.ignore || !data.isEnabled()) {
            continue;
        }

        // 整组锁保护 cmd 读取（替代 per-motor cmd_mutex）
        RevoMotorCmd_t cmd;
        {
            std::lock_guard<std::mutex> lock(batch_mutex_);
            cmd = data.getCmd();
        }

        double raw_target_position = cmd.pos;
        double raw_target_velocity = cmd.vel;
        double raw_target_torque = cmd.torque;
        float kp = static_cast<float>(cmd.kp);
        float kd = static_cast<float>(cmd.kd);

        if (zero_torque_mode_) {
            raw_target_position = 0.0;
            raw_target_velocity = 0.0;
            raw_target_torque = 0.0;
            kp = 0.0f;
            kd = 0.0f;
        } else {
            if (data.config.negtive) {
                raw_target_position = -(raw_target_position + data.config.zero_offset);
                raw_target_velocity = -raw_target_velocity;
                raw_target_torque = -raw_target_torque;
            }
            else {
                raw_target_position += data.config.zero_offset;
            }
        }

        data.motor.controlPTM(
            static_cast<float>(raw_target_position),
            static_cast<float>(raw_target_velocity),
            static_cast<float>(raw_target_torque),
            kp,
            kd
        );
    }

    // Do not clear pending here: TX may overlap a newer command update.
}

RevoMotorControl::~RevoMotorControl() {
    RWLOG_I("RevoMotorControl destructor called, cleaning up resources...");

    // 必须先注销 RT 源：sender 线程每轮会调用 RT 源回调（rtWrite），若 this 已销毁则 use-after-free
    unregisterRtSource();

    // 先失能所有电机
    disableAll();

    // 获取CAN总线控制器实例
    auto& canbus_controller = canbus_sdk::CanBusController::getInstance();

    // 尝试获取bus_id
    auto bus_id_result = canbus_controller.getBusIdByName(canbus_name_);

    if (bus_id_result.has_value()) {
        // 反注册所有电机设备
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore) {
                RWLOG_D("Unregistering motor %u device", id);
                auto devid = static_cast<canbus_sdk::DeviceId>(id);
                canbus_controller.unregisterDevice(canbus_sdk::DeviceType::MOTOR, devid, canbus_name_);
            }
        }

        RWLOG_SUCCESS("RevoMotorControl resources cleaned up successfully");
    } else {
        RWLOG_WARNING("Could not find bus_id for %s during cleanup", canbus_name_.c_str());
    }
}

bool RevoMotorControl::enableAll()
{
    std::unordered_map<MotorId, int> results;
    for (auto& [id, data] : motor_ctrl_datas_) {
        // 电机在配置文件中被标记为 <ignore> 则跳过
        if(data.config.ignore) {
            results.insert(std::make_pair(id, 31415926)); // 31415926 magic number
            continue;
        }
        int ret = enableMotor(id, 500); // 500ms
        results.insert(std::make_pair(id, ret));
    }

    // 检查状态: enable 使能指令是否发送成功
    bool danger_error = false;   // 有严重故障码
    int ignore_count = 0, success_count = 0;
    for (auto& [id, result] : results) {
        if (result == 0) {
            auto& data = motor_ctrl_datas_.at(id);
            if (!data.isEnabled()) {
                RWLOG_FAILURE("MOTOR ID:%u Enable: [Failed]", id);
            } else {
                success_count++;
                RWLOG_SUCCESS("MOTOR ID:%u Enable: [Success]", id);
            }
        }
        else if(result == 31415926) {
            RWLOG_WARNING("MOTOR ID:%u Enable: [Ignored]", id);
            ignore_count++;
        }
        else if(result == 1) {
            RWLOG_FAILURE("MOTOR ID:%u Enable: [Timeout]", id);
        }
        else if(result == 2) {
            RWLOG_FAILURE("MOTOR ID:%u Enable: [Send message failed]", id);
        }
        else if(result == 3) {
            RWLOG_FAILURE("MOTOR ID:%u Enable: [Motor not found]", id);
        }
        else if(result < 0) {
            uint8_t fault_code = static_cast<uint8_t>(-result);
            RWLOG_FAILURE("MOTOR ID:%u Enable: [Fault code 0x%02X]", id, fault_code);
            auto  errcode = static_cast<MotorErrCode>(fault_code);
            if(errcode >= MotorErrCode::ABS_ENCODER_OFFSET_VERIFICATION_FAILURE) {
                danger_error = true;
                std::string err_str = MotorErrCode2string(errcode);
                RWLOG_FAILURE("电机 ID:%u 使能失败: [无法清除的故障码 0x%03X: %s]", id, fault_code, err_str.c_str());
                RWLOG_FAILURE("电机 ID:%u 使能失败: [无法清除的故障码 0x%03X: %s]", id, fault_code, err_str.c_str());
                RWLOG_FAILURE("电机 ID:%u 使能失败: [无法清除的故障码 0x%03X: %s]", id, fault_code, err_str.c_str());
            }
        }
        else {
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Unknown error %d]", id, result);
        }
    }

    // 有严重故障码存在
    if(danger_error) {
        return false;
    }

    if(ignore_count == motor_ctrl_datas_.size()) {
        // 全部忽略那就是成功
        return true;
    }
    if(success_count == 0) {
        // 一个enable成功的都没有
        return false;
    }
    return true;
}

bool RevoMotorControl::disableAll()
{
    bool danger_error = false;
    std::map<MotorId, int> results;
    for (auto& [id, data] : motor_ctrl_datas_) {
        if(data.config.ignore) {
            results.insert(std::make_pair(id, 31415926)); // 31415926 magic number
            continue;
        }
        int ret = disableMotor(id);
        results.insert(std::make_pair(id, ret));
    }
    int ignore_count = 0, success_count = 0;
    for (auto& [id, result] : results) {
        if (result == 0) {
            success_count++;
            RWLOG_SUCCESS("MOTOR ID:%u Disable: [Success]", id);
        }
        else if(result == 31415926) {
            ignore_count++;
            RWLOG_SUCCESS("MOTOR ID:%u Disable: [Ignored]", id);
        }
        else if(result == 1) {
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Timeout]", id);
        }
        else if(result == 2) {
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Send message failed]", id);
        }
        else if(result == 3) {
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Motor not found]", id);
        }
        else if(result < 0) {
            uint8_t fault_code = static_cast<uint8_t>(-result);
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Fault code 0x%02X]", id, fault_code);
            auto  errcode = static_cast<MotorErrCode>(fault_code);
            if(errcode >= MotorErrCode::ABS_ENCODER_OFFSET_VERIFICATION_FAILURE) {
                danger_error = true;
                std::string err_str = MotorErrCode2string(errcode);
                RWLOG_FAILURE("电机 ID:%u 失能失败: [无法清除的故障码 0x%03X: %s]", id, fault_code, err_str.c_str());
                RWLOG_FAILURE("电机 ID:%u 失能失败: [无法清除的故障码 0x%03X: %s]", id, fault_code, err_str.c_str());
                RWLOG_FAILURE("电机 ID:%u 失能失败: [无法清除的故障码 0x%03X: %s]", id, fault_code, err_str.c_str());
            }
        }
        else {
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Unknown error %d]", id, result);
        }
    }

    // 有严重故障码存在
    if(danger_error) {
        return false;
    }

    if(ignore_count == motor_ctrl_datas_.size()) {
        // 全部忽略那就是成功
        return true;
    }
    if(success_count == 0) {
        // 一个成功的都没有
        return false;
    }
    return true;
}

void RevoMotorControl::multiTurnZeroAll()
{
    std::map<MotorId, int> results;
    for (auto& [id, data] : motor_ctrl_datas_) {
        // 电机在配置文件中被标记为 <ignore> 则跳过
        if(data.config.ignore) {
            results.insert(std::make_pair(id, -1));
            continue;
        }
        bool res = multiTurnZero(id, 200); // 200ms
        results.insert(std::make_pair(id, res? 1 : 0));
    }
    for (auto& [id, res] : results) {
        if(res == 1) {
            RWLOG_SUCCESS("MOTOR ID:%u MultiTurnZero: [Success]", id);
        }
        else if(res == -1) {
            RWLOG_SUCCESS("MOTOR ID:%u MultiTurnZero: [Ignored]", id);
        }
        else {
            RWLOG_FAILURE("MOTOR ID:%u MultiTurnZero: [Failed]", id);
        }
    }
}

} // namespace motorevo
