#ifndef _REVOMOTOR_DEF_H_
#define _REVOMOTOR_DEF_H_

#include <stdint.h>
#include <string.h>
#include <cstring>
#include <string>

namespace motorevo {

// 参数映射范围
constexpr float kCAN_COM_THETA_MIN = -12.5f;
constexpr float kCAN_COM_THETA_MAX = 12.5f;
constexpr float kCAN_COM_VELOCITY_MIN = -10.0f;
constexpr float kCAN_COM_VELOCITY_MAX = 10.0f;
constexpr float kCAN_COM_POS_KP_MIN = 0.0f;
constexpr float kCAN_COM_POS_KP_MAX = 250.0f;
constexpr float kCAN_COM_POS_KD_MIN = 0.0f;
constexpr float kCAN_COM_POS_KD_MAX = 50.0f;
constexpr float kCAN_COM_TORQUE_MIN = -50.0f;
constexpr float kCAN_COM_TORQUE_MAX = 50.0f;

// magic number零点偏移
constexpr float kOFFSET_MAGIC_NUMBER = 3.14159265359e-3f;  // π/1000 ≈ 0.00314159265f

typedef uint32_t MotorId;

enum class MotorState {
   None,
   RestSate,                      // 休眠模式
   MotorSate                      // 运行模式
};

enum class MotorMode {
   None,
   ServoMode,                      // 伺服模式
   TorquePositionMixControlMode,   // 力位混合模式
   VelocityMode,                   // 速度模式
   TorqueMode,                     // 力矩模式
   Torque4Mode                     // 力矩 4 模式
};

/// @brief Revo电机控制参数
struct MotorParam_t {
   float vel;       // 速度参数
   float kp_pos;    // 位置比例增益
   float kd_pos;    // 位置微分增益
   float tor;       // 力矩参数
   float kp_vel;    // 速度比例增益
   float kd_vel;    // 速度微分增益
   float ki_vel;    // 速度积分增益
};

/// @brief Revo电机配置参数
struct RevoMotorConfig_t {
   MotorId id;           // 电机ID
   std::string name;     // 电机名称
   bool negtive;         // 电机方向是否为负
   bool ignore;          // 是否忽略该电机
   float zero_offset;    // 零点偏移值(rad)
   float ratio;          // 减速比
   MotorParam_t default_params;  // 电机默认控制参数(外部没有传时使用)
};

/// @brief Revo电机控制命令
struct RevoMotorCmd_t {
   double pos, vel, torque; // 位置(rad), 速度(rad/s), 扭矩(Nm)
   double kp, kd;
};

/// @brief Revo电机故障码
/// 一旦驱动板检测到故障，将会从 Motor State 自动切回 Rest State 以保护驱动器和电机
/// 在排除异常情况后，发送 Enter Rest State 命令清除故障码，
/// 再发送 Enter Motor State 命令让电机重新恢复运行
/// See Details:《Motorevo Driver User Guide v0.2.3》- 6.5.1 反馈帧格式 - 故障码
enum class MotorErrCode:uint8_t {
   NO_FAULT = 0x00,                       // 无故障
   DC_BUS_OVER_VOLTAGE = 0x01,            // 直流母线电压过压
   DC_BUS_UNDER_VOLTAGE = 0x02,           // 直流母线电压欠压
   ENCODER_ANGLE_FAULT = 0x03,            // 编码器电角度故障
   DRV_DRIVER_FAULT = 0x04,               // DRV 驱动器故障
   DC_BUS_CURRENT_OVERLOAD = 0x05,        // 直流母线电流过流
   MOTOR_A_PHASE_CURRENT_OVERLOAD = 0x06, // 电机 A 相电流过载
   MOTOR_B_PHASE_CURRENT_OVERLOAD = 0x07, // 电机 B 相电流过载
   MOTOR_C_PHASE_CURRENT_OVERLOAD = 0x08, // 电机 C 相电流过载
   DRIVER_BOARD_OVERHEAT = 0x09,          // 驱动板温度过高
   MOTOR_WINDING_OVERHEAT = 0x0A,         // 电机线圈过温
   ENCODER_FAILURE = 0x0B,                // 编码器故障
   CURRENT_SENSOR_FAILURE = 0x0C,         // 电流传感器故障
   OUTPUT_ANGLE_OUT_OF_RANGE = 0x0D,      // 输出轴实际角度超过通信范围：CAN COM Theta MIN ~ CAN COM Theta MAX
   OUTPUT_SPEED_OUT_OF_RANGE = 0x0E,      // 输出轴速度超过通信范围 CAN COM Velocity MIN ~ CAN COM Velocity MAX
   STUCK_PROTECTION = 0x0F,               // 堵转保护：电机电枢电流(Iq)大于 Stuck Current，同时电机速度小于 Stuck Velocity，持续时间超过 Stuck Time 后触发
   CAN_COMMUNICATION_LOSS = 0x10,         // CAN 通讯丢失：超过 CAN COM TIMEOUT 时间没有收到 CAN 数据帧时触发
   // WARNING: 大于 128 的故障码为开机自检检测出的故障码，无法用该方法清除
   //
   ABS_ENCODER_OFFSET_VERIFICATION_FAILURE = 0x81, // 离轴/对心多圈绝对值编码器接口帧头校验失败
   ABSOLUTE_ENCODER_MULTI_TURN_FAILURE = 0x82,     // 对心多圈绝对值编编码器多圈接口故障
   ABSOLUTE_ENCODER_EXTERNAL_INPUT_FAILURE = 0x83, // 对心多圈绝对值编码器外部输入故障
   ABSOLUTE_ENCODER_SYSTEM_ANOMALY = 0x84,         // 对心多圈绝对值编码器读值故障
   ERR_OFFS = 0x85,                                // 对心多圈绝对值编码器ERR_OFFS
   ERR_CFG = 0x86,                                 // 对心多圈绝对值编码器ERR_CFG
   ILLEGAL_FIRMWARE_DETECTED = 0x88,               // 检测到非法固件
   INTEGRATED_STATOR_DRIVER_DAMAGED = 0x89,        // 集成式栅极驱动器初始化失败
};

class FeedbackFrame
{
public:
   FeedbackFrame();
   explicit FeedbackFrame(const uint8_t payload[8]);
   FeedbackFrame(const FeedbackFrame& other);
   FeedbackFrame& operator=(const FeedbackFrame& other);

   virtual ~FeedbackFrame()=default;
   
   /// @brief 清空反馈帧
   void     reset();
   /// @brief 重新设置反馈帧
   void     set_payload(const uint8_t payload[8]);
   /// @brief 位置(rad)
   float    position() const;
   /// @brief 速度(rad/s)
   float    velocity() const;
   /// @brief 扭矩(N·m)
   float    torque() const;
   /// @brief 温度(℃)
   uint8_t  temperature() const;
   /// @brief 故障码
   MotorErrCode  errcode() const;
   /// @brief 电机ID
   MotorId  motor_id() const;

private:
   uint8_t    payload_[8];
};

// 数值转换函数
/* 将浮点数转换为指定位数的整数
 * @param bit_num 转换位数（8/12/16），默认为8位
 * @param float_num 输入浮点数
 * @param min_num 数值范围最小值
 * @param max_num 数值范围最大值
 * @return 转换后的整数值（0 ~ 2^bit_num - 1）
 * @note 使用std::clamp进行范围限制，超出范围的值会被截断到边界值
 */
template<uint8_t bit_num = 8>
uint32_t float_to_int(float float_num, float min_num, float max_num);

/* 将指定位数的整数转换为浮点数
 * @param bit_num 转换位数（8/12/16），默认为8位
 * @param int_num 输入整数值
 * @param min_num 数值范围最小值
 * @param max_num 数值范围最大值
 * @return 转换后的浮点数值
 * @note 使用std::clamp进行范围限制，超出范围的整数会被截断到有效范围内
 */
template<uint8_t bit_num = 8>
float int_to_float(uint32_t int_num, float min_num, float max_num);

/* 将电机模式枚举转换为字符串描述
 * @param mode 电机模式枚举值
 * @return 模式描述字符串（如"ServoMode"、"VelocityMode"等）
 */
const char* to_string(MotorMode mode);

/* 将字符串转换为电机模式枚举
 * @param mode_str 模式字符串（支持"ptm"、"servo"、"torque4"、"torque"、"velocity"）
 * @return 对应的电机模式枚举值
 * @note 支持大小写不敏感的字符串匹配
 *       "ptm" -> TorquePositionMixControlMode
 *       "servo" -> ServoMode
 *       "torque4" -> Torque4Mode
 *       "torque" -> TorqueMode
 *       "velocity" -> VelocityMode
 */
MotorMode   to_motor_mode(const std::string& mode_str);


/**
 * @brief 将RuiwoErrCode枚举值转换为可读的字符串描述
 *
 * @param[in] errcode RuiwoErrCode枚举值，表示具体的电机故障码
 * @return std::string 故障码的中文描述
 */
std::string MotorErrCode2string(MotorErrCode errcode);

} // namespace motorevo

#endif