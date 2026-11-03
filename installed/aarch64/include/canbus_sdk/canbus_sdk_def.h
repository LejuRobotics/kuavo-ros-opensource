#ifndef CANBUS_SDK_DEF_H
#define CANBUS_SDK_DEF_H

#include <cstdint>
#include <cstddef>
#include <string>
#include <functional>

namespace canbus_sdk {

using ErrorType = int32_t;
using BusId = uint8_t;
using DeviceId = int32_t;

/**
 * @brief 设备类型枚举
 */
enum class DeviceType : uint8_t {
    MOTOR = 1,           // 电机
    REVO1_HAND = 2,      // Revo1
    REVO2_HAND = 3,      // Revo2
    LEJUCLAW = 4,        // 夹爪
    UNKNOWN = 0,         // 未知类型
};

/**
 * @brief CAN总线SDK操作的标准化错误码
 */
enum class CanBusError {
    SUCCESS = 0,                    // 操作成功完成
    ERROR_UNINITIALIZED = -1,       // 控制器未初始化
    ERROR_INVALID_PARAMETER = -2,   // 无效的输入参数
    ERROR_NO_AVAILABLE_SLOT = -3,    // 没有可用的新总线槽位
    ERROR_BUS_NOT_FOUND = -5,       // CAN总线未找到或不活动
    ERROR_DEVICE_ALREADY_EXISTS = -6, // 设备已注册
    ERROR_NO_AVAILABLE_DEVICE_SLOT = -7, // 没有可用的新设备槽位
    ERROR_DEVICE_NOT_FOUND = -8,    // 设备未找到
    ERROR_MESSAGE_QUEUE_FULL = -9,  // 消息队列已满
    ERROR_HARDWARE_FAILURE = -10,   // 硬件操作失败
    ERROR_MEMORY_ALLOCATION = -11,  // 内存分配失败
    ERROR_TIMEOUT = -12,            // 操作超时
    ERROR_PERMISSION_DENIED = -13,  // 权限被拒绝
    ERROR_DEVICE_DISCONNECTED = -14, // 设备已断开连接
    ERROR_INTERNAL_ERROR = -15,      // 内部错误
    ERROR_DEVICE_NOT_REGISTERED = -16, // 设备未注册
    ERROR_BUS_NOT_ACTIVE = -17,      // 总线未活动
    ERROR_DEVICE_REGISTRATION_FAILED = -18, // 设备注册失败
    ERROR_NOT_IMPLEMENTED = -19,     // 功能未实现
};

enum class CanBusModelType {
    BUSMUST_A, // BUSMUST A 款
    BUSMUST_B, // BUSMUST B 款
    LEJU_CAN_A,  // Leju CAN A
    LEJU_CAN_B,  // Leju CAN B
    UNKNOWN,   // 未知
};

struct CanBusBitrate {
    int nbitrate;       // 仲裁段波特率
    int dbitrate;      // 数据段波特率
    int nsamplepos;    // 仲裁段采样点 (%)
    int dsamplepos;    // 数据段采样点 (%)
};

/**
 * @brief 设备唯一ID - 使用位域避免设备ID冲突
 */
union DeviceUniqueId {
    uint32_t value;  // 整体访问

    struct {
        uint32_t device_id : 24;    // 设备ID（低24位）
        uint32_t device_type : 8;   // 设备类型（高8位）
    };

    // 构造函数
    DeviceUniqueId() : value(0) {}
    explicit DeviceUniqueId(uint32_t val) : value(val) {}
    DeviceUniqueId(DeviceType type, uint32_t id) {
        device_type = static_cast<uint8_t>(type);
        device_id = id;
    }

    // 便捷访问方法
    DeviceType getType() const { return static_cast<DeviceType>(device_type); }
    uint32_t getDeviceId() const { return device_id; }

    // 相等运算符重载
    bool operator==(const DeviceUniqueId& other) const {
        return value == other.value;
    }
    bool operator!=(const DeviceUniqueId& other) const {
        return value != other.value;
    }
};

struct DeviceInfo {
    std::string device_name;     // 设备名称
    DeviceType device_type;      // 设备类型
    DeviceId device_id;          // 设备在总线上的 ID
    std::function<bool(uint32_t can_id, uint32_t device_id)> matcher; // CAN ID 匹配器函数

    // 默认构造函数
    DeviceInfo() : device_type(DeviceType::UNKNOWN), device_id(-1) {}

    // 构造函数
    DeviceInfo(const std::string& name, DeviceType type, DeviceId id,
               std::function<bool(uint32_t can_id, uint32_t device_id)> matcher_func)
        : device_name(name), device_type(type), device_id(id), matcher(matcher_func) {}

    // 获取设备唯一ID
    DeviceUniqueId getUniqueId() const {
        return DeviceUniqueId(device_type, device_id);
    }
};

struct CanMessageId {
    uint32_t SID : 11;                          /**< Standard ID */
    uint32_t EID : 18;                          /**< Extended ID */
    uint32_t SID11 : 1;                         /**< Reserved */
    uint32_t unimplemented1 : 2;                /**< Reserved */
};

struct CanMessageCtrl {
    struct {
        uint32_t dlc : 4;  // Data Length Code for transmission (4 bits)
        uint32_t reserved : 28;  // Reserved bits (28 bits)
    } tx;
    struct {
        uint32_t dlc : 4;  // Data Length Code for reception (4 bits)
        uint32_t reserved : 28;  // Reserved bits (28 bits)
    } rx;
};

struct CanMessageFrame {
    struct CanMessageId id;
    struct CanMessageCtrl ctrl;
    uint8_t payload[64];
};

struct CallbackContext {
    void* userdata;
};

/**
 * @brief CAN总线消息回调函数类型
 *
 * @param frame 指向接收到的CAN消息帧的指针（堆分配）
 * @param context 回调注册时传递的可选用户上下文
 * @note 需要由使用者负责使用freeCanMessageFrame()释放帧。
 */
typedef void (*MessageCallback)(CanMessageFrame* frame, const CallbackContext* context);

/**
 * @brief TEF异步发送完成事件回调函数类型
 *
 * @param frame 指向接收到的CAN消息帧的指针（堆分配）
 * @param context 回调注册时传递的可选用户上下文
 * @note 用于处理TEF发送完成件的回调函数,可用于确认异步发送消息时，消息是否成功发出。
 *       需要由使用者负责使用freeCanMessageFrame()释放帧。
 */
typedef void (*TefEeventCallback)(CanMessageFrame* frame, const CallbackContext* context);


struct CallbackParams {
    // 消息回调
    MessageCallback msg_callback;
    const CallbackContext* msg_cb_ctx;
    // TEF事件回调
    TefEeventCallback tef_callback;
    const CallbackContext* tef_cb_ctx;
};

} // namespace canbus_sdk

// 为了支持std::unordered_map，需要特化std::hash
namespace std {
    template<>
    struct hash<canbus_sdk::DeviceUniqueId> {
        size_t operator()(const canbus_sdk::DeviceUniqueId& key) const {
            return std::hash<uint32_t>()(key.value);
        }
    };
}

#endif // CANBUS_SDK_DEF_H