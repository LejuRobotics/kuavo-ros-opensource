#ifndef CANBUS_SDK_H
#define CANBUS_SDK_H
#include "canbus_sdk/canbus_sdk_def.h"
#include "canbus_sdk/result.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <memory>
#include <functional>
#include <pthread.h>

namespace canbus_sdk {

/**
 * @brief 将CAN DLC（数据长度码）转换为实际有效载荷长度
 * @param dlc 数据长度码（标准CAN为0-8，CAN FD为9-15）
 * @return 实际有效载荷长度（字节）
 */
uint8_t dlc_to_payload_length(uint8_t dlc);

/**
 * @brief 将有效载荷长度转换为CAN DLC（数据长度码）
 * @param payload_length 有效载荷长度（字节）
 * @return 对应的DLC值
 */
uint8_t payload_length_to_dlc(uint8_t payload_length);

/**
 * @brief 创建新的CAN消息帧
 * @return 新分配的CanMessageFrame指针，如果分配失败则返回nullptr
 */
CanMessageFrame* createCanMessageFrame();

/**
 * @brief 释放CAN消息帧
 * @param frame 要释放的CanMessageFrame指针
 */
void freeCanMessageFrame(CanMessageFrame* frame);

/**
 * @brief 将错误码转换为可读字符串
 * @param error 要转换的错误码
 * @return 错误的字符串表示
 */
const char* errorToString(CanBusError error);

/**
 * @brief 将错误码转换为可读字符串
 * @param error 要转换的错误码
 * @return 错误的字符串表示
 */
const char* errorToString(ErrorType error);

/**
 * @brief 将CAN总线模型类型转换为可读字符串
 * @param type 要转换的CAN总线模型类型
 * @return CAN总线模型类型的字符串表示
 */
const char* to_string(CanBusModelType type);

/**
 * @brief 将设备类型转换为可读字符串
 * @param type 要转换的设备类型
 * @return 设备类型的字符串表示
 */
const char* to_string(DeviceType type);

/**
 * @brief 将可读字符串转换为CAN总线模型类型
 * @param str 要转换的字符串
 * @return CAN总线模型类型
 */
CanBusModelType canbus_model_from_string(const std::string& str);

/**
 * @brief 将可读字符串转换为设备类型
 * @param str 要转换的字符串
 * @return 设备类型
 */
DeviceType device_type_from_string(const std::string& str);


/**
 * @brief CAN总线控制器类，用于管理CAN总线连接和设备
 */
class CanBusController {
public:
    /**
     * @brief 获取CanBusController的单例实例
     * @return 单例实例的引用
     */
    static CanBusController& getInstance();

    /**
     * @brief 初始化CAN总线控制器
     */
    void init();

    /**
     * @brief 打开并配置CAN总线连接
     * @param bus_name CAN总线接口名称
     * @param canbus_type CAN总线硬件类型
     * @param bitrate CAN总线比特率配置
     * @return 成功返回总线ID，失败返回错误码
     * 
     * @note 使用示例：
     * @code
     * auto result = CanBusController::getInstance().openCanBus("can0", CanBusModelType::BUSMUST_A, bitrate);
     * if (result.has_value()) {
     *     BusId bus_id = result.value();  // 获取总线ID
     *     // 使用总线ID进行后续操作
     * } else {
     *     ErrorType error_code = result.error();  // 获取错误码
     *     const char* error_msg = errorToString(error_code);  // 获取错误信息
     *     printf("Failed to open CAN bus: %s (error code: %d)\n", error_msg, error_code);
     * }
     * @endcode
     */
    Result<BusId> openCanBus(const std::string& bus_name, CanBusModelType canbus_type, const CanBusBitrate& bitrate);

    /**
     * @brief 关闭CAN总线连接
     * @param bus_name 要关闭的CAN总线名称
     * @return 成功返回成功码，失败返回错误码
     */
    Result<ErrorType> closeCanBus(const std::string& bus_name);

    /**
     * @brief 关闭所有CAN总线连接
     * @return 成功返回成功码，失败返回错误码
     */
    Result<ErrorType> closeAllCanBuses();

    /**
     * @brief 注册设备以接收CAN总线消息
     * @param device_info 设备信息
     * @param canbus_name CAN总线名称
     * @param callback_params 回调参数结构，包含消息回调和TEF回调
     * @return 成功返回成功码，失败返回错误码
     */
    Result<ErrorType> registerDevice(const DeviceInfo& device_info,
                                     const std::string& canbus_name,
                                     const CallbackParams& callback_params);

    /**
     * @brief 从CAN总线注销设备
     * @param device_id 要注销的设备ID
     * @param bus_name CAN总线名称
     * @return 成功返回成功码，失败返回错误码
     */
    Result<ErrorType> unregisterDevice(DeviceType device_type, DeviceId device_id, const std::string& bus_name);

    // ===== 消息发送方法 =====
    /**
     * @brief 向指定总线发送CAN消息帧（异步）
     * @param bus_name CAN总线名称
     * @param frame 要发送的CAN消息帧
     * @return 成功返回成功码，失败返回错误码
     */
    Result<ErrorType> sendMessage(const std::string& bus_name, const CanMessageFrame& frame);

    /**
     * @brief 向指定总线ID发送CAN消息帧（异步）
     * @param bus_id CAN总线ID
     * @param frame 要发送的CAN消息帧
     * @return 成功返回成功码，失败返回错误码
     */
    Result<ErrorType> sendMessage(BusId bus_id, const CanMessageFrame& frame);

    /**
     * @brief 设置发送线程CPU亲和性
     * @param cpu_core CPU核心编号
     * @return 设置是否成功
     */
    bool setSenderThreadAffinity(int cpu_core);

    /**
     * @brief 设置指定CAN总线接收线程CPU亲和性
     * @param bus_name CAN总线名称
     * @param cpu_core CPU核心编号
     * @return 设置是否成功
     */
    bool setRecvThreadAffinity(const std::string& bus_name, int cpu_core);

    /**
     * @brief 设置指定CAN总线接收线程频率
     * @param bus_name CAN总线名称
     * @param frequency_ms 接收线程频率（毫秒）
     * @return 设置是否成功
     */
    bool setRecvThreadFrequency(const std::string& bus_name, int frequency_ms);

    /**
     * @brief 获取指定CAN总线接收线程频率
     * @param bus_name CAN总线名称
     * @return 当前接收线程频率（毫秒），如果总线不存在返回-1
     */
    int getRecvThreadFrequency(const std::string& bus_name) const;

    /**
     * @brief 通过CAN总线名称获取总线ID
     * @param bus_name CAN总线名称
     * @return 成功返回总线ID，失败返回错误码
     *
     * @note 使用示例：
     * @code
     * auto result = CanBusController::getInstance().getBusIdByName("can0");
     * if (result.has_value()) {
     *     BusId bus_id = result.value();
     *     // 使用总线ID进行后续操作
     * } else {
     *     ErrorType error_code = result.error();
     *     printf("Failed to get bus ID: %s\n", errorToString(error_code));
     * }
     * @endcode
     */
    Result<BusId> getBusIdByName(const std::string& bus_name);

private:
    CanBusController() = default;
    ~CanBusController();  // Destructor implementation in cpp file
    CanBusController(const CanBusController&) = delete;
    CanBusController& operator=(const CanBusController&) = delete;

    class Impl;
    struct ImplDeleter {
        void operator()(Impl* ptr) const;
    };
    std::unique_ptr<Impl, ImplDeleter> impl_;
};

} // namespace canbus_sdk

#endif // CANBUS_SDK_H