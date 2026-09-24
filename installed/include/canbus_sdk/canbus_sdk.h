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
    bool setRecvThreadFrequency(const std::string& bus_name, double frequency_ms);

    /**
     * @brief 获取指定CAN总线接收线程频率
     * @param bus_name CAN总线名称
     * @return 当前接收线程频率（毫秒），如果总线不存在返回-1.0
     */
    double getRecvThreadFrequency(const std::string& bus_name) const;

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

    // ===== RT 实时发送 slot =====
    /**
     * @brief 注册一个实时发送源（RT slot）
     *
     * 注册后，sender 线程每轮循环会**先**执行所有 RT 源回调（直写 CAN，绕过 ring 队列），
     * 然后才从 ring 队列 pop 常规消息。RT 源回调在 sender 线程内执行，天然与 ring 消费串行，
     * 不受 blocks_mutex_ 阻塞，也无需额外加锁。
     *
     * @note 用途：电机控制帧等 "latest-wins" 语义的高实时性消息，走 RT 直发路径；
     *       灵巧手/使能/配置等请求-响应型消息继续走 ring，保证不丢帧。
     * @param bus_id 目标总线 ID（由 openCanBus 返回）
     * @param callback RT 源回调：在 sender 线程内，负责取最新 cmd → 编码 CAN 帧 → 直写。
     *                 回调内必须调用 sendMessageRt()（或 bus 级直发），不得调用 sendMessage()（会入队，形成环）。
     * @return 成功返回 true
     */
    bool registerRtSource(BusId bus_id, std::function<void()> callback);

    /**
     * @brief 注销指定总线的 RT 发送源
     * @param bus_id 目标总线 ID
     * @return 成功返回 true
     */
    bool unregisterRtSource(BusId bus_id);

    /**
     * @brief 注销全部 RT 发送源
     * @return 成功返回 true
     */
    bool unregisterRtSources();

    /**
     * @brief 直发一条 CAN 帧（绕过 ring 队列）
     *
     * 线程安全：内部加 blocks_mutex_ 保护，可被任意线程调用。
     *
     * @param bus_id 目标总线 ID
     * @param frame 要发送的 CAN 帧
     * @return 成功返回成功码
     */
    Result<ErrorType> sendMessageRt(BusId bus_id, const CanMessageFrame& frame);

    /**
     * @brief 直发一条 CAN 帧（不加锁版本）
     *
     * 仅供 sender 线程的 RT 源回调内部使用。调用方必须保证已持有 blocks_mutex_
     * （senderThreadFunction 的 RT 扫描段持锁调用 RT 源回调）。外部线程误用会与
     * sender 线程竞争，导致数据竞争。
     *
     * @param bus_id 目标总线 ID
     * @param frame 要发送的 CAN 帧
     * @return 成功返回成功码
     */
    Result<ErrorType> sendMessageRtUnlocked(BusId bus_id, const CanMessageFrame& frame);

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