#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/canbus_log.h"
#include "bus_control_block.h"
#include "bm_canbus_wrapper.h"
#include "lock_free_ring_buffer.h"

#include <mutex>
#include <thread>
#include <atomic>
#include <functional>
#include <chrono>
#include <limits>
#include <condition_variable>
#include <vector>

// 宏控制：是否启用CAN线程统计信息打印（频率、消息计数等）
#ifndef DEBUG_CAN_THREAD_STATS
#define DEBUG_CAN_THREAD_STATS 0
#endif

namespace canbus_sdk {

///////////////////////////////////////////////////////////////////////////////
// CanMessageQueueItem - Message queue item for CAN frame transmission
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Message queue item for CAN frame transmission
 */
struct CanMessageQueueItem {
    BusId bus_id;                             ///< Target CAN bus ID (index)
    CanMessageFrame frame;                   ///< CAN message frame to send
    
    CanMessageQueueItem() = default;
    
    CanMessageQueueItem(BusId id, const CanMessageFrame& f) 
        : bus_id(id), frame(f) {}
};

// Ring buffer size (must be power of 2)
constexpr size_t CAN_MESSAGE_QUEUE_SIZE = 1024*8;

/**
 * @brief Type alias for lock-free CAN message queue
 */
using CanMessageQueue = LockFreeRingBuffer<CanMessageQueueItem, CAN_MESSAGE_QUEUE_SIZE>;

///////////////////////////////////////////////////////////////////////////////
// CanBusController::Impl - Implementation class for CanBusController
///////////////////////////////////////////////////////////////////////////////

class CanBusController::Impl {
public:
    Impl();
    ~Impl();

    static constexpr int MAX_CANBUS_HANDLERS = 10;
    std::unique_ptr<BusControlBlock> canbus_blocks_[MAX_CANBUS_HANDLERS];  ///< CAN总线控制块数组
    std::mutex blocks_mutex_;                   ///< 保护canbus_blocks_数组的互斥锁
    int active_bus_count_;                      ///< 活动总线连接数量

    // 消息队列和发送线程
    CanMessageQueue message_queue_;
    std::thread sender_thread_;
    std::atomic<bool> sender_running_{false};
    int sender_thread_cpu_core_;               ///< 发送线程绑定的CPU核心，-1表示未绑定

    // 条件变量：队列有新消息时唤醒发送线程
    std::mutex queue_cv_mutex_;
    std::condition_variable queue_cv_;

    // RT 实时发送源（RT slot）：sender 线程每轮先执行这些回调（直写 CAN，绕过 ring）
    struct RtSource {
        BusId bus_id;                         ///< 目标总线 ID
        std::function<void()> callback;       ///< RT 源回调（在 sender 线程内执行）
    };
    std::vector<std::unique_ptr<RtSource>> rt_sources_;   ///< RT 源列表
    std::mutex rt_sources_mutex_;                         ///< 保护 rt_sources_ 的互斥锁

    void startSenderThread();
    void stopSenderThread();
    void senderThreadFunction();

    // 按名称查找总线用于消息分发
    BusControlBlock* findBusByName(const std::string& bus_name);

    // 创建并初始化新的总线控制块
    Result<BusId> createBusControlBlock(const std::string& bus_name,
                                       CanBusModelType canbus_type, const CanBusBitrate& bitrate);

    // 根据总线名称获取总线ID
    BusId getBusIdByName(const std::string& bus_name) const;

    // DeviceHandler 使用的消息入队方法
    Result<ErrorType> enqueueMessage(BusId bus_id, const CanMessageFrame& frame);

  };

CanBusController::Impl::Impl() : active_bus_count_(0), sender_thread_cpu_core_(-1) {
    LOG_I("CAN bus controller initializing");
    // BusControlBlock数组由其构造函数自动初始化
    startSenderThread();
    LOG_I("CAN bus controller initialized successfully");
}

CanBusController::Impl::~Impl() {
    LOG_I("CAN bus controller destroying");
    stopSenderThread();
    // 无需手动清理
    LOG_I("CAN bus controller destroyed");
}

BusControlBlock* CanBusController::Impl::findBusByName(const std::string& bus_name) {
    std::lock_guard<std::mutex> lock(blocks_mutex_);
    for (int i = 0; i < MAX_CANBUS_HANDLERS; ++i) {
        if (canbus_blocks_[i] && canbus_blocks_[i]->active() && canbus_blocks_[i]->busName() == bus_name) {
            return canbus_blocks_[i].get();
        }
    }
    return nullptr;
}

BusId CanBusController::Impl::getBusIdByName(const std::string& bus_name) const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(blocks_mutex_));
    for (int i = 0; i < MAX_CANBUS_HANDLERS; ++i) {
        if (canbus_blocks_[i] && canbus_blocks_[i]->active() && canbus_blocks_[i]->busName() == bus_name) {
            // 验证槽位ID是否在BusId范围内
            if (i > std::numeric_limits<BusId>::max()) {
                LOG_E("Bus slot ID %d exceeds BusId range", i);
                return static_cast<BusId>(-1);  // 返回无效ID
            }
            return static_cast<BusId>(i);
        }
    }
    return static_cast<BusId>(-1);  // 未找到，返回无效ID
}

Result<ErrorType> CanBusController::Impl::enqueueMessage(BusId bus_id, const CanMessageFrame& frame) {
    // 验证总线 ID 是否有效
    if (bus_id == static_cast<BusId>(-1) || bus_id >= MAX_CANBUS_HANDLERS || !canbus_blocks_[bus_id]) {
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_BUS_NOT_FOUND));
    }
    
    // 验证总线是否活跃
    if (!canbus_blocks_[bus_id]->active()) {
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_BUS_NOT_ACTIVE));
    }
    
    // 创建消息队列项并推送到队列
    CanMessageQueueItem item(bus_id, frame);
    
    if (!message_queue_.push(item)) {
        LOG_FAILURE("Message queue FULL! bus_id=%d, CAN ID=0x%03X", bus_id, frame.id.SID);
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_MESSAGE_QUEUE_FULL));
    }

    // 唤醒发送线程
    queue_cv_.notify_one();

    // LOG_D("Message queued successfully for bus ID %d", bus_id);
    return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
}

Result<BusId> CanBusController::Impl::createBusControlBlock(const std::string& bus_name, 
                                                             CanBusModelType canbus_type, const CanBusBitrate& bitrate) {
    LOG_D("Creating bus control block for %s", bus_name.c_str());
    
    std::lock_guard<std::mutex> lock(blocks_mutex_);
    
    // 查找空槽位
    int empty_slot = -1;
    for (int i = 0; i < MAX_CANBUS_HANDLERS; ++i) {
        if (!canbus_blocks_[i]) {
            empty_slot = i;
            break;
        }
    }
    
    if (empty_slot == -1) {
        return Result<BusId>::error(static_cast<ErrorType>(CanBusError::ERROR_NO_AVAILABLE_SLOT));
    }
    
    // 验证槽位ID是否在BusId范围内
    if (empty_slot > std::numeric_limits<BusId>::max()) {
        LOG_E("Bus slot ID %d exceeds BusId range", empty_slot);
        return Result<BusId>::error(static_cast<ErrorType>(CanBusError::ERROR_INTERNAL_ERROR));
    }
    
    // 创建并初始化总线控制块
    canbus_blocks_[empty_slot] = BusControlBlock::create(bus_name, canbus_type, bitrate);
    if (!canbus_blocks_[empty_slot]) {
        LOG_E("Failed to create bus control block for %s", bus_name.c_str());
        return Result<BusId>::error(static_cast<ErrorType>(CanBusError::ERROR_HARDWARE_FAILURE));  // 创建失败
    }
    
    active_bus_count_++;
    LOG_D("Bus control block created successfully in slot %d, active bus count: %d", empty_slot, active_bus_count_);
    return Result<BusId>::ok(static_cast<BusId>(empty_slot));
}

void CanBusController::Impl::startSenderThread() {
    LOG_I("Starting CAN message sender thread");
    sender_running_.store(true, std::memory_order_release);
    sender_thread_ = std::thread(&CanBusController::Impl::senderThreadFunction, this);

    // 如果之前设置了CPU核心绑定，则重新应用
    if (sender_thread_cpu_core_ >= 0) {
        // 获取线程native句柄
        pthread_t thread = sender_thread_.native_handle();

        // 设置CPU亲和性
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(sender_thread_cpu_core_, &cpuset);

        int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
        if (result == 0) {
            LOG_I("Set sender thread affinity to CPU core %d", sender_thread_cpu_core_);
        } else {
            LOG_E("Failed to set sender thread affinity to CPU core %d: %s",
                  sender_thread_cpu_core_, strerror(result));
        }
    }

    LOG_I("CAN message sender thread started");
}

void CanBusController::Impl::stopSenderThread() {
    if (sender_thread_.joinable()) {
        LOG_I("Stopping CAN message sender thread");
        sender_running_.store(false, std::memory_order_release);
        queue_cv_.notify_one();  // 唤醒可能在等待的发送线程
        sender_thread_.join();
        LOG_I("CAN message sender thread stopped");
    }
}

void CanBusController::Impl::senderThreadFunction() {
    LOG_I("CAN message sender thread started");
    int message_count = 0;
    uint64_t failed_writes = 0;
    auto last_write_error_log = std::chrono::steady_clock::time_point{};

    while (sender_running_.load(std::memory_order_acquire)) {
        // ① 逐总线扫 RT：每条总线电机直发后立刻给该总线 ring 让出几帧。
        //    左手在 canbus0（9 电机），不能等 canbus1 的 7 帧 RT 扫完才发手。
        constexpr int kMaxAfterEachRt = 8;
        constexpr int kMaxQueueFramesPerSweep = 32;
        int drained = 0;
        auto send_queued = [&](const CanMessageQueueItem& item) {
            bool message_sent = false;
            {
                std::lock_guard<std::mutex> lock(blocks_mutex_);
                if (item.bus_id != static_cast<BusId>(-1) && item.bus_id < MAX_CANBUS_HANDLERS &&
                    canbus_blocks_[item.bus_id] && canbus_blocks_[item.bus_id]->active()) {
                    const auto result = canbus_blocks_[item.bus_id]->sendMessage(item.frame);
                    message_sent = result.has_value() && result.value() == 0;
                    if (message_sent) {
                        message_count++;
                    } else {
                        ++failed_writes;
                        const auto now = std::chrono::steady_clock::now();
                        if (now - last_write_error_log >= std::chrono::seconds(1)) {
                            const auto error = result.has_value() ? result.value() : result.error();
                            LOG_W("CAN queued write failed: bus=%d id=0x%X error=%d total=%llu",
                                  item.bus_id, item.frame.id.SID, static_cast<int>(error),
                                  static_cast<unsigned long long>(failed_writes));
                            last_write_error_log = now;
                        }
                        // No retry of a stale motion frame; the producer refreshes its target.
                        return;
                    }
                }
            }
            if (!message_sent) {
                LOG_W("Target bus ID %d not found or inactive, message discarded", item.bus_id);
            }
        };

        size_t nsrc = 0;
        {
            std::lock_guard<std::mutex> rlock(rt_sources_mutex_);
            nsrc = rt_sources_.size();
        }
        for (size_t i = 0; i < nsrc; ++i) {
            {
                std::lock_guard<std::mutex> lock(blocks_mutex_);
                std::lock_guard<std::mutex> rlock(rt_sources_mutex_);
                if (i < rt_sources_.size() && rt_sources_[i] && rt_sources_[i]->callback) {
                    rt_sources_[i]->callback();
                }
            }
            CanMessageQueueItem item;
            int n = 0;
            while (n < kMaxAfterEachRt && drained < kMaxQueueFramesPerSweep && message_queue_.pop(item)) {
                send_queued(item);
                n++;
                drained++;
            }
        }

        CanMessageQueueItem item;
        while (drained < kMaxQueueFramesPerSweep && message_queue_.pop(item)) {
            send_queued(item);
            drained++;
        }

        if (drained == 0) {
            std::unique_lock<std::mutex> lk(queue_cv_mutex_);
            queue_cv_.wait_for(lk, std::chrono::milliseconds(1), [this]() {
                return !message_queue_.empty() || !sender_running_.load(std::memory_order_acquire);
            });
        }
    }

    // 停止后处理剩余项目（优雅关闭）
    LOG_I("Processing remaining messages in queue");
    int remaining_count = 0;
    constexpr int MAX_REMAINING_MESSAGES = 1000;  // 防止无限处理
    CanMessageQueueItem item;

    while (remaining_count < MAX_REMAINING_MESSAGES && message_queue_.pop(item)) {
        // 快速处理剩余项目，同样需要线程保护
        {
            std::lock_guard<std::mutex> lock(blocks_mutex_);
            if (item.bus_id != static_cast<BusId>(-1) && item.bus_id < MAX_CANBUS_HANDLERS &&
                canbus_blocks_[item.bus_id] && canbus_blocks_[item.bus_id]->active()) {
                canbus_blocks_[item.bus_id]->sendMessage(item.frame);
                remaining_count++;
            }
        }
    }

    if (remaining_count > 0) {
        LOG_I("Processed %d remaining messages", remaining_count);
        if (remaining_count >= MAX_REMAINING_MESSAGES) {
            LOG_W("Reached maximum remaining messages limit, some messages may be discarded");
        }
    }
    #if DEBUG_CAN_THREAD_STATS
    LOG_I("CAN message sender thread ended, total messages sent: %d", message_count);
#endif
}


///////////////////////////////////////////////////////////////////////////////
// CanBusController - Main controller class implementation
///////////////////////////////////////////////////////////////////////////////

CanBusController& CanBusController::getInstance() {
    static CanBusController instance;
    return instance;
}

void CanBusController::init() {
    if (!impl_) {
        LOG_I("Initializing CAN bus controller");
        impl_.reset(new Impl());
        LOG_I("CAN bus controller initialized");
    } else {
        LOG_W("CAN bus controller already initialized");
    }
}

Result<BusId> CanBusController::openCanBus(const std::string& bus_name, CanBusModelType canbus_type, const CanBusBitrate& bitrate) {
    LOG_I("Opening CAN bus: %s, type: %d, bitrate: %d/%d", bus_name.c_str(), static_cast<int>(canbus_type), bitrate.nbitrate, bitrate.dbitrate);

    if (!impl_) {
        LOG_FAILURE("Opening CAN bus: %s failed, CAN bus controller not initialized", bus_name.c_str());
        return Result<BusId>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }

    // 检查总线是否已打开
    BusId existing_bus_id = impl_->getBusIdByName(bus_name);
    if (existing_bus_id != static_cast<BusId>(-1)) {
        LOG_W("CAN bus %s already open with ID %d", bus_name.c_str(), existing_bus_id);
        return Result<BusId>::ok(existing_bus_id);  // 总线已打开，返回现有ID
    }
    
    // 创建并初始化总线控制块（内部处理槽位查找和分配）
    auto result = impl_->createBusControlBlock(bus_name, canbus_type, bitrate);
    if (result.has_value()) {
        BusId bus_id = result.value();
        LOG_SUCCESS("CAN bus %s opened successfully with ID %d", bus_name.c_str(), bus_id);
        return Result<BusId>::ok(bus_id);
    } else {
        LOG_FAILURE("Failed to open CAN bus %s, error: %s", bus_name.c_str(), errorToString(result.error()));
        return Result<BusId>::error(result.error());
    }
}

Result<ErrorType> CanBusController::closeCanBus(const std::string& bus_name) {
    LOG_I("Closing CAN bus: %s", bus_name.c_str());

    if (!impl_) {
        LOG_E("close bus failed, CAN bus controller not initialized");
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }

    // 验证输入参数
    if (bus_name.empty()) {
        LOG_E("close bus failed, Empty bus name provided");
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_INVALID_PARAMETER));
    }

    bool bus_found = false;
    int final_count = 0;

    {
        std::lock_guard<std::mutex> lock(impl_->blocks_mutex_);
        // 查找目标CAN总线
        for (int i = 0; i < impl_->MAX_CANBUS_HANDLERS; ++i) {
            if (impl_->canbus_blocks_[i] && impl_->canbus_blocks_[i]->busName() == bus_name) {
                // 找到总线，关闭并清理
                impl_->canbus_blocks_[i].reset();
                if (impl_->active_bus_count_ > 0) {
                    impl_->active_bus_count_--;
                }
                bus_found = true;
                final_count = impl_->active_bus_count_;
                break;
            }
        }
    }

    if (bus_found) {
        LOG_SUCCESS("CAN bus %s closed successfully, active bus count: %d", bus_name.c_str(), final_count);
        return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
    } else {
        LOG_E("close bus failed, CAN bus %s not found", bus_name.c_str());
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_BUS_NOT_FOUND));
    }
}

Result<ErrorType> CanBusController::closeAllCanBuses() {
    LOG_I("Closing all CAN buses");

    if (!impl_) {
        LOG_E("close all buses failed, CAN bus controller not initialized");
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }

    int closed_count = 0;
    std::vector<std::string> closed_bus_names;

    {
        std::lock_guard<std::mutex> lock(impl_->blocks_mutex_);
        // 遍历所有CAN总线并关闭
        for (int i = 0; i < impl_->MAX_CANBUS_HANDLERS; ++i) {
            if (impl_->canbus_blocks_[i]) {
                // 记录总线名称用于日志
                closed_bus_names.push_back(impl_->canbus_blocks_[i]->busName());
                // 关闭并清理总线
                impl_->canbus_blocks_[i].reset();
                closed_count++;
            }
        }

        // 重置活动总线计数
        impl_->active_bus_count_ = 0;
    }

    if (closed_count > 0) {
        LOG_SUCCESS("Closed %d CAN buses successfully, active bus count: %d", closed_count, impl_->active_bus_count_);
        // 记录关闭的总线名称
        for (const auto& name : closed_bus_names) {
            LOG_D("Closed bus: %s", name.c_str());
        }
        return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
    } else {
        LOG_W("No active CAN buses found to close");
        return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
    }
}

// ===== 设备管理方法实现 =====

Result<ErrorType> CanBusController::registerDevice(const DeviceInfo& device_info,
                                                   const std::string& canbus_name,
                                                   const CallbackParams& callback_params) {
    if (!impl_) {
        LOG_FAILURE("Registering device: %s (ID: 0x%04X) on CAN bus: %s failed, CAN bus controller not initialized",
            device_info.device_name.c_str(), device_info.device_id, canbus_name.c_str());
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }

    // 验证输入参数
    if (device_info.device_name.empty() || canbus_name.empty() || callback_params.msg_callback == nullptr) {
        LOG_FAILURE("Registering device: %s (ID: 0x%04X) on CAN bus: %s failed, Invalid parameters for device registration",
            device_info.device_name.c_str(), device_info.device_id, canbus_name.c_str());
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_INVALID_PARAMETER));
    }

    // 查找目标CAN总线
    BusControlBlock* target_bus = impl_->findBusByName(canbus_name);

    if (target_bus == nullptr) {
        LOG_FAILURE("Registering device: %s (ID: 0x%04X) on CAN bus: %s failed, CAN bus not found",
            device_info.device_name.c_str(), device_info.device_id, canbus_name.c_str());
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_BUS_NOT_FOUND));
    }

    // 向总线添加设备
    if (!target_bus->addDevice(device_info, callback_params)) {
        // 检查设备是否已存在
        if (target_bus->hasDevice(device_info.getUniqueId())) {
            LOG_E("Registering device, Device %s (ID: %d) already exists on bus %s",
                  device_info.device_name.c_str(), device_info.device_id, canbus_name.c_str());
            return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_DEVICE_ALREADY_EXISTS));
        } else {
            LOG_E("Registering device, No available device slot for device %s on bus %s",
                  device_info.device_name.c_str(), canbus_name.c_str());
            return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_NO_AVAILABLE_DEVICE_SLOT));
        }
    }

    LOG_SUCCESS("Registering device, Device %s (ID: %d, UniqueID: 0x%08X) registered successfully on bus %s",
          device_info.device_name.c_str(), device_info.device_id, device_info.getUniqueId().value, canbus_name.c_str());

    return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
}

Result<ErrorType> CanBusController::unregisterDevice(DeviceType device_type, DeviceId device_id, const std::string& bus_name) {    
    if (!impl_) {
        LOG_FAILURE("Unregistering device ID: %d from CAN bus: %s fail, CAN bus controller not initialized", device_id, bus_name.c_str());
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }
  
    // 验证输入参数
    if (bus_name.empty() || device_id < 0 || device_type == DeviceType::UNKNOWN) {
        LOG_FAILURE("Unregistering device type %d ID: %d from CAN bus: %s failed, Invalid parameters for device unregistration",
                   static_cast<int>(device_type), device_id, bus_name.c_str());
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_INVALID_PARAMETER));
    }
    
    // 查找目标CAN总线
    BusControlBlock* target_bus = impl_->findBusByName(bus_name);
  
    if (target_bus == nullptr) {
        LOG_FAILURE("Unregistering device type %d ID: %d from CAN bus: %s failed, CAN bus not found",
                   static_cast<int>(device_type), device_id, bus_name.c_str());
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_BUS_NOT_FOUND));
    }

    // 从总线移除设备
    if (!target_bus->removeDevice(device_type, device_id)) {
        LOG_FAILURE("Unregistering device type %d ID: %d from CAN bus: %s failed, Device not found",
                   static_cast<int>(device_type), device_id, bus_name.c_str());
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_DEVICE_NOT_FOUND));
    }

    // 获取剩余设备数，之后不再使用 target_bus 指针
    int remaining_devices = target_bus->deviceCount();
    target_bus = nullptr;  // 显式置空，防止后续误用

    // 检查总线是否没有设备了，如果是则自动关闭总线
    if (remaining_devices == 0) {
        LOG_I("Bus %s has no more devices, closing automatically", bus_name.c_str());
        auto close_result = closeCanBus(bus_name);
        if (!close_result) {
            LOG_W("Failed to auto-close bus %s: %s", bus_name.c_str(), errorToString(close_result.error()));
        }
    }

    LOG_SUCCESS("Device type %d ID %d (UniqueID: 0x%08X) unregistered successfully from bus %s",
               static_cast<int>(device_type), device_id, DeviceUniqueId(device_type, device_id).value, bus_name.c_str());
    return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
}

// ===== 消息发送方法实现 =====

Result<ErrorType> CanBusController::sendMessage(const std::string& bus_name, const CanMessageFrame& frame) {
    // LOG_D("Sending message to bus %s, ID: 0x%03X, DLC: %d",
    //       bus_name.c_str(), frame.id.SID, frame.ctrl.tx.dlc);
    
    if (!impl_) {
        // LOG_E("CAN bus controller not initialized");
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }
    
    // 验证输入参数
    if (bus_name.empty()) {
        LOG_E("Empty bus name provided for message sending");
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_INVALID_PARAMETER));
    }
    
    // 获取总线ID并调用bus_id版本的sendMessage
    BusId bus_id = impl_->getBusIdByName(bus_name);
    return sendMessage(bus_id, frame);
}

Result<ErrorType> CanBusController::sendMessage(BusId bus_id, const CanMessageFrame& frame) {
    // LOG_D("Sending message to bus ID %d, ID: 0x%03X, DLC: %d",
    //       bus_id, frame.id.SID, frame.ctrl.tx.dlc);

    if (!impl_) {
        // LOG_E("CAN bus controller not initialized");
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }

    // 验证输入参数
    if (bus_id == static_cast<BusId>(-1)) {
        LOG_E("Invalid bus ID %d provided for message sending", bus_id);
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_INVALID_PARAMETER));
    }

    // 委托给 enqueueMessage 处理具体的消息入队逻辑
    return impl_->enqueueMessage(bus_id, frame);

    // 先看看ringbuffer 队列的方式有无问题，如果有那么可以直接采用如下方式
    // 验证总线 ID 是否有效
    // if (bus_id >= impl_->MAX_CANBUS_HANDLERS || !impl_->canbus_blocks_[bus_id]) {
    //     return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_BUS_NOT_FOUND));
    // }

    // // 验证总线是否活跃
    // if (!impl_->canbus_blocks_[bus_id]->active()) {
    //     return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_BUS_NOT_ACTIVE));
    // }

    // // 直接发送消息，不通过队列
    // std::lock_guard<std::mutex> lock(impl_->blocks_mutex_);
    // impl_->canbus_blocks_[bus_id]->sendMessage(frame);
    // return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
}

Result<BusId> CanBusController::getBusIdByName(const std::string& bus_name) {
    // LOG_D("Getting bus ID for bus name: %s", bus_name.c_str());

    if (!impl_) {
        LOG_E("CAN bus controller not initialized");
        return Result<BusId>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }

    // 验证输入参数
    if (bus_name.empty()) {
        LOG_E("Empty bus name provided");
        return Result<BusId>::error(static_cast<ErrorType>(CanBusError::ERROR_INVALID_PARAMETER));
    }

    {
        std::lock_guard<std::mutex> lock(impl_->blocks_mutex_);
        // 查找目标CAN总线
        for (int i = 0; i < impl_->MAX_CANBUS_HANDLERS; ++i) {
            if (impl_->canbus_blocks_[i] && impl_->canbus_blocks_[i]->busName() == bus_name) {
                BusId bus_id = static_cast<BusId>(i);
                // LOG_D("Found bus ID %d for bus name: %s", bus_id, bus_name.c_str());
                return Result<BusId>::ok(bus_id);
            }
        }
    }

    LOG_W("Bus not found for name: %s", bus_name.c_str());
    return Result<BusId>::error(static_cast<ErrorType>(CanBusError::ERROR_BUS_NOT_FOUND));
}

bool CanBusController::setSenderThreadAffinity(int cpu_core) {
    if (!impl_) {
        LOG_W("Cannot set sender thread affinity: controller not initialized");
        return false;
    }

    if (cpu_core < 0) {
        LOG_W("Invalid CPU core %d for sender thread", cpu_core);
        return false;
    }

    if (!impl_->sender_thread_.joinable()) {
        LOG_W("Sender thread not running, cannot set affinity");
        return false;
    }

    // 获取线程native句柄
    pthread_t thread = impl_->sender_thread_.native_handle();

    // 设置CPU亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);

    int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (result == 0) {
        impl_->sender_thread_cpu_core_ = cpu_core;
        LOG_I("Set sender thread affinity to CPU core %d", cpu_core);
        return true;
    } else {
        LOG_E("Failed to set sender thread affinity to CPU core %d: %s",
              cpu_core, strerror(result));
        return false;
    }
}

bool CanBusController::setRecvThreadAffinity(const std::string& bus_name, int cpu_core) {
    if (!impl_) {
        LOG_W("Cannot set receive thread affinity: controller not initialized");
        return false;
    }

    if (bus_name.empty()) {
        LOG_W("Empty bus name provided for receive thread affinity");
        return false;
    }

    if (cpu_core < 0) {
        LOG_W("Invalid CPU core %d for bus %s", cpu_core, bus_name.c_str());
        return false;
    }

    // 查找目标CAN总线
    BusControlBlock* target_bus = impl_->findBusByName(bus_name);
    if (target_bus == nullptr) {
        LOG_W("Bus %s not found for receive thread affinity", bus_name.c_str());
        return false;
    }

    // 调用BusControlBlock的setRecvThreadAffinity方法
    return target_bus->setRecvThreadAffinity(cpu_core);
}

bool CanBusController::setRecvThreadFrequency(const std::string& bus_name, double frequency_ms) {
    if (!impl_) {
        LOG_W("Cannot set receive thread frequency: controller not initialized");
        return false;
    }

    if (bus_name.empty()) {
        LOG_W("Empty bus name provided for receive thread frequency");
        return false;
    }

    if (frequency_ms <= 0.0) {
        LOG_W("Invalid receive thread frequency %.2f ms for bus %s", frequency_ms, bus_name.c_str());
        return false;
    }

    // 查找目标CAN总线
    BusControlBlock* target_bus = impl_->findBusByName(bus_name);
    if (target_bus == nullptr) {
        LOG_W("Bus %s not found for receive thread frequency", bus_name.c_str());
        return false;
    }

    // 调用BusControlBlock的setRecvThreadFrequency方法
    return target_bus->setRecvThreadFrequency(frequency_ms);
}

double CanBusController::getRecvThreadFrequency(const std::string& bus_name) const {
    if (!impl_) {
        LOG_W("Cannot get receive thread frequency: controller not initialized");
        return -1.0;
    }

    if (bus_name.empty()) {
        LOG_W("Empty bus name provided for receive thread frequency");
        return -1.0;
    }

    // 查找目标CAN总线
    BusControlBlock* target_bus = impl_->findBusByName(bus_name);
    if (target_bus == nullptr) {
        LOG_W("Bus %s not found for receive thread frequency", bus_name.c_str());
        return -1.0;
    }

    // 调用BusControlBlock的getRecvThreadFrequency方法
    return target_bus->getRecvThreadFrequency();
}

// ===== RT 实时发送 slot 实现 =====

bool CanBusController::registerRtSource(BusId bus_id, std::function<void()> callback) {
    if (!impl_) {
        LOG_E("Register RT source failed, CAN bus controller not initialized");
        return false;
    }
    if (bus_id == static_cast<BusId>(-1) || bus_id >= impl_->MAX_CANBUS_HANDLERS || !callback) {
        LOG_E("Register RT source failed, invalid bus_id=%d or empty callback", static_cast<int>(bus_id));
        return false;
    }

    std::lock_guard<std::mutex> lock(impl_->rt_sources_mutex_);
    auto src = std::unique_ptr<Impl::RtSource>(new Impl::RtSource());
    src->bus_id = bus_id;
    src->callback = std::move(callback);
    impl_->rt_sources_.push_back(std::move(src));
    LOG_I("Registered RT source for bus_id=%d, total RT sources: %zu",
          static_cast<int>(bus_id), impl_->rt_sources_.size());
    return true;
}

bool CanBusController::unregisterRtSource(BusId bus_id) {
    if (!impl_) {
        LOG_W("Unregister RT source failed, CAN bus controller not initialized");
        return false;
    }
    if (bus_id == static_cast<BusId>(-1) || bus_id >= impl_->MAX_CANBUS_HANDLERS) {
        LOG_W("Unregister RT source failed, invalid bus_id=%d", static_cast<int>(bus_id));
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->rt_sources_mutex_);
    auto it = impl_->rt_sources_.begin();
    while (it != impl_->rt_sources_.end()) {
        if ((*it) && (*it)->bus_id == bus_id) {
            it = impl_->rt_sources_.erase(it);
            LOG_I("Unregistered RT source for bus_id=%d", static_cast<int>(bus_id));
        } else {
            ++it;
        }
    }
    return true;
}

bool CanBusController::unregisterRtSources() {
    if (!impl_) {
        LOG_W("Unregister RT sources failed, CAN bus controller not initialized");
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->rt_sources_mutex_);
    impl_->rt_sources_.clear();
    LOG_I("Unregistered all RT sources");
    return true;
}

Result<ErrorType> CanBusController::sendMessageRt(BusId bus_id, const CanMessageFrame& frame) {
    if (!impl_) {
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }
    if (bus_id == static_cast<BusId>(-1) || bus_id >= impl_->MAX_CANBUS_HANDLERS) {
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_INVALID_PARAMETER));
    }

    // 加锁保护 canbus_blocks_ 访问（与 sender 线程的 ring 消费/RV 扫描互斥），
    // 供外部/独立线程安全调用
    std::lock_guard<std::mutex> lock(impl_->blocks_mutex_);
    return sendMessageRtUnlocked(bus_id, frame);
}

Result<ErrorType> CanBusController::sendMessageRtUnlocked(BusId bus_id, const CanMessageFrame& frame) {
    // 不加锁的直发版本：仅在 sender 线程的 RT 源回调内使用（此时 senderThreadFunction
    // 已持有 blocks_mutex_ 保护 RT 扫描段）。调用方必须保证已持 blocks_mutex_。
    if (!impl_) {
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }
    if (bus_id == static_cast<BusId>(-1) || bus_id >= impl_->MAX_CANBUS_HANDLERS) {
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_INVALID_PARAMETER));
    }
    if (!impl_->canbus_blocks_[bus_id] || !impl_->canbus_blocks_[bus_id]->active()) {
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_BUS_NOT_ACTIVE));
    }
    impl_->canbus_blocks_[bus_id]->sendMessage(frame);
    return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
}

// Custom deleter implementation for Impl
void CanBusController::ImplDeleter::operator()(Impl* ptr) const {
    delete ptr;
}

} // namespace canbus_sdk