#ifndef DHCLAW_CAN_H
#define DHCLAW_CAN_H

#include <cstdint>
#include <string>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>

// canbus_sdk (BMAPI / LEJU CANable)
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/canbus_sdk_def.h"

namespace dhclaw {

// ============================================================================
// DH 夹爪寄存器地址 (Modbus 协议 over CAN)
// ============================================================================
namespace Reg {
    constexpr uint16_t INIT         = 0x0100;
    constexpr uint16_t FORCE        = 0x0101;
    constexpr uint16_t POSITION     = 0x0103;
    constexpr uint16_t SPEED        = 0x0104;
    constexpr uint16_t INIT_STATUS  = 0x0200;
    constexpr uint16_t GRIP_STATUS  = 0x0201;
    constexpr uint16_t POS_FEEDBACK = 0x0202;
    constexpr uint16_t SAVE_FLASH   = 0x0300;
    constexpr uint16_t INIT_DIR     = 0x0301;
    constexpr uint16_t DEVICE_ID    = 0x0302;
    constexpr uint16_t BAUDRATE     = 0x0303;
}

enum class GripStatus : int { MOVING = 0, REACHED = 1, GRIPPED = 2, DROPPED = 3 };
enum class InitStatus : int { NOT_INIT = 0, INIT_DONE = 1 };

/**
 * @brief bus_type 取值（与 canbus_sdk::CanBusModelType 对齐）
 *  0 = BUSMUST_A
 *  1 = BUSMUST_B
 *  2 = LEJU_CAN_A   (canable / 左手 canbus0, /dev/canable1)
 *  3 = LEJU_CAN_B   (canable / 右手 canbus1, /dev/canable2)
 *
 * 与手臂电机共总线时请使用 LEJU_CAN_*，默认 1Mbps / 2Mbps，
 * 设备 ID 见 ~/.config/lejuconfig/canbus_device_cofig.yaml
 * （左夹爪常用 0x11=17，右夹爪常用 0x12=18）。
 */
enum class BusType : int {
    BUSMUST_A  = 0,
    BUSMUST_B  = 1,
    LEJU_CAN_A = 2,
    LEJU_CAN_B = 3,
};

// 与 canbus_device_cofig.yaml 中 Llejuclaw 默认一致
constexpr int kDefaultCanId     = 0x11;  // 17
constexpr int kDefaultNBitrate  = 1000;  // kbps, 与电机仲裁段一致
constexpr int kDefaultDBitrate  = 2000;  // kbps
constexpr int kDefaultBusType   = static_cast<int>(BusType::LEJU_CAN_A);

// ============================================================================
// DH 夹爪控制类 (canbus_sdk / LEJU CANable 共总线版本)
// ============================================================================
class DhClaw {
public:
    DhClaw() = default;
    ~DhClaw();

    // ---- 连接 ----
    /// @param bus_name   CAN 总线名称 (如 "canbus0")
    /// @param can_id     夹爪 CAN ID (默认 17 / 0x11)
    /// @param bus_type   见 BusType；默认 LEJU_CAN_A
    /// @param nbitrate   仲裁段波特率 kbps (默认 1000=1M，与手臂共总线)
    /// @param dbitrate   数据段波特率 kbps (默认 2000)
    /// @param close_bus_on_exit  true=close() 时关闭整条总线；
    ///                           false(默认)=仅注销设备，适合与电机共总线
    int  open(const std::string& bus_name,
              int can_id = kDefaultCanId,
              int bus_type = kDefaultBusType,
              int nbitrate = kDefaultNBitrate,
              int dbitrate = kDefaultDBitrate,
              bool close_bus_on_exit = false);

    /// 从 canbus_device_cofig.yaml 按设备名打开（默认 "Llejuclaw"）
    /// 读取 bus/type/bitrate/device_id，与电机共用同一 CanBusController
    int  openFromConfig(const std::string& device_name = "Llejuclaw",
                        const std::string& config_path = "");

    void close();
    bool isOpen() const;

    int  canId() const { return can_id_; }
    const std::string& busName() const { return bus_name_; }
    int  nbitrate() const { return nbitrate_; }
    int  busType() const { return static_cast<int>(model_); }

    // ---- 探测 ----
    /// @brief ping 指定 CAN ID: 读寄存器 0x0202, 有回复即在线
    bool ping(uint32_t can_id, int timeout_ms = 100);

    // ---- 初始化 ----
    bool calibrate();
    bool goHome();
    int  getInitStatus();

    // ---- 运动控制 ----
    /// @param position  0-100 (内部反转映射发给硬件: 0=全开→硬件1000, 100=全关→硬件0)
    bool setPosition(uint16_t position, uint8_t speed = 50, uint8_t force = 50);
    bool setForce(uint8_t force);
    bool setSpeed(uint8_t speed);

    // ---- 状态反馈 ----
    /// @return 当前位置 0-100 (硬件 0-1000 已反转映射, 0=全开, 100=全关)
    int      getGripStatus();
    uint16_t getPosition();
    /// Return false on unreadable/out-of-range feedback; leave output unchanged.
    bool getPosition(uint16_t& position);

    // ---- 低级 Modbus ----
    bool writeRegister(uint16_t addr, uint16_t value);
    bool readRegister(uint16_t addr, uint16_t& value);

    /// bus_type int → CanBusModelType
    static canbus_sdk::CanBusModelType toModelType(int bus_type);
    /// 根据总线名推断默认 LEJU 类型：canbus0→A, canbus1→B
    static int defaultBusTypeFor(const std::string& bus_name);

private:
    bool sendAndWait(uint32_t can_id, const uint8_t* tx, int timeout_ms,
                     uint8_t* rx_payload, uint8_t& rx_dlc);
    static void msgCallback(canbus_sdk::CanMessageFrame* frame,
                            const canbus_sdk::CallbackContext* ctx);

    // 同步等待结构
    struct SyncCtx {
        uint8_t       payload[64];
        uint8_t       dlc = 0;
        uint32_t      id   = 0;
        std::atomic<uint32_t> expected_id{0};
        std::mutex    mtx;
        std::condition_variable cv;
        std::atomic<bool>      received{false};
    };

    canbus_sdk::CanBusModelType model_ = canbus_sdk::CanBusModelType::LEJU_CAN_A;
    canbus_sdk::BusId   bus_id_ = 0;
    int    can_id_     = kDefaultCanId;
    int    nbitrate_   = kDefaultNBitrate;
    int    dbitrate_   = kDefaultDBitrate;
    bool   opened_     = false;
    bool   close_bus_on_exit_ = false;  // 共总线时默认不关总线
    bool   opened_bus_ = false;         // 本次 open 是否新打开了总线
    std::string bus_name_;
    std::string device_name_ = "dhclaw";
    std::mutex io_mutex_;   // 保护 sendAndWait, 防止多线程并发

    SyncCtx sync_;
    canbus_sdk::CallbackContext cb_ctx_;
    canbus_sdk::CallbackContext tef_ctx_;
};

} // namespace dhclaw

#endif // DHCLAW_CAN_H
