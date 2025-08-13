#ifndef _DEXHAND_BASE_H_
#define _DEXHAND_BASE_H_
#include <array>
#include <iostream>
#include <vector>
#include <memory>

namespace dexhand {

enum DexHandFwType : uint8_t {
    RS485_PROTOBUF = 0,
    V1_STANDARD = 1,
    V1_TOUCH = 2,
    V2_STANDARD = 3,
};

enum DexHandType : uint8_t {
  SKU_TYPE_MEDIUM_RIGHT = 1,
  SKU_TYPE_MEDIUM_LEFT = 2,
  SKU_TYPE_SMALL_RIGHT = 3,
  SKU_TYPE_SMALL_LEFT = 4,
  SKU_TYPE_NONE = 110,  
};

enum GripForce : uint8_t {
  FORCE_LEVEL_SMALL = 1,
  FORCE_LEVEL_NORMAL = 2,
  FORCE_LEVEL_FULL = 3,
};

/// 内置手势1~6：张开、握拳、两只捏、三只捏、侧边捏、单指点
/// 自定义手势6个: 10~15
enum ActionSequenceId_t : uint8_t {
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_OPEN = 1,
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_FIST = 2,
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_TWO = 3,
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_THREE = 4,
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_SIDE = 5,
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_POINT = 6,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE1 = 10,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE2 = 11,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE3 = 12,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE4 = 13,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE5 = 14,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE6 = 15,
};

struct TouchSensorStatus_t {
  uint16_t normal_force1;
  uint16_t normal_force2;
  uint16_t normal_force3;
  uint16_t tangential_force1;
  uint16_t tangential_force2;
  uint16_t tangential_force3;
  uint16_t tangential_direction1;
  uint16_t tangential_direction2;
  uint16_t tangential_direction3;
  uint32_t self_proximity1;
  uint32_t self_proximity2;
  uint32_t mutual_proximity;
  uint16_t status;
};

struct TurboConfig_t {
  uint16_t interval;
  uint16_t duration;
};

using FingerArray = std::array<int16_t, 6>;
using UnsignedFingerArray = std::array<uint16_t, 6>;
using TouchSensorStatusArray = std::array<TouchSensorStatus_t, 5>;
using FingerTouchStatusPtr = std::shared_ptr<TouchSensorStatusArray>;

struct ActionSeqDataType {
    uint16_t duration_ms; // ms
    UnsignedFingerArray positions;
    UnsignedFingerArray speeds;
    UnsignedFingerArray forces;

    ActionSeqDataType(): duration_ms(500) {
        for (int i = 0; i < 6; i++) {
            positions[i] = 0;
            speeds[i] = 0;
            forces[i] = 0;
        }
    }
};
using ActionSeqDataTypeVec = std::vector<ActionSeqDataType>;

/* open finger positions */
const UnsignedFingerArray kOpenFingerPositions = {
    0, 0, 0, 0, 0, 0
};
/* close finger positions */
const UnsignedFingerArray kCloseFingerPositions = {
    50, 50, 50, 50, 50, 50
};

struct DeviceInfo_t {
    DexHandType sku_type;
    std::string serial_number;
    std::string firmware_version;

    friend std::ostream& operator<<(std::ostream& os, const DeviceInfo_t& info) {
        os << "Sku Type: " << static_cast<int>(info.sku_type) << "\nSerial Number: " << info.serial_number << "\nFirmware Version: " << info.firmware_version << "\n";
        return os;
    }
};

struct FingerStatus {
    UnsignedFingerArray positions;
    FingerArray speeds;
    FingerArray currents;
    UnsignedFingerArray states;

    FingerStatus() {
        positions.fill(0);
        speeds.fill(0);
        currents.fill(0);
        states.fill(0);
    }
    friend std::ostream& operator<<(std::ostream& os, const FingerStatus& status) {
        os << "Finger Positions: ";
        for (const auto& pos : status.positions) {
            os << pos << " ";
        }
        os << "\nFinger Speeds: ";
        for (const auto& speed : status.speeds) {
            os << speed << " ";
        }
        os << "\nFinger Currents: ";
        for (const auto& current : status.currents) {
            os << current << " ";
        }
        os << "\nFinger States: ";
        for (const auto& state : status.states) {
            os << state << " ";
        }
        return os;
    }
};
using FingerStatusPtr = std::shared_ptr<FingerStatus>;


class DexHandBase {
public:
    explicit DexHandBase() = default;
    DexHandBase(const DexHandBase&) = delete;
    DexHandBase& operator=(const DexHandBase&) = delete;
    virtual ~DexHandBase() = default;

/* Interfaces */
public:
    /**
     * @brief Get the DexHandFwType object
     * 
     * @return DexHandFwType 
     */
    virtual DexHandFwType getDexHandFwType() = 0;

    /**
     * @brief Get the Device Info object
     * 
     * @return DeviceInfo 
     */
    virtual DeviceInfo_t getDeviceInfo() = 0;

    /**
     * @brief Set the Finger Positions.
     * 
     * @param positions range: 0~100, 0 for open, 100 for close.
     */
    virtual void setFingerPositions(const UnsignedFingerArray &positions) = 0;
    
    /**
     * @brief Set the Finger Speeds object
     * 
     * @param speeds range: -100~100
     * @note The fingers will move at the set speed values until they stall. 
     *       The value range is -100 to 100. Positive values indicate flexion, negative values indicate extension.
     */
    virtual void setFingerSpeeds(const FingerArray &speeds) = 0;

    /**
     * @brief Get the Finger Status object
     * 
     * @return FingerStatusPtr 
     */
    virtual FingerStatusPtr getFingerStatus() = 0;

    /**
     * @brief Set the Force Level object
     * 
     * @param level {GripForce::NORMAL, GripForce::SMALL, GripForce::FULL}
     */
    virtual void setGripForce(GripForce level) = 0;

    /**
     * @brief Get the Force Level object
     * 
     * @return GripForce 
     */
    virtual GripForce  getGripForce() = 0;

    // /**
    //  * @brief Get the Touch Status object
    //  * 
    //  * @return FingerTouchStatusPtr 
    //  */
    // FingerTouchStatusPtr getTouchStatus();

    /**
     * @brief Set the Turbo Mode Enabled object
     * @note 开启之后会持续握紧, 掉电后，Turbo 模式会恢复到默认关闭状态。
     * @param enabled 
     */
    virtual void setTurboModeEnabled(bool enabled) = 0;

    /**
     * @brief Check if Turbo Mode is enabled
     * 
     * @return true if Turbo Mode is enabled
     * @return false if Turbo Mode is not enabled
     */
    virtual bool isTurboModeEnabled() = 0;

    virtual TurboConfig_t getTurboConfig() = 0;

    /**
     * @brief 运行动作序列
     * 
     * @param seq_id 
     */
    virtual void runActionSequence(ActionSequenceId_t seq_id) = 0;

    /**
     * @brief 设置动作序列
     * 
     * @param seq_id 
     * @param sequences
     * @return true if success
     * @return false if failed
     */
    virtual bool setActionSequence(ActionSequenceId_t seq_id, const ActionSeqDataTypeVec &sequences) = 0;
};
} // namespace dexhand

#endif // _DEXHAND_BASE_H_
