#ifndef REVO2_HAND_CAN_CUSTOMED_H
#define REVO2_HAND_CAN_CUSTOMED_H

#include <mutex>
#include <queue>
#include <condition_variable>
#include "dexhand_base.h"
#include "modbus.h"
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"

namespace dexhand {

/**
 * @brief 基于CAN通信的  [Revo2灵巧手]
 *
 */
class Revo2CanDexhand: public ModbusDexhand {
public:
    ~Revo2CanDexhand();

    /**
     * @brief 获取灵巧手固件类型  => 固定返回 DexHandFwType::V2_STANDARD
     * @return 固件类型
     */
    DexHandFwType getDexHandFwType() override final;

    /**
     * @brief 连接并创建Revo2灵巧手设备实例
     *
     * 根据设备配置信息，初始化CAN总线连接，注册设备到消息路由系统，
     * 并创建设备句柄。此方法会自动设置CAN回调函数和消息队列。
     *
     * @param config 设备配置信息，包含设备名称、ID、CAN总线名称等
     * @return 成功返回设备实例指针，失败返回nullptr
     */
    static std::unique_ptr<Revo2CanDexhand> Connect(
        const canbus_sdk::DeviceConfig& config
    );

    /**
     * @brief 设置灵巧手手指位置，范围0~100，0为张开，100为闭合
     * 
     * @param positions 
     */
    void setFingerPositions(const UnsignedFingerArray &positions) override final;

    /**
     * @brief 设置手指位置和持续时间
     *
     * 设置每个手指的目标位置和运动持续时间。位置值会被限制在0~100范围，
     * 持续时间会被限制在1~2000毫秒范围内。
     *
     * @param positions 手指位置数组，范围0~100，0为张开，100为闭合
     * @param durations 每个手指的运动持续时间，范围1~2000毫秒
     */
    void setFingerPositionsAndDurations(const UnsignedFingerArray &positions, const UnsignedFingerArray &durations);

    /**
     * @brief 设置抓力等级
     *
     * @warning Revo2灵巧手不支持设置抓力等级，此函数仅打印警告信息。
     *
     * @param level 抓力等级
     */
    void setGripForce(GripForce level) override final;

    /**
     * @brief 获取抓力等级
     *
     * @warning Revo2灵巧手不支持获取抓力等级，此函数仅打印警告信息并返回默认值。
     *
     * @return 默认抓力等级
     */
    GripForce getGripForce() override final;

    bool usesRevo2MotorApi() const override { return true; }

private:
    /**
     * @brief 私有构造函数，用于Connect方法创建实例
     * @param handle 设备句柄
     * @param slave_id_ 从机地址
     * @param config 设备配置信息
     */
    explicit Revo2CanDexhand(DeviceHandler* handle, uint8_t slave_id_, const canbus_sdk::DeviceConfig& config);

    canbus_sdk::DeviceConfig config_;  ///< 设备配置信息
};

}

#endif // REVO2_HAND_CAN_CUSTOMED_H