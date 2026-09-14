#ifndef CANBUS_SDK_CONFIG_PARSER_H
#define CANBUS_SDK_CONFIG_PARSER_H

#include <string>
#include <vector>
#include <map>
#include "canbus_sdk_def.h"

namespace canbus_sdk {

/**
 * @brief CAN总线接口配置
 */
struct CanBusConfig {
    std::string name;                  // CAN总线名称 (如 canbus0, canbus1)
    CanBusModelType type;              // 接口类型
    std::string device_name;           // 设备名称
    CanBusBitrate bitrate;            // 波特率配置
    bool enable;                       // 是否启用（默认true）
};

/**
 * @brief 设备配置
 */
struct DeviceConfig {
    std::string name;                  // 设备名称
    DeviceType device_type;            // 设备类型
    DeviceId device_id;                // 设备ID
    std::string canbus_name;           // 所属CAN总线接口名称
    bool ignore;                       // 是否忽略/禁用（默认false）

    // 电机专用配置
    bool negtive;                      // 是否负方向（仅电机使用）
    std::vector<float> motor_params;   // 电机参数（仅电机使用）
    float ratio;                       // 减速比（仅电机使用）
};

/**
 * @brief CAN总线配置解析器
 *
 * 负责解析YAML格式的配置文件，提取CAN总线接口和设备配置信息
 */
class ConfigParser {
public:
    ConfigParser() = default;
    ~ConfigParser() = default;

    // 禁用拷贝构造和赋值
    ConfigParser(const ConfigParser&) = delete;
    ConfigParser& operator=(const ConfigParser&) = delete;

    /**
     * @brief 从YAML文件解析配置
     *
     * @param config_path 配置文件路径
     * @return bool 解析结果（true成功，false失败）
     * @throws std::runtime_error 配置文件格式错误或缺少必需字段
     * @throws std::runtime_error CAN总线接口配置无效（名称重复、缺少必需字段等）
     * @throws std::runtime_error 设备配置无效（名称重复、ID冲突、缺少必需字段等）
     * @throws std::bad_alloc 内存分配失败
     */
    bool parseFromFile(const std::string& config_path) noexcept(false);

    /**
     * @brief 获取CAN总线配置列表
     *
     * @return const std::vector<CanBusConfig>& CAN总线配置列表
     */
    const std::vector<CanBusConfig>& getCanBusConfigs() const;

    /**
     * @brief 获取CAN总线上的设备列表
     *
     * @param canbus_name CAN总线名称
     * @return std::vector<DeviceConfig> 该CAN总线上的设备列表
     */
    std::vector<DeviceConfig> getDevices(const std::string& canbus_name) const;

    /**
     * @brief 获取CAN总线上指定类型的设备列表
     *
     * @param canbus_name CAN总线名称
     * @param device_type 设备类型
     * @return std::vector<DeviceConfig> 符合条件的设备列表
     */
    std::vector<DeviceConfig> getDevices(const std::string& canbus_name, DeviceType device_type) const;

    /**
     * @brief 清空配置
     */
    void clear();

    /**
     * @brief 打印配置信息
     */
    void printConfig() const;

    /**
     * @brief 获取默认配置文件路径
     * @return 默认配置文件路径 $HOME/.config/lejuconfig/canbus_device_cofig.yaml
     */
    static std::string getDefaultConfigFilePath();

    /**
     * @brief 将配置写入YAML文件
     *
     * @param config_path 配置文件路径
     * @return bool 写入结果（true成功，false失败）
     * @throws std::runtime_error 文件写入权限不足或磁盘空间不足
     */
    bool writeToFile(const std::string& config_path) const;

    /**
     * @brief 更新设备的ignore状态
     *
     * @param device_name 设备名称
     * @param ignore 是否忽略该设备
     * @return bool 更新结果（true成功，false失败-设备不存在）
     */
    bool updateDeviceIgnore(const std::string& device_name, bool ignore);

    /**
     * @brief 设置电机的negtive字段
     *
     * @param device_name 设备名称
     * @param negtive 是否负方向
     * @return bool 设置结果（true成功，false失败-设备不存在或不是电机）
     */
    bool setMotorNegative(const std::string& device_name, bool negtive);

private:
    std::vector<CanBusConfig> canbus_configs_;
    std::map<std::string, std::vector<DeviceConfig>> canbus_to_devices_;
};

} // namespace canbus_sdk

#endif // CANBUS_SDK_CONFIG_PARSER_H