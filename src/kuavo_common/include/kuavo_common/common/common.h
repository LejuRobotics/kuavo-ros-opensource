#pragma once
#include <string>
#include <cstdint>

/// @brief 机器人版本号类
/// @details 机器人版本号类，用于表示机器人版本号
/// @note 机器人版本号格式为：
/// - MMMM 表示次版本号, 范围为0-9999
/// - N 表示修订号, 范围为0-9
/// - PPP 表示主版本号, 范围为0-9999
/// - BIGNUMBER: PPPPMMMMN
/// - STRING: PPPPMMMMN  Patch 为 0 时，不显示 Patch 部分, 比如 M(4), N(5),P(0) 表示为 45 而非 000000045 (0000,0004,5)
class RobotVersion
{
private:
    uint16_t Major = 0;
    uint16_t Minor = 0;
    uint16_t Patch = 0;
public:    
    RobotVersion() : Major(0), Minor(0), Patch(0) {}
    RobotVersion(const RobotVersion &) = default;
    RobotVersion &operator=(const RobotVersion &) = default;

    ///////////////////////////////////////////////////////////////////////////////////
    
    /// @brief 从版本号数字创建 RobotVersion 对象
    /// @param major 主版本号 (0-9999)
    /// @param minor 次版本号 (0-9)
    /// @param patch 补丁版本号 (0-9999), 默认为0
    /// @return RobotVersion 实例
    /// @throws std::invalid_argument 如果版本号超出有效范围
    RobotVersion(uint16_t major, uint16_t minor, uint16_t patch = 0);

    /// @brief 从大整数创建 RobotVersion 对象
    /// @param big_number 大整数
    /// @return RobotVersion 实例
    /// @throws std::invalid_argument 如果版本号超出有效范围
    static RobotVersion create(int big_number);

    /// @brief 判断版本号是否合法
    /// @param big_number 版本号数字
    /// @return 是否合法
    static bool is_valid(int big_number);

    /// @brief 将版本号转换为字符串
    /// @return 版本号字符串 如 45, 100045, 4000045, 100000049
    std::string to_string() const;

    /// @brief 判断版本号是 major 版本系列
    /// @param major 主版本号
    /// @return 是否以 major 开头
    /// @note 例如：45, 100045, 4000045, 100000049 都是属于 4 代系列
    bool start_with(uint16_t major) const;

    /// @brief 判断版本号是 major.minor 版本系列
    /// @param major 主版本号
    /// @param minor 次版本号
    /// @return 是否以 major.minor 开头
    /// @note 例如：45, 100045, 4000045 都是属于 45 版本系列
    bool start_with(uint16_t major, uint16_t minor) const;


    /// @brief 获取版本号对应的 PPPPMMMMN 数字
    /// @return PPPPMMMMN 数字 
    /// @note 例如：45 --> 45
    ///            100045 --> 00045.1 --> 45.1 --> 4.5.1
    ///            2000105 --> 00105.20 --> 105.20 --> 10.5.20
    uint32_t version_number() const;

    /// @brief 获取版本号标准的 semantic version 字符串
    /// @return 版本号字符串 如 4.5.0, 10.5.20
    /// @note 例如：45 --> 4.5.0
    ///            100045 --> 4.5.1
    ///            2000105 --> 10.5.20
    /// @ref https://semver.org/lang/zh-CN/
    std::string version_name() const;

    uint16_t major() const;
    uint16_t minor() const;
    uint16_t patch() const;
    bool operator==(const RobotVersion &other) const;
    bool operator!=(const RobotVersion &other) const;
    bool operator<(const RobotVersion &other) const;
    friend std::ostream &operator<<(std::ostream &os, const RobotVersion &version);
};