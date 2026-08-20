#ifndef EC_FIRMWARE_PARAM_CHECK_H
#define EC_FIRMWARE_PARAM_CHECK_H

#include <cstdint>
#include <string>
#include <vector>
#include <functional>

namespace highlydynamic {

/**
 * @brief EC 驱动器固件与参数校验模块。
 *
 * 在 HWPlantInit 中、EC master cyclic 起来之后、电机使能之前调用 run()，
 * 复用 live master 的 SDO upload 读取每个 youda 驱动器的固件版本与全量参数，
 * 与标准定义 JSON（拷贝自 ec_master_tools/config/sdo_config）逐字段比对。
 *
 * 严格模式下任何 mismatch（含固件版本不符）即返回 false，由调用方 exit 拉起 launch 退出。
 * 仅覆盖 youda(YD100/YD300) 驱动器且有标准 JSON 的机型；其它情况打 WARN 跳过。
 */
class EcFirmwareParamCheck
{
public:
    /**
     * @brief 执行固件与参数校验。
     *
     * @param robot_version_int  机器人版本整数（rb_version_.version_number()，如 53/54/17/42...）
     * @param ecmaster_type      EC 驱动器类型字符串（"youda"/"youda3"/"elmo"/"leju"/"lunbi"）
     * @param allow_mismatch     true=有 mismatch 只 WARN 不阻断；false=严格模式，mismatch 返回 false
     * @return true  校验通过或被跳过（非 youda / 无标准 JSON）
     * @return false 严格模式下发现 mismatch
     */
    static bool run(uint32_t robot_version_int,
                    const std::string &ecmaster_type,
                    bool allow_mismatch);
};

} // namespace highlydynamic

#endif // EC_FIRMWARE_PARAM_CHECK_H
