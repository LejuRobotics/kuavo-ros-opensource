#include <mutex>
#include "dexhand_base.h"
#include "modbus.h"

namespace dexhand {
class TouchDexhand: public ModbusDexhand {
public:
    TouchDexhand(const std::string& port, uint8_t slave_id);
    ~TouchDexhand();
    DexHandFwType getDexHandFwType() override;
        /**
     * @brief   Connect to the device
     * 
     * @param port 串口名称，例如："/dev/ttyUSB0"
     * @param slave_id 设备ID，默认为1，范围为1~255, 0 为广播地址
     * @param baudrate  波特率，115200, 57600, 19200, 460800
     * @return std::unique_ptr<TouchDexhand> 失败返回 nullptr
     */
    static std::unique_ptr<TouchDexhand> Connect(
        const std::string& port,
        uint8_t slave_id,
        uint32_t baudrate
    );
    //

    /**
     * @brief Get the Touch Status object
     * 
     * @return FingerTouchStatusPtr 
     */
    FingerTouchStatusPtr getTouchStatus();

    /**
     * @brief 重置触觉传感器采集通道
     * @note 在执行该指令时，手指传感器尽量不要受力, 0b00000001 表示重置第一个传感器
     * 
     * @param bits 
     */
    void resetTouchSensor(uint8_t bits = 0xFF);

    /**
     * @brief 启用触觉传感器
     * @note 0b00000001 表示启用第一个传感器
     * 
     * @param bits 
     */
    void enableTouchSensor(uint8_t bits = 0xFF);
private:
    explicit TouchDexhand(ModbusHandle* handle, uint8_t slave_id_);
};
} // namespace dexhand
