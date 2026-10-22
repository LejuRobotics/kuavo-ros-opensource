#ifndef _STARK_DEXHAND_H_
#define _STARK_DEXHAND_H_
#include "dexhand_base.h"

struct DeviceHandler;
namespace dexhand {

class StarkDexhand : public DexHandBase {
public:
    StarkDexhand(const StarkDexhand&) = delete;
    StarkDexhand& operator=(const StarkDexhand&) = delete;
    virtual ~StarkDexhand();

    /**
     * @brief Auto-detect protocol and connect to device
     *
     * 1. stark_auto_detect(false, port, 0) to scan specified port
     * 2. init_from_detected() to initialize
     * 3. Fallback: protobuf_open(port, 10) for legacy firmware
     *
     * @param port Serial port path, e.g. "/dev/stark_serial_R"
     * @return Device instance, or nullptr on failure
     */
    static std::unique_ptr<StarkDexhand> Connect(const std::string& port);

    DexHandFwType getDexHandFwType() override;
    bool getDeviceInfo(DeviceInfo_t& info) override;
    void setFingerPositions(const UnsignedFingerArray &positions) override;
    void setFingerSpeeds(const FingerArray &speeds) override;
    bool getFingerStatus(FingerStatus& status) override;
    void setGripForce(GripForce level) override;
    GripForce getGripForce() override;
    void setTurboModeEnabled(bool enabled) override;
    bool isTurboModeEnabled() override;
    TurboConfig_t getTurboConfig() override;
    void runActionSequence(ActionSequenceId_t seq_id) override;
    bool setActionSequence(ActionSequenceId_t seq_id, const ActionSeqDataTypeVec &sequences) override;

private:
    explicit StarkDexhand(DeviceHandler* handle, uint8_t slave_id, uint8_t protocol, uint8_t hardware_type);

    DeviceHandler* handle_;
    uint8_t slave_id_;
    uint8_t protocol_;       // StarkProtocolType
    uint8_t hardware_type_;  // StarkHardwareType
};

} // namespace dexhand
#endif // _STARK_DEXHAND_H_
