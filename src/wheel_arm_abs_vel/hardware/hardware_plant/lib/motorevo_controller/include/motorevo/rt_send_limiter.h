#pragma once
#include <atomic>
#include <chrono>

namespace motorevo {
// Single CAN sender thread owns the clock; command producers only set pending.
class RtSendLimiter {
public:
    using Clock = std::chrono::steady_clock;
    bool consume(std::atomic<bool>& pending, Clock::time_point now = Clock::now()) {
        if (!pending.load(std::memory_order_acquire)) return false;
        if (sent_ && now - last_send_ < std::chrono::milliseconds(4)) return false;
        // Consume before encoding: a producer arriving during TX stays pending.
        if (!pending.exchange(false, std::memory_order_acq_rel)) return false;
        last_send_ = now;
        sent_ = true;
        return true;
    }
private:
    Clock::time_point last_send_{};
    bool sent_{false};
};
}  // namespace motorevo
