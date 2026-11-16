#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>

namespace lejuclaw_can {
struct FeedbackSnapshot {
    uint64_t rx_count{0}, tx_attempts{0}, tx_enqueue_failures{0}, tef_count{0};
    int64_t age_ms{-1};
    bool fresh{false};
};

// RX age is based on arrival of valid frames, never on changes in their values.
class FeedbackHealth {
public:
    static constexpr int64_t kTimeoutMs = 200;
    static int64_t nowNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void received(int64_t now = nowNs()) {
        rx_count_.fetch_add(1, std::memory_order_relaxed);
        last_rx_ns_.store(now, std::memory_order_release);
    }
    void transmitted(bool accepted) {
        tx_attempts_.fetch_add(1, std::memory_order_relaxed);
        if (!accepted) tx_enqueue_failures_.fetch_add(1, std::memory_order_relaxed);
    }
    void tef() { tef_count_.fetch_add(1, std::memory_order_relaxed); }
    FeedbackSnapshot snapshot(int64_t now = nowNs()) const {
        FeedbackSnapshot s;
        const auto last = last_rx_ns_.load(std::memory_order_acquire);
        s.rx_count = rx_count_.load(std::memory_order_relaxed);
        s.tx_attempts = tx_attempts_.load(std::memory_order_relaxed);
        s.tx_enqueue_failures = tx_enqueue_failures_.load(std::memory_order_relaxed);
        s.tef_count = tef_count_.load(std::memory_order_relaxed);
        // A newer RX racing with a previously captured 'now' has age zero.
        s.age_ms = last < 0 ? -1 : (now > last ? (now - last) / 1000000 : 0);
        s.fresh = last >= 0 && (now <= last || now - last < kTimeoutMs * 1000000);
        return s;
    }
private:
    std::atomic<int64_t> last_rx_ns_{-1};
    std::atomic<uint64_t> rx_count_{0}, tx_attempts_{0}, tx_enqueue_failures_{0}, tef_count_{0};
};
}  // namespace lejuclaw_can
