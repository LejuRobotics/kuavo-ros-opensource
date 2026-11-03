#include "motorevo/rt_send_limiter.h"
#include <gtest/gtest.h>
using motorevo::RtSendLimiter;
TEST(RtSendLimiter, Bounds500HzInputTo250HzAndKeepsLatest) {
    RtSendLimiter limiter;
    std::atomic<bool> pending{false};
    const auto epoch = RtSendLimiter::Clock::time_point{};
    int sends = 0, latest = -1, transmitted = -1;
    for (int ms = 0; ms < 1000; ++ms) {
        if (ms % 2 == 0) { latest = ms; pending = true; }
        if (limiter.consume(pending, epoch + std::chrono::milliseconds(ms))) {
            ++sends; transmitted = latest; EXPECT_EQ(ms % 4, 0);
        }
    }
    EXPECT_EQ(sends, 250);
    EXPECT_EQ(transmitted, 996);
    EXPECT_TRUE(pending); // The 998 ms target must survive the throttle window.
    EXPECT_TRUE(limiter.consume(pending, epoch + std::chrono::milliseconds(1000)));
}
TEST(RtSendLimiter, UpdateDuringTransmissionSurvives) {
    RtSendLimiter limiter;
    std::atomic<bool> pending{true};
    const auto epoch = RtSendLimiter::Clock::time_point{};
    ASSERT_TRUE(limiter.consume(pending, epoch));
    pending = true; // Producer runs while caller encodes/sends the previous batch.
    EXPECT_FALSE(limiter.consume(pending, epoch + std::chrono::milliseconds(3)));
    EXPECT_TRUE(pending);
    EXPECT_TRUE(limiter.consume(pending, epoch + std::chrono::milliseconds(4)));
    EXPECT_FALSE(limiter.consume(pending, epoch + std::chrono::milliseconds(8)));
}
TEST(RtSendLimiter, IndependentBusesAndIdleInput) {
    RtSendLimiter left, right;
    std::atomic<bool> a{false}, b{true};
    const auto epoch = RtSendLimiter::Clock::time_point{};
    EXPECT_FALSE(left.consume(a, epoch));
    EXPECT_TRUE(right.consume(b, epoch));
    a = true;
    EXPECT_TRUE(left.consume(a, epoch + std::chrono::milliseconds(1)));
}
