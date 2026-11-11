#include "lejuclaw_can_customed.h"
#include "lejuclaw_controller.h"
#include <gtest/gtest.h>
#include <tuple>

namespace lejuclaw_can {
struct LeJuClawCanTestAccess {
    static auto stamp(const LeJuClawCan& claw) {return claw.last_stream_command_time_;}
    static int timeout(const LeJuClawCan& claw) {return claw.cfg_STREAM_WATCHDOG_TIMEOUT_MS;}
    static bool armed(const LeJuClawCan& claw) {return claw.stream_watchdog_armed_;}
    static void add(LeJuClawCan& claw, uint8_t id) {
        motorevo::RevoMotorConfig_t config{};
        config.id=id; config.ignore=false;
        claw.motor_ctrl_datas_.emplace(std::piecewise_construct, std::forward_as_tuple(id),
            std::forward_as_tuple(false, motorevo::RevoMotor(0xff, id,
                motorevo::MotorMode::TorquePositionMixControlMode), config));
    }
    static void rx(LeJuClawCan& claw, uint8_t id, int dlc=8) {
        auto* f=canbus_sdk::createCanMessageFrame();
        f->ctrl.rx.dlc=dlc;
        const uint8_t payload[8]={id,0x80,0,0x80,0,0,0,25};
        std::memcpy(f->payload,payload,8);
        canbus_sdk::CallbackContext context{&claw};
        LeJuClawCan::internalMessageCallback(f,&context);
    }
    static void age(LeJuClawCan& claw, uint8_t id, int ms) {
        claw.motor_ctrl_datas_.at(id).feedback_health.received(FeedbackHealth::nowNs()-int64_t(ms)*1000000);
    }
    static void startIdle(LeJuClawCan& claw) {
        for(auto& entry:claw.motor_ctrl_datas_) {
            entry.second.operation=LeJuClawCan::Operation::ENABLE;
            entry.second.operation_status=LeJuClawCan::OperationStatus::SUCCESS;
            entry.second.feedback_received=true;
        }
        claw.init_status_variables();
        claw.thread_running=true;
        claw.control_thread_=std::thread(&LeJuClawCan::control_thread,&claw);
    }
    static void stop(LeJuClawCan& claw) {
        claw.thread_running=false;
        if(claw.control_thread_.joinable())claw.control_thread_.join();
    }
};
}
namespace eef_controller {
struct LejuClawControllerTestAccess {
    static void attach(LejuClawController& controller,lejuclaw_can::LeJuClawCan* driver) {
        controller.claw_can_ptr_=driver;
        controller.updateState({LejuClawController::State::kReached,LejuClawController::State::kReached});
    }
    static void detach(LejuClawController& controller) { controller.claw_can_ptr_=nullptr; }
};
}
using namespace lejuclaw_can;
TEST(FeedbackHealth, FirstRxTimeoutAndRecovery) {
    FeedbackHealth h;
    EXPECT_FALSE(h.snapshot(0).fresh);
    EXPECT_EQ(h.snapshot(0).age_ms,-1);
    h.received(0);
    EXPECT_TRUE(h.snapshot(199999999).fresh);
    EXPECT_FALSE(h.snapshot(200000000).fresh);
    h.received(300000000);
    EXPECT_TRUE(h.snapshot(300000001).fresh);
    EXPECT_EQ(h.snapshot(300000001).rx_count,2);
    EXPECT_TRUE(h.snapshot(299999999).fresh); // Concurrent new frame vs captured clock.
    h.transmitted(false);h.transmitted(true);h.tef();
    const auto s=h.snapshot(300000001);
    EXPECT_EQ(s.tx_attempts,2);EXPECT_EQ(s.tx_enqueue_failures,1);EXPECT_EQ(s.tef_count,1);
}
TEST(ClawFeedback, IdenticalFramesStillRefreshAndMalformedFramesDoNot) {
    LeJuClawCan claw;
    LeJuClawCanTestAccess::add(claw,17);
    LeJuClawCanTestAccess::rx(claw,17);
    LeJuClawCanTestAccess::age(claw,17,500);
    EXPECT_FALSE(claw.get_feedback_health().at(17).fresh);
    LeJuClawCanTestAccess::rx(claw,17); // Identical position, velocity, torque.
    EXPECT_TRUE(claw.get_feedback_health().at(17).fresh);
    const auto count=claw.get_feedback_health().at(17).rx_count;
    LeJuClawCanTestAccess::rx(claw,17,3);
    LeJuClawCanTestAccess::rx(claw,99);
    EXPECT_EQ(claw.get_feedback_health().at(17).rx_count,count);
}
TEST(ClawFeedback, OnlyStaleSideIsMarkedErrorAndItCanRecover) {
    LeJuClawCan claw;
    for(uint8_t id:{17,18}) {LeJuClawCanTestAccess::add(claw,id);LeJuClawCanTestAccess::rx(claw,id);}
    LeJuClawCanTestAccess::age(claw,17,250);
    auto controller=eef_controller::LejuClawController::Create(true);
    eef_controller::LejuClawControllerTestAccess::attach(*controller,&claw);
    const auto states=controller->get_state();
    EXPECT_EQ(states[0],eef_controller::LejuClawController::State::kError);
    EXPECT_EQ(states[1],eef_controller::LejuClawController::State::kReached);
    LeJuClawCanTestAccess::rx(claw,17);
    EXPECT_EQ(controller->get_state()[0],eef_controller::LejuClawController::State::kReached);
    eef_controller::LejuClawControllerTestAccess::detach(*controller);
}
TEST(ClawFeedback, IdleTargetsRefreshAtBoundedRateAndEnqueueFailuresAreCounted) {
    // Invalid bus 0xff; never open/register a device or initialize the CAN SDK.
    LeJuClawCan claw;
    LeJuClawCanTestAccess::add(claw,17);
    const auto start=std::chrono::steady_clock::now();
    LeJuClawCanTestAccess::startIdle(claw);
    std::this_thread::sleep_for(std::chrono::milliseconds(165));
    LeJuClawCanTestAccess::stop(claw);
    const auto health=claw.get_feedback_health().at(17);
    EXPECT_GE(health.tx_attempts,2);
    const auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now()-start).count();
    EXPECT_LE(health.tx_attempts,static_cast<uint64_t>(elapsed/50+1));
    EXPECT_EQ(health.tx_attempts,health.tx_enqueue_failures);
    EXPECT_FALSE(health.fresh);
}

TEST(ClawMailboxDeadline, ConsumingAnOldFrameCannotRenewTheLease) {
    // No CAN initialization, ports or motor writes.
    LeJuClawCan claw;
    auto arrival=std::chrono::steady_clock::now()-std::chrono::milliseconds(20);
    EXPECT_NE(claw.refresh_stream_watchdog(arrival),0u);
    EXPECT_EQ(LeJuClawCanTestAccess::stamp(claw),arrival);
    auto expired=std::chrono::steady_clock::now()-std::chrono::milliseconds(LeJuClawCanTestAccess::timeout(claw)+1);
    EXPECT_EQ(claw.refresh_stream_watchdog(expired),0u);
    EXPECT_EQ(LeJuClawCanTestAccess::stamp(claw),arrival);
    claw.disarm_stream_watchdog();
    EXPECT_EQ(claw.refresh_stream_watchdog(expired),0u);
    EXPECT_FALSE(LeJuClawCanTestAccess::armed(claw));
}
