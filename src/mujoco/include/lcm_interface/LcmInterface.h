
#pragma once
#include <iostream>
#include <atomic>
#include <lcm/lcm-cpp.hpp>

#include "lcm_msg/low_state_t.hpp"
#include "lcm_msg/low_cmd_t.hpp"
#include "lcm_msg/full_state_t.hpp"
#include <pthread.h>

#include <mujoco/mujoco.h>
#include <chrono>


class MujocoLcm{

private:
    lcm::LCM lcm_;

    low_cmd_t recvCmd_;
    low_state_t sendState_;
    full_state_t sendFullState_;

    uint64_t last_timestamp = 0;
    bool new_msg =false;

    pthread_mutex_t recvMutex_;
    pthread_mutex_t sendMutex_;

    pthread_t lcmThread_;
    // The worker below blocks inside lcm_.handle(); this flag lets the owner
    // stop it and join before ~MujocoLcm() tears the LCM instance down.
    std::atomic<bool> lcmThreadRunning_{false};
    bool lcmThreadStarted_ = false;
public:
    MujocoLcm(/* args */);
    ~MujocoLcm();

    void HandleLowCmd(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const low_cmd_t* msg);
    void GetRecv(low_cmd_t& cmd);
    void SetSend(const mjData * d);
    void Send();

    static void* lcmThreadFunc(void* handler) {
        MujocoLcm* handlerObject = static_cast<MujocoLcm*>(handler);
        // Bounded wait: the thread must observe lcmThreadRunning_ going false
        // even when no LCM traffic ever arrives on the LOWCMD channel.
        while (handlerObject->lcmThreadRunning_.load()) {
            handlerObject->lcm_.handleTimeout(100);
        }
        return NULL;
    }

    void startLCMThread() {
        lcmThreadRunning_.store(true);
        if (pthread_create(&lcmThread_, NULL, &MujocoLcm::lcmThreadFunc, this) == 0) {
            lcmThreadStarted_ = true;
        } else {
            lcmThreadRunning_.store(false);
            std::cerr << "LCM thread creation failed" << std::endl;
        }
    }

    void joinLCMThread() {
        if (!lcmThreadStarted_) {
            return;
        }
        lcmThreadRunning_.store(false);
        pthread_join(lcmThread_, NULL);
        lcmThreadStarted_ = false;
    }
};
