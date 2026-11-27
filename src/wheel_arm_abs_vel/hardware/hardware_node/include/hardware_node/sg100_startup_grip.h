#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <kuavo_msgs/SG100HandCommand.h>
#include <kuavo_msgs/SG100HandState.h>
#include <chrono>
#include <cmath>
#include <functional>
#include <vector>

namespace sg100 {

// Run once after the arm reaches the prep pose, matching the standard wheel entry.
// A private callback queue keeps feedback flowing while hardware startup waits.
inline bool runStartupGrip(ros::NodeHandle nh, const std::function<bool()>& stopped) {
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    kuavo_msgs::SG100HandState::ConstPtr state;
    using Clock = std::chrono::steady_clock;
    Clock::time_point received;
    auto feedback = nh.subscribe<kuavo_msgs::SG100HandState>("/sg100_hand_state", 1,
        [&](const kuavo_msgs::SG100HandState::ConstPtr& msg) {
            state = msg;
            received = Clock::now();
        });
    auto publisher = nh.advertise<kuavo_msgs::SG100HandCommand>("/sg100_hand_command", 1);
    const auto ready = [&] {
        if (!state || std::chrono::duration<double>(Clock::now() - received).count() >= 0.5 ||
            !state->left_hand_connected || !state->right_hand_connected ||
            !state->left_feedback_valid || !state->right_feedback_valid ||
            state->left_error_code || state->right_error_code ||
            state->left_hand_positions.size() != 11 ||
            state->right_hand_positions.size() != 11 || !publisher.getNumSubscribers()) {
            return false;
        }
        for (unsigned i = 0; i < 11; ++i) {
            if (!std::isfinite(state->left_hand_positions[i]) ||
                !std::isfinite(state->right_hand_positions[i])) return false;
        }
        return true;
    };
    // Startup commands need a heartbeat: the driver expires ordinary commands
    // after 0.5 seconds, while each phase must be observed for at least 1 second.
    const auto waitForPose = [&](const std::vector<float>& target,
                                 const std::function<void()>& send) {
        const auto start = Clock::now();
        while (ros::ok() && !stopped() &&
               std::chrono::duration<double>(Clock::now() - start).count() < 15.0) {
            queue.callAvailable(ros::WallDuration(0.02));
            if (!ros::ok() || stopped()) return false;
            if (!ready()) {
                if (!target.empty()) return false;
                continue;
            }
            if (target.empty()) return true;
            send();
            bool reached = std::chrono::duration<double>(Clock::now() - start).count() >= 1.0;
            for (unsigned i = 0; i < 11; ++i) {
                reached &= std::abs(state->left_hand_positions[i] - target[i]) <= 0.10f &&
                           std::abs(state->right_hand_positions[i] - target[i]) <= 0.10f;
            }
            if (reached) return true;
            ros::WallDuration(0.02).sleep();
        }
        return false;
    };
    if (!waitForPose({}, [] {})) {
        ROS_WARN("SG100 startup grip stopped: both hands are not ready or shutdown requested");
        return false;
    }

    kuavo_msgs::SG100HandCommand cmd;
    cmd.control_mode = kuavo_msgs::SG100HandCommand::MODE_JOINT_POSITION;
    // Keep startup separate from VR (1), services (2) and scripts (3).
    // Existing driver arbitration prevents either source from overwriting the other.
    cmd.source_id = 4;
    cmd.left_enable_mask = cmd.right_enable_mask = 0x07ff;
    const std::vector<float> initial{0, 0, 0, 0, 0, 1.37f, 0, 1.37f, 0, 0, 1.37f};
    const std::vector<std::vector<float>> poses{
        initial,
        {1.0f, -0.7f, 1.0f, 0, 1.37f, 1.37f, 1.37f, 1.37f, 0, 1.37f, 1.37f},
        initial};
    // Use the verified startup gains even when the glove uses lower driver defaults.
    constexpr float startup_kp = 10.0f;
    constexpr float startup_kd = 0.2f;
    cmd.left_hand_kp.assign(initial.size(), startup_kp);
    cmd.right_hand_kp = cmd.left_hand_kp;
    cmd.left_hand_kd.assign(initial.size(), startup_kd);
    cmd.right_hand_kd = cmd.left_hand_kd;
    cmd.left_hand_positions = cmd.right_hand_positions = initial;
    const auto release = [&] {
        cmd.header.stamp = ros::Time::now();
        cmd.control_mode = kuavo_msgs::SG100HandCommand::MODE_NULL;
        cmd.left_enable_mask = cmd.right_enable_mask = 0;
        publisher.publish(cmd);
        ros::WallDuration(0.05).sleep();
    };
    for (unsigned step = 0; step < poses.size(); ++step) {
        if (!ros::ok() || stopped()) { release(); return false; }
        cmd.left_hand_positions = cmd.right_hand_positions = poses[step];
        ROS_INFO("SG100 startup grip: %s", step == 1 ? "grasp" : "initialize");
        if (!waitForPose(poses[step], [&] {
                cmd.header.stamp = ros::Time::now();
                publisher.publish(cmd);
            })) {
            release();
            ROS_WARN("SG100 startup grip stopped: feedback lost, pose timed out or shutdown requested");
            return false;
        }
    }
    if (!ros::ok() || stopped()) { release(); return false; }
    // Release this source explicitly so glove/VR commands can take over.
    release();
    ROS_INFO("SG100 startup grip complete");
    return true;
}

}  // namespace sg100
