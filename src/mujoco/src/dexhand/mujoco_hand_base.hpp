#ifndef __MUJOCO_HAND_BASE_HPP__
#define __MUJOCO_HAND_BASE_HPP__
#include <atomic>
#include <memory>
#include <string>
#include <array>
#include <map>
#include <vector>
#include <algorithm>
#include <iostream>
#include <mujoco/mujoco.h>
#include "../joint_address.hpp"

namespace mujoco_node {
using FingerArray = std::array<int16_t, 6>;
using UnsignedFingerArray = std::array<uint16_t, 6>;
struct FingerStatus {
    UnsignedFingerArray positions;
    FingerArray speeds;
    FingerArray currents;
    UnsignedFingerArray states;

    FingerStatus() : positions{}, speeds{}, currents{}, states{} {}
    friend std::ostream& operator<<(std::ostream& os, const FingerStatus& status) {
        os << "Finger Positions: ";
        for (const auto& pos : status.positions) {
            os << pos << " ";
        }
        os << "\nFinger Speeds: ";
        for (const auto& speed : status.speeds) {
            os << speed << " ";
        }
        os << "\nFinger Currents: ";
        for (const auto& current : status.currents) {
            os << current << " ";
        }
        os << "\nFinger States: ";
        for (const auto& state : status.states) {
            os << state << " ";
        }
        return os;
    }
};
using DualHandsArray = std::array<FingerArray, 2>;
using UnsignedDualHandsArray = std::array<UnsignedFingerArray, 2>;
using FingerStatusPtr = std::shared_ptr<FingerStatus>;
using FingerStatusPtrArray = std::array<FingerStatusPtr, 2>;

class MujocoHandBase {
public:
    virtual ~MujocoHandBase() = default;

    /**
     * @brief read sensor data from mjData
     * @note callback for read thread.
     * @param d
     */
    virtual void readCallback(const mjData *d) = 0;

    /**
     * @brief write command to mjData
     * @note callback for write/control thread.
     * @param d
     */
    virtual void writeCallback(mjData *d) = 0;

    /**
     * @brief Set the Finger Positions.
     *
     * @param positions range: 0~max_range, 0 for open, max_range for close.
     */
    virtual void setFingerPositions(const UnsignedFingerArray &positions) = 0;

    /**
     * @brief Set the Finger Speeds object
     *
     * @param speeds range: -100~100
     * @note The fingers will move at the set speed values until they stall.
     *       The value range is -100 to 100. Positive values indicate flexion, negative values indicate extension.
     */
    virtual void setFingerSpeeds(const FingerArray &speeds) = 0;

    /**
     * @brief Get the Finger Status object
     *
     * @return FingerStatusPtr
     */
    virtual FingerStatusPtr getFingerStatus() = 0;
};

using MujocoHandBasePtr = std::shared_ptr<mujoco_node::MujocoHandBase>;

} // namespace mujoco_node

#endif