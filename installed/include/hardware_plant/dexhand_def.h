#ifndef DEXHAND_DEF_H
#define DEXHAND_DEF_H

#include <stdint.h>
#include <string>
#include <vector>
#include <memory>
#include <array>
#include <mutex>
#include <iostream>

namespace eef_controller {

enum class HandSide: int {
    LEFT = 0,
    RIGHT = 1,
    BOTH = 2
};

struct ActionSequenceData{
    uint16_t duration_ms; // ms
    int8_t positions[6]; 
    int8_t speeds[6];
    int8_t forces[6];

    ActionSequenceData(): duration_ms(500) {
        for (int i = 0; i < 6; i++) {
            positions[i] = 0;
            speeds[i] = 0;
            forces[i] = 0;
        }
    }
};

struct GestureInfo {
    std::string gesture_name;               // 手势名称
    std::string description;                // 描述
    std::vector<std::string> alias;         // 别名列表
    std::vector<ActionSequenceData> action_sequences; // 手势序列的动作值
};
using GestureInfoPtr = std::shared_ptr<GestureInfo>;

struct GestureExecuteInfo {
    HandSide hand_side;
    std::string gesture_name;
};
using GestureExecuteInfoVec = std::vector<GestureExecuteInfo>;

} // namespace eef_controller
#endif