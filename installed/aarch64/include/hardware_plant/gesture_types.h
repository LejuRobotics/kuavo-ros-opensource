#ifndef GESTURE_TYPES_H_
#define GESTURE_TYPES_H_

#include <string>
#include <vector>

namespace eef_controller {

struct GestureInfoMsg {
    std::string gesture_name;
    std::vector<std::string> alias;
    std::string description;
};

struct GestureListResponse {
    bool success;
    int32_t gesture_count;
    std::string message;
    std::vector<GestureInfoMsg> gesture_infos;
};

} // namespace eef_controller

#endif 