#pragma once

#include <mujoco/mujoco.h>

namespace mujoco_cpp {

inline constexpr char kDepthCameraName[] = "waist_camera";
inline constexpr char kDepthCameraFrameId[] = "waist_camera_link";
inline constexpr char kDepthImageTopic[] = "/waist_camera/depth/image_raw";
inline constexpr char kDepthImageArrayTopic[] = "depth_image";
inline constexpr char kDepthHistoryTopic[] = "/camera/depth/depth_history_array";

inline bool ModelHasNamedCamera(const mjModel* model, const char* camera_name) {
  return model != nullptr && camera_name != nullptr && camera_name[0] != '\0' &&
         mj_name2id(model, mjOBJ_CAMERA, camera_name) >= 0;
}

inline bool ModelHasTargetDepthCamera(const mjModel* model) {
  return ModelHasNamedCamera(model, kDepthCameraName);
}

}  // namespace mujoco_cpp
