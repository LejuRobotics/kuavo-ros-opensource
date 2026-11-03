#ifndef LINKER_HAND_MAPPING_H
#define LINKER_HAND_MAPPING_H

#include <cstdint>
#include "dexhand_base.h"
#include "touch_hand_controller.h"

namespace eef_controller {
namespace linker_hand {

// ========== 正向映射 (BrainCo → LinkBot, 命令路径) ==========

/// 拇指屈曲(index 0): BrainCo → LinkBot
uint16_t forwardMapThumbFlexion(uint16_t brainco);

/// 拇指侧摆(index 1): BrainCo → LinkBot
uint16_t forwardMapThumbAbduction(uint16_t brainco);

/// 单只手正向映射（in-place 修改 index 0/1）
void forwardMapSingleHandInplace(UnsignedFingerArray &pos);

/// 双手正向映射（in-place 修改 index 0/1）
void forwardMapDualHandsInplace(UnsignedDualHandsArray &pos);

// ========== 反向映射 (LinkBot → BrainCo, 反馈路径) ==========

/// 拇指屈曲(index 0): LinkBot → BrainCo
uint16_t inverseMapThumbFlexion(uint16_t linkbot);

/// 拇指侧摆(index 1): LinkBot → BrainCo
uint16_t inverseMapThumbAbduction(uint16_t linkbot);

}  // namespace linker_hand
}  // namespace eef_controller

#endif
