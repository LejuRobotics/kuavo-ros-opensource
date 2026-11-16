#!/usr/bin/env bash
# 标定流程统一机型表（bash 侧，与 robot_versions.py 一一对应）。
#
# 用法（脚本内）：
#   _RV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # 按脚本所在目录调整
#   source "${_RV_DIR}/robot_versions.sh"
#   rv_fold 200049          # -> 49
#   rv_layout 100049        # -> biped45
#   rv_urdf 400062          # -> biped_v3_arm_s62.urdf
#   rv_head_rot180 45       # -> true
#   rv_supports_wrist 49 || echo "不支持腕相机"
#   rv_supported 200053     # 支持则返回 0
#
# 三条约定同 robot_versions.py：
#   1) 小版本按 %100 折回基础型号（100049→49、200053→53、400062→62 …）；
#   2) 标定按基础型号走：布局/URDF/FK 根/头相机旋转/腕相机/零点槽位/EC 归属查本表；
#   3) kuavo_v${ROBOT_VERSION}/、biped_s${ROBOT_VERSION}/ 等资源路径仍用原始版本号。
#
# 字段顺序（| 分隔）：
#   base|layout|urdf|calibrated_urdf|fk_root|head_rot180|supports_wrist|zero_slots|ec_l1_r1|
#   apply_script|calib_config|calib_config_qingtong|shoulder_profile

# shellcheck disable=SC2034
RV_SUPPORTED_BASES="45 49 52 53 55 56 62 63"

rv_fold() {
  local v="${1:-}"
  if [[ "${v}" =~ ^[0-9]+$ ]]; then
    echo "$(( 10#${v} % 100 ))"
  else
    echo "${v}"
  fi
}

# 45 与 49 系列同布局，但 URDF/标定配置/输出各自独立（避免 49 覆盖 45 的标定结果）。
# 参数：base rot180 urdf_suffix（s45 / s49）
_rv_biped45_row() {
  echo "$1|biped45|biped_v3_arm_$3.urdf|biped_v3_arm_$3_calibrated.urdf|base_link|$2|false|14|true|apply_zero_deltas_to_arms_zero.py|calib_$3.yaml|calib_qingtong_$3.yaml|s45"
}

# 53/55/56 同属 biped56 布局，模板与输出独立；zhead_1_joint 相对 waist_yaw_link 的 Z 坐标统一为 0.49575 m。
_rv_record() {
  local base
  base="$(rv_fold "${1:-}")"
  case "${base}" in
    45) _rv_biped45_row 45 true s45 ;;
    49) _rv_biped45_row 49 false s49 ;;
    52) echo "52|biped52|biped_v3_arm.urdf|biped_v3_arm_calibrated.urdf|waist_yaw_link|false|true|14|true|apply_zero_deltas_to_arms_zero.py|calib.yaml|calib_qingtong.yaml|default" ;;
    53) echo "53|biped56|biped_v3_arm_s53.urdf|biped_v3_arm_s53_calibrated.urdf|waist_yaw_link|false|true|16|false|apply_zero_deltas_to_arms_zero.py|calib.yaml|calib_qingtong.yaml|default" ;;
    55) echo "55|biped56|biped_v3_arm_s55.urdf|biped_v3_arm_s55_calibrated.urdf|waist_yaw_link|false|true|16|false|apply_zero_deltas_to_arms_zero.py|calib.yaml|calib_qingtong.yaml|default" ;;
    56) echo "56|biped56|biped_v3_arm_s56.urdf|biped_v3_arm_s56_calibrated.urdf|waist_yaw_link|false|true|16|false|apply_zero_deltas_to_arms_zero.py|calib.yaml|calib_qingtong.yaml|default" ;;
    62) echo "62|wheel62|biped_v3_arm_s62.urdf|biped_v3_arm_s62_calibrated.urdf|waist_yaw_link|false|true|16|false|apply_zero_deltas_to_arms_zero_wheel62.py|calib.yaml|calib_qingtong.yaml|default" ;;
    63) echo "63|wheel62|biped_v3_arm_s63.urdf|biped_v3_arm_s63_calibrated.urdf|waist_yaw_link|false|true|16|false|apply_zero_deltas_to_arms_zero_wheel62.py|calib.yaml|calib_qingtong.yaml|default" ;;
    *) return 1 ;;
  esac
}

_rv_field() {
  local rec
  rec="$(_rv_record "${1:-}")" || return 1
  printf '%s\n' "${rec}" | cut -d'|' -f"${2}"
}

rv_supported() { _rv_record "${1:-}" >/dev/null 2>&1; }
rv_base()      { _rv_field "${1:-}" 1; }
rv_layout()    { _rv_field "${1:-}" 2; }
rv_urdf()      { _rv_field "${1:-}" 3; }
rv_calibrated_urdf() { _rv_field "${1:-}" 4; }
rv_fk_root()   { _rv_field "${1:-}" 5; }
rv_head_rot180()      { _rv_field "${1:-}" 6; }
rv_zero_slots()       { _rv_field "${1:-}" 8; }
rv_apply_script()     { _rv_field "${1:-}" 10; }
rv_calib_config()     { _rv_field "${1:-}" 11; }
rv_calib_config_qingtong() { _rv_field "${1:-}" 12; }
rv_shoulder_profile() { _rv_field "${1:-}" 13; }

# 以下两个返回字符串 true/false（与 rv_head_rot180 一致，便于日志与赋值）。
# 需要布尔判断时用 [[ "$(rv_supports_wrist "${ROBOT_VERSION}")" == "true" ]]。
rv_supports_wrist()   { _rv_field "${1:-}" 7; }
rv_ec_l1_r1()         { _rv_field "${1:-}" 9; }

# 一行机型摘要，用于日志。
rv_describe() {
  local v="${1:-}" base layout wrist
  base="$(rv_fold "${v}")"
  if ! rv_supported "${v}"; then
    echo "ROBOT_VERSION=${v}（不支持，折回基础型号=${base}；支持 ${RV_SUPPORTED_BASES} 及对应小版本）"
    return 0
  fi
  layout="$(rv_layout "${v}")"
  if [[ "$(rv_supports_wrist "${v}")" == "true" ]]; then
    wrist="支持"
  else
    wrist="不支持"
  fi
  echo "ROBOT_VERSION=${v}（基础型号 ${base}，布局 ${layout}，URDF $(rv_urdf "${v}")，FK 根 $(rv_fk_root "${v}")，头相机旋转 $(rv_head_rot180 "${v}")，腕相机 ${wrist}，零点 $(rv_zero_slots "${v}") 维）"
}
