#!/bin/bash
# 触发 humanoid_controller nodelet 故意段错误,生成 core dump
# 用于验证 strip 后的 .so 库在 core dump 下的 gdb 调试能力。
#
# 配套代码: src/humanoid-control/humanoid_controllers/src/controllerNodelet.cpp
#           crashTestCallback —— 订阅 /crash_test_trigger,收到后解引用空指针触发 SIGSEGV
#
# 前置条件:
#   1. 仿真从 installed 空间启动 (source installed/setup.bash,不要用 devel —— devel 未 strip)
#   2. 以 root/sudo 启动,并带 coredump:=true (start_node.sh 仅在 root 下才开 core dump):
#        roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch coredump:=true
#   3. 控制器已初始化
#
# 用法: bash crash_test_trigger.sh
#
# 崩溃进程是 nodelet_manager(crashTestCallback 在其加载的 libnodelet_controller.so 内)。
# core 落点 (由 start_node.sh 设定): ~/.ros/coredumps/<PPID>/core.nodelet.<pid>.<t>
#   找 core : find ~/.ros/coredumps -name 'core.nodelet.*' -printf '%T@ %p\n' | sort -nr | head -1
#   手动 gdb: gdb $(which nodelet || echo /opt/ros/noetic/lib/nodelet/nodelet) <core>
#     控制线程经过的 installed/lib stripped 库帧应显示 ?? / 无行号 → strip 生效(场景 a)

set -e

# 1. 发布话题触发 crash
rostopic pub -1  /crash_test_trigger std_msgs/Bool "data: true"

