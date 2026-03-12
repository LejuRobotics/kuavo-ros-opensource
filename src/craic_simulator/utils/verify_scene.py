#!/usr/bin/env python3
"""
场景文件完整性校验节点
启动仿真前验证 MuJoCo 场景 XML 文件未被篡改，校验失败则终止仿真。
哈希值从编译后的 craic_secret.so 中读取，选手无法查看。
"""

import sys
import os

# craic_secret.so 位于 ../lib/ 目录
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))

import rospy

try:
    from craic_secret import verify_scene_files
except ImportError:
    rospy.logfatal("verify_scene: 无法导入 craic_secret 模块（.so 文件缺失）")
    sys.exit(1)


def main():
    rospy.init_node("verify_scene", anonymous=False)

    robot_version = rospy.get_param("~robot_version", 47)
    pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    rospy.loginfo("verify_scene: 正在校验场景文件完整性 (robot_version=%s)...", robot_version)

    passed, messages = verify_scene_files(pkg_path, robot_version)

    for msg in messages:
        if passed:
            rospy.logwarn("verify_scene: %s", msg)
        else:
            rospy.logerr("verify_scene: %s", msg)

    if not passed:
        rospy.logfatal("verify_scene: 场景文件校验失败，禁止启动仿真！")
        sys.exit(1)

    rospy.loginfo("verify_scene: 所有场景文件校验通过。")

    # 校验通过后保持运行，避免 required="true" 导致 roslaunch 关闭
    rospy.spin()


if __name__ == "__main__":
    main()
