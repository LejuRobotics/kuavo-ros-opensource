#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通过 ROS 服务触发机器人舞蹈，命令发送成功即提示用户。

调 /humanoid_controller/switch_to_dance_controller (SetString) 触发舞蹈：
  data="" 表示舞蹈列表第一支，"#N" 为下标，其它字符串按舞蹈名匹配。

能否触发完全交由控制器判定：服务返回 success=True 即触发成功；返回 False
说明当前不允许切换（例如正在跳舞、舞蹈列表为空等），脚本据此提示用户即可，
无需自行做状态检测。

用法示例：
  rosrun demo trigger_dance.py                # 触发首支舞
  python3 trigger_dance.py --dance "#1"       # 触发列表下标 1 的舞蹈
  python3 trigger_dance.py --dance dance_xxx  # 按名称触发指定舞蹈
  python3 trigger_dance.py --list             # 列出所有可用舞蹈控制器
"""

import argparse
import sys

import rospy
from kuavo_msgs.srv import (
    SetString, SetStringRequest,
    GetStringList, GetStringListRequest,
)

# 与控制器端保持一致的服务名
SRV_SWITCH_TO_DANCE = "/humanoid_controller/switch_to_dance_controller"
SRV_GET_DANCE_LIST = "/humanoid_controller/get_dance_controller_list"

SERVICE_TIMEOUT = 5.0   # 等待服务上线的超时时间（秒）


def list_dances():
    """打印可用舞蹈控制器列表。"""
    rospy.wait_for_service(SRV_GET_DANCE_LIST, timeout=SERVICE_TIMEOUT)
    resp = rospy.ServiceProxy(SRV_GET_DANCE_LIST, GetStringList)(GetStringListRequest())
    if not resp.success:
        rospy.logerr("[trigger_dance] 获取舞蹈列表失败: %s", resp.message)
        return False
    if not resp.data:
        rospy.loginfo("[trigger_dance] 当前没有已加载的舞蹈控制器")
        return True
    rospy.loginfo("[trigger_dance] 可用舞蹈控制器（共 %d 支）:", len(resp.data))
    for i, name in enumerate(resp.data):
        rospy.loginfo("  [#%d] %s", i, name)
    return True


def switch_to_dance(dance_data=""):
    """调 switch_to_dance_controller 触发舞蹈，由控制器判定能否切换。"""
    req = SetStringRequest()
    req.data = dance_data
    rospy.wait_for_service(SRV_SWITCH_TO_DANCE, timeout=SERVICE_TIMEOUT)
    resp = rospy.ServiceProxy(SRV_SWITCH_TO_DANCE, SetString)(req)
    if resp.success:
        rospy.loginfo("[trigger_dance] 舞蹈指令已发送: %s", resp.message)
    else:
        rospy.logwarn("[trigger_dance] 当前不可触发舞蹈: %s", resp.message)
    return resp.success


def main():
    parser = argparse.ArgumentParser(
        description="触发机器人舞蹈",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--list", action="store_true", help="列出所有可用舞蹈控制器")
    group.add_argument("--dance", metavar="NAME_OR_#IDX", default="",
                       help='指定舞蹈：名称，或 "#N" 下标，或 "" 表示首支（默认）')
    args = parser.parse_args()

    rospy.init_node("trigger_dance", anonymous=True)

    ok = False
    try:
        if args.list:
            ok = list_dances()
        else:
            ok = switch_to_dance(args.dance)
    except KeyboardInterrupt:
        rospy.loginfo("[trigger_dance] 已退出本程序")
        sys.exit(130)
    except rospy.ServiceException as e:
        rospy.logerr("[trigger_dance] 服务调用异常: %s", e)
    except rospy.ROSException as e:
        rospy.logerr("[trigger_dance] 服务未上线（控制器是否已启动？）: %s", e)

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
