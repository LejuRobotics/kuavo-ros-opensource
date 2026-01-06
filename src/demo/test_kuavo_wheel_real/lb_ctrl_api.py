#!/usr/bin/env python3
"""
lb_ctrl_api  ——  快速设置移动机械臂控制模式
使用示例：
    import lb_ctrl_api as tc
    tc.set_control_mode(2)
"""
import rospy
from kuavo_msgs.srv import changeTorsoCtrlMode, changeTorsoCtrlModeRequest

def set_control_mode(target_mode: int) -> bool:
    """
    设置移动机械臂控制模式

    :param target_mode: 0~4 对应不同模式
    :return: True 成功，False 失败
    """
    MODES = {
        0: "NoControl",
        1: "ArmOnly",
        2: "BaseOnly",
        3: "BaseArm",
        4: "ArmEeOnly",
    }
    if target_mode not in MODES:
        rospy.logerr(f"无效模式号 {target_mode}，允许值 {list(MODES.keys())}")
        return False

    try:
        rospy.wait_for_service('/mobile_manipulator_mpc_control', timeout=5.0)
        client = rospy.ServiceProxy('/mobile_manipulator_mpc_control', changeTorsoCtrlMode)
        req = changeTorsoCtrlModeRequest()
        req.control_mode = target_mode
        resp = client(req)
        if resp.result:
            rospy.loginfo(f"✅ 已切换到模式 {target_mode}: {MODES[target_mode]}")
            return True
        else:
            rospy.logerr(f"❌ 切换失败: {resp.message}")
            return False
    except rospy.ROSException as e:
        rospy.logerr(f"❌ 服务等待超时: {e}")
        return False
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ 服务调用失败: {e}")
        return False
    except Exception as e:
        rospy.logerr(f"❌ 未知错误: {e}")
        return False