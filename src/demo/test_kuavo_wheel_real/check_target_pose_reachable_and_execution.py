import rospy
import lb_ctrl_api as ct
from typing import List, Tuple, Dict

def execute_ik_solution(leg_q: List[float], left_arm_q: List[float], right_arm_q: List[float], 
                        desire_time: float = 4.0, is_sync: bool = True) -> Tuple[bool, float, str]:
    """
    执行IK求解出的下肢和手臂关节角度
    
    :param leg_q: 下肢关节角度 (4维, 单位: 弧度)
    :param left_arm_q: 左臂关节角度 (7维, 单位: 弧度)
    :param right_arm_q: 右臂关节角度 (7维, 单位: 弧度)
    :param desire_time: 期望执行时间（秒）
    :param is_sync: 是否同步执行下肢和手臂运动
    :return: (success, actual_time, message)
    """
    # 验证输入维度
    if len(leg_q) != 4:
        error_msg = f"下肢关节角度维度错误, 期望4维, 实际{len(leg_q)}维"
        rospy.logerr(f"❌ {error_msg}")
        return False, 0.0, error_msg
    
    if len(left_arm_q) != 7:
        error_msg = f"左臂关节角度维度错误, 期望7维, 实际{len(left_arm_q)}维"
        rospy.logerr(f"❌ {error_msg}")
        return False, 0.0, error_msg
    
    if len(right_arm_q) != 7:
        error_msg = f"右臂关节角度维度错误, 期望7维, 实际{len(right_arm_q)}维"
        rospy.logerr(f"❌ {error_msg}")
        return False, 0.0, error_msg
    
    rospy.loginfo(f"🦾 执行IK解算结果:")
    rospy.loginfo(f"   下肢关节(rad): {[f'{angle:.4f}' for angle in leg_q]}")
    rospy.loginfo(f"   左臂关节(rad): {[f'{angle:.4f}' for angle in left_arm_q]}")
    rospy.loginfo(f"   右臂关节(rad): {[f'{angle:.4f}' for angle in right_arm_q]}")
    rospy.loginfo(f"   期望时间: {desire_time}s")
    rospy.loginfo(f"   同步模式: {'同步' if is_sync else '异步'}")
    
    # 构建多指令列表
    timed_cmd_vec = [
        {
            'planner_index': 3,   # 下肢规划器索引
            'desire_time': desire_time,
            'cmd_vec': leg_q
        },
        {
            'planner_index': 8,   # 左臂规划器索引
            'desire_time': desire_time,
            'cmd_vec': left_arm_q
        },
        {
            'planner_index': 9,   # 右臂规划器索引
            'desire_time': desire_time,
            'cmd_vec': right_arm_q
        }
    ]
    
    # 发送命令
    success, actual_time, message = ct.send_timed_multi_commands(
        timed_cmd_vec=timed_cmd_vec,
        is_sync=is_sync
    )
    
    if success:
        rospy.loginfo(f"✅ IK解算结果执行成功, 实际时间: {actual_time:.2f}s")
    else:
        rospy.logerr(f"❌ IK解算结果执行失败: {message}")
    
    return success, actual_time, message


def plan_and_execute_ik_target(is_left: bool, is_local: bool, is_whole_body: bool,
                                pose_desired: List[float],
                                desire_time: float = 4.0,
                                fallback_to_position_priority: bool = True) -> Tuple[bool, float, str, Dict]:
    """
    规划并执行IK目标位姿 (完整流程：检查可达性 -> 获取IK解 -> 下发指令）
    
    :param is_left: 是否为左臂
    :param is_local: 是否使用局部坐标系
    :param is_whole_body: 是否使用全身运动
    :param pose_desired: 期望位姿 [x, y, z, roll, pitch, yaw]
    :param desire_time: 执行时间（秒）
    :param fallback_to_position_priority: 是否在精确IK失败时使用位置优先解
    :return: (success, actual_time, message, solution_info)
             solution_info包含: solution_type, q_leg, q_left_arm, q_right_arm, linear_error, angular_error
    """
    
    # 1. 调用IK可达性检查接口
    rospy.loginfo(f"🎯 开始规划目标位姿: {pose_desired}")
    rospy.loginfo(f"   手臂: {'左臂' if is_left else '右臂'}")
    rospy.loginfo(f"   坐标系: {'局部' if is_local else '世界'}")
    rospy.loginfo(f"   运动模式: {'全身运动' if is_whole_body else '仅手臂运动'}")
    
    # 调用已有函数获取IK解, 返回格式: [下肢4维, 左臂7维, 右臂7维]
    # 注意：check_target_pose_reachable_with_fallback 不接受额外参数
    ik_success, q_solution, solution_type, linear_error, angular_error, ik_msg = \
        ct.check_target_pose_reachable_with_fallback(
            is_left=is_left,
            is_local=is_local,
            is_whole_body=is_whole_body,
            pose_desired=pose_desired,
            fallback_to_position_priority=fallback_to_position_priority
        )
    
    # 2. 解析关节角度（统一格式：[下肢4, 左臂7, 右臂7]）
    if not ik_success:
        error_msg = f"❌ 无法获得有效IK解: {ik_msg}"
        rospy.logerr(error_msg)
        solution_info = {
            'solution_type': 'none',
            'q_leg': [],
            'q_left_arm': [],
            'q_right_arm': [],
            'linear_error': linear_error,
            'angular_error': angular_error
        }
        return False, 0.0, error_msg, solution_info
    
    # 解析：期望返回格式为 [下肢4, 左臂7, 右臂7] 共18维
    if len(q_solution) == 18:
        q_leg = q_solution[0:4]
        q_left_arm = q_solution[4:11]
        q_right_arm = q_solution[11:18]
        rospy.loginfo(f"✅ 获得{solution_type}解 (下肢+双臂)")
    else:
        error_msg = f"解算结果维度异常, 期望18维, 实际{len(q_solution)}维"
        rospy.logerr(f"❌ {error_msg}")
        solution_info = {
            'solution_type': 'none',
            'q_leg': [],
            'q_left_arm': [],
            'q_right_arm': [],
            'linear_error': linear_error,
            'angular_error': angular_error
        }
        return False, 0.0, error_msg, solution_info
    
    # 3. 执行运动
    exec_success, actual_time, exec_msg = execute_ik_solution(
        leg_q=q_leg,
        left_arm_q=q_left_arm,
        right_arm_q=q_right_arm,
        desire_time=desire_time,
        is_sync=True
    )
    
    # 4. 构建返回信息
    solution_info = {
        'solution_type': solution_type,
        'q_leg': q_leg,
        'q_left_arm': q_left_arm,
        'q_right_arm': q_right_arm,
        'linear_error': linear_error,
        'angular_error': angular_error
    }
    
    if exec_success:
        success_msg = f"✅ 成功执行{solution_type}解, 位姿误差 - 线:{linear_error:.4f}m, 角:{angular_error:.4f}rad"
        rospy.loginfo(success_msg)
        return True, actual_time, success_msg, solution_info
    else:
        return False, actual_time, exec_msg, solution_info


# 简化版使用示例
def test_ik_execution():
    """测试IK规划执行流程 - 多测试用例模式"""
    
    # 定义多个测试用例, 每个用例保持原有的参数结构
    test_cases = [
        {  # 测试用例1
            "is_left": False,
            "is_local": True,
            "is_whole_body": True,
            "pose_desired": [0.3, -0.4, 0.6, 0.0, 0.0, 0.0],
            "desire_time": 3.0,
            "fallback_to_position_priority": True,
            "description": "右臂-世界坐标系-前上方"
        },
        {  # 测试用例2
            "is_left": True,
            "is_local": True,
            "is_whole_body": True,
            "pose_desired": [0.0, 0.3, 0.6, 0.0, 0.0, 0.0],
            "desire_time": 3.0,
            "fallback_to_position_priority": True,
            "description": "左臂-世界坐标系-左侧"
        },
        {  # 测试用例3
            "is_left": False,
            "is_local": True,
            "is_whole_body": False,
            "pose_desired": [0.2, -0.3, 0.8, 0.0, 0.0, 0.0],
            "desire_time": 2.5,
            "fallback_to_position_priority": True,
            "description": "右臂-世界坐标系-右上方"
        }
    ]
    
    for test_case in test_cases:
        # 原有的调用方式, 保持不变
        success, actual_time, message, info = plan_and_execute_ik_target(
            is_left=test_case["is_left"],
            is_local=test_case["is_local"],
            is_whole_body=test_case["is_whole_body"],
            pose_desired=test_case["pose_desired"],
            desire_time=test_case["desire_time"],
            fallback_to_position_priority=test_case["fallback_to_position_priority"]
        )
        
        # 原有的信息打印, 完全保持不变
        if success:
            rospy.loginfo(f"执行成功！解类型: {info['solution_type']}")
            rospy.loginfo(f"下肢角度: {[f'{ang:.3f}' for ang in info['q_leg']]}")
            rospy.loginfo(f"左臂角度: {[f'{ang:.3f}' for ang in info['q_left_arm']]}")
            rospy.loginfo(f"右臂角度: {[f'{ang:.3f}' for ang in info['q_right_arm']]}")
        else:
            rospy.logerr(f"执行失败: {message}")
        
        rospy.sleep(actual_time + 0.5)  # 等待运动完成
    
    return success


if __name__ == '__main__':
    rospy.init_node('ik_planner_executor', anonymous=True)
    
    # 等待服务就绪
    rospy.sleep(1.0)
    
    # 执行测试
    test_ik_execution()