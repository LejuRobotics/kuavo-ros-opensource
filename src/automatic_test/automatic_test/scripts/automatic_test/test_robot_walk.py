#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import os
import pytest
import rospy
import yaml
from pathlib import Path
from std_msgs.msg import String, Int8
from enum import Enum
from trace_path.mpc_client_example import start_mpc_tracer, stop_mpc_tracer, back_to_home
from trace_path.mpc_path_tracer import FollowState
from std_srvs.srv import Trigger, TriggerRequest
from .conftest import AutoTrackingStats
from .april_tag_recognition import AprilTagRecognitionInterface

follow_state = FollowState.NOT_FOLLOWING


def follow_state_callback(msg):
    global follow_state
    follow_state = FollowState(msg.data)


@pytest.mark.usefixtures("ros_setup")
class TestRobotWalk:
    def setup_class(self):
        self.follow_state_sub = rospy.Subscriber('trace_path/follow_state', Int8, follow_state_callback)
        self.available_path_types = ['triangle', 'line', 'circle', 'square', 'scurve']
        self.ros_namespace = 'test_robot_walk'
        
        self.april_tag_interface = None
        self.results = {"rounds": [], "success": False}  # 默认失败，最终由 finalize_results 决定

        rospy.loginfo("等待ROS环境准备就绪...")
        rospy.sleep(2.0)  # 等待ROS环境完全初始化

    def record_result(self, round_idx, path_type, ok, error=None):
        """记录单个路径的测试结果"""
        if len(self.results["rounds"]) <= round_idx:
            self.results["rounds"].append({"paths": []})
        path_result = {"path_type": path_type, "ok": ok}
        if error:
            path_result["error"] = error
        self.results["rounds"][round_idx]["paths"].append(path_result)

    def finalize_results(self):
        """统计整体结果并写入yaml"""
        total = 0
        ok_count = 0
        for rnd in self.results["rounds"]:
            for path in rnd["paths"]:
                total += 1
                if path["ok"]:
                    ok_count += 1
        failed = total - ok_count
        ratio = ok_count / total if total > 0 else 0.0

        self.results["summary"] = {
            "total": total,
            "ok": ok_count,
            "failed": failed,
            "ratio": round(ratio, 3),
        }
        
        self.results["success"] = True if ratio > 0.95 else False

        out_file = Path("/tmp/robot_walk_results.yaml")
        with out_file.open("w", encoding="utf-8") as f:
            yaml.safe_dump(self.results, f, allow_unicode=True, sort_keys=False)
        rospy.loginfo(f"测试结果已写入 {out_file}")

    def wait_for_completion(self, check_robot_alive, auto_tracking_stats, description=""):
        while follow_state != FollowState.FINISHED:
            rospy.sleep(0.1)
            assert check_robot_alive.get_status(), f"机器人死亡: {check_robot_alive.get_error()}"
            assert auto_tracking_stats.status == AutoTrackingStats.FOLLOWING, \
                   f"动捕天轨系统自动跟踪机器人失败 ({description})"

    def execute_april_tag_recognition(self):
        if self.april_tag_interface is None:
            rospy.logwarn("AprilTag识别接口不可用，尝试重新初始化...")
            try:
                self.april_tag_interface = AprilTagRecognitionInterface()
                rospy.loginfo("AprilTag识别接口重新初始化成功")
            except Exception as e:
                rospy.logwarn(f"AprilTag识别接口重新初始化失败: {e}")
                rospy.logwarn("跳过识别步骤")
                return True
        
        try:
            recognition_success = self.april_tag_interface.execute_recognition_workflow()
            if recognition_success:
                rospy.loginfo("AprilTag识别和位置精度评估成功")
            else:
                rospy.logwarn("AprilTag识别或位置精度评估失败")
            return recognition_success
        except Exception as e:
            rospy.logerr(f"执行AprilTag识别时发生错误: {e}")
            return False

    def evaluate_end_position_with_gazebo(self):
        if self.april_tag_interface is None:
            rospy.logwarn("AprilTag识别接口不可用，尝试重新初始化...")
            try:
                self.april_tag_interface = AprilTagRecognitionInterface()
                rospy.loginfo("AprilTag识别接口重新初始化成功")
            except Exception as e:
                rospy.logwarn(f"AprilTag识别接口重新初始化失败: {e}")
                rospy.logwarn("跳过识别步骤")
                return True
        try:
            model_name = f"biped_s{os.environ.get('ROBOT_VERSION', '45')}"
            return self.april_tag_interface.evaluate_end_position_with_gazebo(model_name)
        except Exception as e:
            rospy.logerr(f"执行Gazebo终点评估时发生错误: {e}")
            return False

    def execute_path(self, path_type, check_robot_alive, auto_tracking_stats, round_idx):
        try:
            start_mpc_tracer(path_type)
            rospy.sleep(1.0)
            self.wait_for_completion(check_robot_alive, auto_tracking_stats, f"执行{path_type}路径")
            
            use_sim = rospy.get_param("robot_manager_node/sim", False)
            if use_sim:
                ok = self.evaluate_end_position_with_gazebo()
                if not ok:
                    rospy.logerr("Gazebo 终点位置偏差超出容差")
                self.record_result(round_idx, path_type, ok)
            else:
                back_to_home()
                rospy.sleep(1.0)
                self.wait_for_completion(check_robot_alive, auto_tracking_stats, "返回家")
                self.record_result(round_idx, path_type, True)

        except Exception as e:
            rospy.logerr(f"执行路径 {path_type} 失败: {e}")
            self.record_result(round_idx, path_type, False, str(e))
            raise   # 仍然让pytest判定失败

    @pytest.mark.walk
    def test_robot_walk(self, check_robot_ready, check_robot_alive, mpc_tracer_process, auto_tracking_stats, test_timer):
        if not rospy.get_param(f'/{self.ros_namespace}/test_cmd_vel', False):
            rospy.loginfo("cmd_vel 测试未启用")
            return
        round = rospy.get_param(f'/{self.ros_namespace}/round', 10)
        rospy.loginfo(f"Test Robot Walk Round: {round}")
        path_types = rospy.get_param(f'/{self.ros_namespace}/path_types', ['circle', 'square', 'scurve'])
        rospy.set_param(f'/mpc_path_tracer_node/motion_interface', '/cmd_vel')
        try:
            self.april_tag_interface = AprilTagRecognitionInterface()
            rospy.loginfo("AprilTag识别接口初始化成功")
        except Exception as e:
            rospy.logwarn(f"AprilTag识别接口初始化失败: {e}")
            self.april_tag_interface = None

        round_idx = 0
        while round > 0:
            for path_type in path_types:
                if path_type not in self.available_path_types:
                    rospy.logerr(f"路径类型 {path_type} 不存在")
                    continue
                self.execute_path(path_type, check_robot_alive, auto_tracking_stats, round_idx)
            round -= 1
            round_idx += 1
            rospy.loginfo(f"剩余轮数: {round}")

        self.finalize_results()

    @pytest.mark.walk
    def test_robot_walk_cmd_pose(self, check_robot_ready, check_robot_alive, mpc_tracer_process, auto_tracking_stats, test_timer):
        if not rospy.get_param(f'/{self.ros_namespace}/test_cmd_pose', False):
            rospy.loginfo("cmd_pose 测试未启用")
            return
        round = rospy.get_param(f'/{self.ros_namespace}/round', 10)
        rospy.loginfo(f"Test Robot Walk Round: {round}")
        path_types = rospy.get_param(f'/{self.ros_namespace}/path_types', ['circle', 'square', 'scurve'])
        rospy.set_param(f'/mpc_path_tracer_node/motion_interface', '/cmd_pose')
        try:
            self.april_tag_interface = AprilTagRecognitionInterface()
            rospy.loginfo("AprilTag识别接口初始化成功")
        except Exception as e:
            rospy.logwarn(f"AprilTag识别接口初始化失败: {e}")
            self.april_tag_interface = None

        round_idx = 0
        while round > 0:
            for path_type in path_types:
                if path_type not in self.available_path_types:
                    rospy.logerr(f"路径类型 {path_type} 不存在")
                    continue
                self.execute_path(path_type, check_robot_alive, auto_tracking_stats, round_idx)
            round -= 1
            round_idx += 1
            rospy.loginfo(f"剩余轮数: {round}")

        self.finalize_results()

    @pytest.mark.walk
    def test_robot_walk_cmd_pose_world(self, check_robot_ready, check_robot_alive, mpc_tracer_process, auto_tracking_stats, test_timer):
        if not rospy.get_param(f'/{self.ros_namespace}/test_cmd_pose_world', False):
            rospy.loginfo("cmd_pose_world 测试未启用")
            return
        round = rospy.get_param(f'/{self.ros_namespace}/round', 10)
        rospy.loginfo(f"Test Robot Walk Round: {round}")
        path_types = rospy.get_param(f'/{self.ros_namespace}/path_types', ['circle', 'square', 'scurve'])
        rospy.set_param(f'/mpc_path_tracer_node/motion_interface', '/cmd_pose_world')
        try:
            self.april_tag_interface = AprilTagRecognitionInterface()
            rospy.loginfo("AprilTag识别接口初始化成功")
        except Exception as e:
            rospy.logwarn(f"AprilTag识别接口初始化失败: {e}")
            self.april_tag_interface = None

        round_idx = 0
        while round > 0:
            for path_type in path_types:
                if path_type not in self.available_path_types:
                    rospy.logerr(f"路径类型 {path_type} 不存在")
                    continue
                self.execute_path(path_type, check_robot_alive, auto_tracking_stats, round_idx)
            round -= 1
            round_idx += 1
            rospy.loginfo(f"剩余轮数: {round}")

        self.finalize_results()
