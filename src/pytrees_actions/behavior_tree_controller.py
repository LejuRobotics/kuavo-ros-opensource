#!/usr/bin/env python3
import rospy
import actionlib
import threading
import json
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse
from pytrees_actions.msg import GetBtStateAction, GetBtStateFeedback, GetBtStateResult
from pytrees_actions.srv import SetMaxIterations, SetMaxIterationsResponse
import time
from collections import defaultdict
import os
from datetime import datetime

# 尝试导入matplotlib，如果失败则禁用可视化功能
try:
    import matplotlib
    matplotlib.use('Agg')  # 使用非交互式后端
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.colors import LinearSegmentedColormap
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    # 注意：此时rospy可能还未初始化，所以不在这里输出日志

def update_bt_state(behavior_tree):
    """报告当前行为树的子树和节点运行状态

    Args:
        behavior_tree: 要检查状态的行为树实例

    Returns:
        dict: 包含行为树状态信息的字典
    """
    if behavior_tree is None:
        return {"status": "error", "message": "行为树实例为None"}

    # 创建状态报告
    status_report = {
        "root_name": behavior_tree.name,
        "root_status": str(behavior_tree.status),
        "timestamp": time.time(),
        "nodes": []
    }


class BehaviorTreeController:
    def __init__(self, behavior_tree_core):
        # 注入行为树核心实例
        self.bt_core = behavior_tree_core
        # 执行控制相关状态
        self.running_flag = False
        self.paused_flag = False  # 添加暂停标志
        self.bt_instance = None
        self.action_server = None
        # 添加存储行为树配置的属性
        self._last_tree_json_path = None
        self._last_blackboard_client = None
        # 存储行为树线程引用
        self.bt_thread = None
        # 性能监控和循环控制
        self.max_iterations = 2  # -1表示无限循环，>0表示最大执行次数
        self.current_iteration = 0
        self.node_execution_times = defaultdict(list)  # 节点名 -> [执行时间列表]
        self.total_execution_times = []  # 每次完整执行的总时间
        self.enable_performance_log = False  # 是否启用性能日志
        self.current_execution_start_time = None  # 当前执行开始时间
        self.last_root_status = None  # 上一次根节点状态，用于检测状态变化

    def _clear_task_status(self):
        """清除任务状态参数"""
        try:
            if rospy.has_param('/task_status'):
                rospy.delete_param('/task_status')
                rospy.loginfo("[行为树启动] 已清除 /task_status 参数")
        except Exception as e:
            rospy.logwarn(f"[行为树启动] 清除参数失败: {e}")

    def _handle_tree_failure(self):
        """处理行为树执行失败时的参数服务器操作"""
        try:
            if rospy.has_param('/task_status'):
                rospy.delete_param('/task_status')
            rospy.set_param("/task_status", "false")
            rospy.loginfo(f"抓取执行失败")
        except Exception as e:
            rospy.logerr(f"[行为树失败处理] 设置参数服务器失败: {e}")

    def start_behavior_tree(self, tree_json_path, blackboard_client=None):
        """启动行为树"""
        # 存储配置信息以便重启使用
        self._last_tree_json_path = tree_json_path
        self._last_blackboard_client = blackboard_client

        # 清除任务状态参数
        self._clear_task_status()

        # 设置运行标志为True，重置暂停标志
        self.running_flag = True
        self.paused_flag = False
        # 重置执行计数和状态
        self.current_iteration = 0
        self.current_execution_start_time = None
        self.last_root_status = None

        # 如果启用了性能日志，同时启用性能监控模块
        if self.enable_performance_log:
            try:
                from node.performance_monitor import enable_performance_monitoring, record_phase_start
                enable_performance_monitoring(True)
                record_phase_start("BEHAVIOR_TREE_EXECUTION")
                rospy.loginfo("[性能监控] 已启用性能监控模块，开始记录完整执行流程")
            except Exception as e:
                rospy.logwarn(f"[性能监控] 启用性能监控模块失败: {e}")

        try:
            # 如果已有行为树实例，则直接使用它（用于恢复暂停的树）
            if not self.bt_instance:
                # 使用核心类构建行为树
                self.bt_instance = self.bt_core.load_tree_from_json(tree_json_path)
                if not self.bt_instance:
                    rospy.logerr("行为树构建失败，无法启动")
                    self.running_flag = False
                    return
            else:
                rospy.loginfo("使用已存在的行为树实例")

            rate = rospy.Rate(50)  # 50Hz 更新频率
            rospy.loginfo("行为树已启动，开始输出日志...")
            if self.max_iterations > 0:
                rospy.loginfo(f"[行为树执行] 最大循环次数: {self.max_iterations}")

            # 主循环，响应运行标志和暂停标志
            while not rospy.is_shutdown() and self.running_flag:
                if self.paused_flag:
                    rospy.loginfo_throttle(1, "[行为树执行] 行为树已暂停")
                else:
                    # 检查循环次数限制
                    # if self.max_iterations > 0 and self.current_iteration >= self.max_iterations:
                    #     rospy.loginfo(f"[行为树执行] 已达到最大循环次数 {self.max_iterations}，停止执行")
                    #     self.running_flag = False
                    #     break

                    # 获取当前根节点状态
                    current_root_status = None
                    if self.bt_instance and hasattr(self.bt_instance, 'root') and self.bt_instance.root:
                        current_root_status = self.bt_instance.root.status

                    # 检测状态变化：从 RUNNING 变为 SUCCESS 或 FAILURE，表示完成一次执行
                    from py_trees import common
                    execution_completed = False
                    if self.last_root_status is not None and current_root_status is not None:
                        # 如果上一次是 RUNNING，现在不是 RUNNING，说明完成了一次执行
                        if (self.last_root_status == common.Status.RUNNING and
                            current_root_status != common.Status.RUNNING):
                            execution_completed = True

                    # 记录一次完整执行的开始时间（当状态变为 RUNNING 时）
                    if (current_root_status == common.Status.RUNNING and
                        self.current_execution_start_time is None):
                        self.current_execution_start_time = time.time()
                        # 重置性能监控的时间基准，使用实际执行开始时间，确保时间从0开始
                        if self.enable_performance_log:
                            try:
                                from node.performance_monitor import reset_execution_start_time, clear_performance_data
                                reset_execution_start_time(self.current_execution_start_time)
                                # 清空节点执行时间统计，只统计这次执行的数据
                                self.node_execution_times.clear()
                            except Exception as e:
                                rospy.logwarn(f"[性能监控] 重置时间基准失败: {e}")

                    # 执行一次tick
                    tick_start = time.time()

                    # 记录tick开始
                    if self.enable_performance_log:
                        try:
                            from node.performance_monitor import record_phase_start
                            record_phase_start(f"TICK_{self.current_iteration + 1}")
                        except Exception as e:
                            pass

                    tick_result = self.bt_core.tick()
                    tick_end = time.time()
                    tick_duration = tick_end - tick_start

                    # 记录tick结束
                    if self.enable_performance_log:
                        try:
                            from node.performance_monitor import record_phase_end
                            record_phase_end(f"TICK_{self.current_iteration + 1}")
                        except Exception as e:
                            pass

                    # 更新根节点状态（tick 后可能状态已变化）
                    if self.bt_instance and hasattr(self.bt_instance, 'root') and self.bt_instance.root:
                        current_root_status = self.bt_instance.root.status

                    # 注意：不在每个tick后累积节点执行时间，只在执行完成时统计
                    # 这样可以避免累积所有tick的时间，只统计单次执行的时间

                    # 检测执行完成：状态从 RUNNING 变为 SUCCESS/FAILURE
                    if (self.last_root_status == common.Status.RUNNING and
                        current_root_status is not None and
                        current_root_status != common.Status.RUNNING):
                        execution_completed = True

                    # 如果检测到执行完成
                    if execution_completed:
                        # 记录总执行时间
                        if self.current_execution_start_time is not None:
                            total_time = tick_end - self.current_execution_start_time
                            self.total_execution_times.append(total_time)
                            self.current_iteration += 1

                            # 总是输出总执行时间
                            status_str = str(current_root_status).replace("Status.", "")
                            rospy.loginfo(f"[性能统计] 第 {self.current_iteration} 次完整执行总时间: {total_time*1000:.2f}ms, 最终状态: {status_str}")
                            
                            # 如果执行失败，处理失败逻辑
                            if current_root_status == common.Status.FAILURE:
                                self._handle_tree_failure()

                            # 如果启用了性能监控，收集并输出完整执行流程统计
                            if self.enable_performance_log:
                                try:
                                    from node.performance_monitor import (
                                        get_performance_data, clear_performance_data,
                                        get_execution_timeline, get_execution_phases,
                                        record_execution_end
                                    )

                                    # 记录执行结束事件
                                    status_str_for_timeline = "SUCCESS" if "SUCCESS" in status_str else "FAILURE"
                                    record_execution_end(status_str_for_timeline)

                                    # 获取这次执行期间的节点性能数据（只统计这次执行的数据）
                                    node_perf_data = get_performance_data()
                                    if node_perf_data:
                                        # 只统计这次执行的数据，不累积之前的数据
                                        for node_name, times in node_perf_data.items():
                                            self.node_execution_times[node_name] = list(times)  # 直接赋值，不extend

                                    # 输出完整的执行流程分析
                                    self._print_complete_execution_analysis()

                                    # 清空性能数据，为下一次执行准备
                                    clear_performance_data()
                                    # 同时清空节点执行时间统计，为下一次执行准备
                                    self.node_execution_times.clear()
                                except Exception as e:
                                    import traceback
                                    traceback.print_exc()
                                    rospy.logwarn(f"[性能监控] 获取执行流程数据失败: {e}")
                                    # 输出节点详细统计（降级模式）
                                if self.node_execution_times:
                                    self._print_node_performance_summary()
                                else:
                                    rospy.logwarn("[性能监控] 未收集到节点执行时间数据，请确保：1) 性能监控已启用 2) 节点使用了@performance_monitor装饰器")

                            self.current_execution_start_time = None

                            # 检查是否达到最大循环次数
                            if self.max_iterations > 0 and self.current_iteration >= self.max_iterations:
                                rospy.loginfo(f"[行为树执行] 已完成 {self.current_iteration} 次执行，达到最大循环次数，停止执行")
                                self.running_flag = False
                                break

                            # 如果设置了循环次数且未达到，重置行为树继续执行
                            if self.max_iterations > 0 and self.current_iteration < self.max_iterations:
                                rospy.loginfo(f"[行为树执行] 准备第 {self.current_iteration + 1} 次执行...")
                                # 重置行为树状态以便重新执行
                                if self.bt_instance and hasattr(self.bt_instance, 'root') and self.bt_instance.root:
                                    try:
                                        self.bt_instance.root.stop(common.Status.INVALID)
                                    except:
                                        pass
                                self.last_root_status = None  # 重置状态记录
                                continue

                    # 如果行为树执行失败（返回False），停止执行
                    if tick_result == False:
                        rospy.logwarn("[行为树执行] 行为树执行失败，停止执行")
                        self._handle_tree_failure()
                        self.running_flag = False
                        break

                    # 更新上一次的状态
                    if current_root_status is not None:
                        self.last_root_status = current_root_status

                    # 继续下一次tick
                    rate.sleep()

        except Exception as e:
            # 记录异常结束
            if self.enable_performance_log:
                try:
                    from node.performance_monitor import record_phase_end
                    record_phase_end("BEHAVIOR_TREE_EXECUTION")
                except:
                    pass
            rospy.logerr(f"行为树执行异常：{str(e)}")
            import traceback
            traceback.print_exc()
        finally:
            self.running_flag = False
            # 记录正常结束
            if self.enable_performance_log:
                try:
                    from node.performance_monitor import record_phase_end
                    record_phase_end("BEHAVIOR_TREE_EXECUTION")
                except:
                    pass
            rospy.loginfo("[行为树执行] 行为树主循环已退出")

    def restart_behavior_tree(self, req=None):
        """重启行为树（如果是暂停状态则恢复）"""
        # 检查是否处于暂停状态
        if self.paused_flag:
            rospy.loginfo("[行为树控制] 恢复暂停的行为树")
            self.paused_flag = False
            return EmptyResponse() if req is not None else None

        # 先停止当前运行的行为树
        self.stop_behavior_tree()
        # 延迟一小段时间确保完全停止
        rospy.sleep(0.5)
        # 重新启动行为树
        rospy.loginfo("[行为树控制] 正在重启行为树...")

        # 检查是否有保存的配置信息
        if self._last_tree_json_path is None:
            rospy.logerr("[行为树控制] 无法重启行为树：未找到之前使用的tree_json_path")
            return EmptyResponse() if req is not None else None

        # 在新线程中启动行为树，避免阻塞，并使用保存的配置
        self.bt_thread = threading.Thread(
            target=self.start_behavior_tree,
            args=(self._last_tree_json_path, self._last_blackboard_client)
        )
        self.bt_thread.daemon = True
        self.bt_thread.start()
        rospy.loginfo("[行为树控制] 行为树重启成功")
        return EmptyResponse() if req is not None else None

    def init_services(self):
        """初始化控制服务"""
        rospy.Service('/stop_behavior_tree', Empty, self.stop_behavior_tree)
        rospy.Service('/restart_behavior_tree', Empty, self.restart_behavior_tree)
        rospy.Service('/pause_behavior_tree', Empty, self.pause_behavior_tree)  # 添加暂停服务
        rospy.Service('/set_max_iterations', SetMaxIterations, self.set_max_iterations)  # 设置最大循环次数
        rospy.Service('/enable_performance_log', SetBool, self.enable_performance_log)  # 启用/禁用性能日志
        rospy.Service('/print_performance_stats', Empty, self.print_performance_stats)  # 打印性能统计
        rospy.loginfo("控制服务已注册：/stop_behavior_tree, /restart_behavior_tree, /pause_behavior_tree, "
                     "/set_max_iterations, /enable_performance_log, /print_performance_stats")

    def pause_behavior_tree(self, req=None):
        """暂停行为树的执行，但保留当前状态"""
        if self.running_flag and not self.paused_flag:
            self.paused_flag = True
            rospy.loginfo("[行为树控制] 已暂停行为树执行")
        elif self.paused_flag:
            rospy.loginfo("[行为树控制] 行为树已处于暂停状态")
        else:
            rospy.loginfo("[行为树控制] 行为树未在运行，无需暂停")
        return EmptyResponse() if req is not None else None

    def stop_behavior_tree(self, req=None):
        """停止行为树的执行"""
        if self.running_flag:
            self.running_flag = False
            self.paused_flag = False  # 同时清除暂停标志
            rospy.loginfo("[行为树控制] 已停止行为树执行")
            # 重置所有节点状态
            if self.bt_instance:
                try:
                    # 首先尝试标准的root属性访问方式
                    if hasattr(self.bt_instance, 'root') and self.bt_instance.root:
                        from py_trees import common
                        self.bt_instance.root.stop(common.Status.INVALID)
                    # 如果是Sequence类型或没有root属性，直接尝试stop方法
                    elif hasattr(self.bt_instance, 'stop'):
                        from py_trees import common
                        self.bt_instance.stop(common.Status.INVALID)
                    else:
                        rospy.logwarn(f"[行为树控制] 无法重置节点状态: bt_instance是{type(self.bt_instance)}类型，没有root属性或stop方法")
                except Exception as e:
                    rospy.logerr(f"[行为树控制] 重置节点状态时出错: {str(e)}")
                    import traceback
                    traceback.print_exc()
        else:
            rospy.loginfo("[行为树控制] 行为树未在运行，无需停止")
        return EmptyResponse() if req is not None else None

    def _bt_state_action_execute_cb(self, goal):
        """Action服务回调函数"""
        rospy.loginfo("start behavior tree state action")
        feedback = GetBtStateFeedback()
        result = GetBtStateResult()

        # 获取当前状态信息
        result.total_node_count = len(self.bt_core.real_time_state)
        root_status = str(self.bt_core.root_node.status).replace("Status.", "") if self.bt_core.root_node else "Unknown"
        result.root_node_status = root_status

        if goal.query_type == "single":
            # 单次查询模式
            if not self.running_flag:
                # 如果未运行，启动一次行为树
                temp_bt = self.bt_core.build_and_get_behavior_tree()
                if temp_bt:
                    temp_bt.tick()
                    self.bt_core.update_bt_state()

            # 处理查询目标
            if goal.target_node_label.strip():
                target = goal.target_node_label.strip()
                result.all_nodes_state = json.dumps({target: self.bt_core.real_time_state.get(target, {})})
                result.message = f"查询节点：{target}"
            else:
                result.all_nodes_state = json.dumps(self.bt_core.real_time_state)
                result.message = f"查询所有节点（共 {len(self.bt_core.real_time_state)} 个）"

            result.success = True  # 单次查询成功，显式设为 True
            self.action_server.set_succeeded(result)
            return

        elif goal.query_type == "realtime_feedback":
            # 实时反馈模式
            start_time = rospy.Time.now()
            prev_state = json.dumps(self.bt_core.real_time_state)

            # 如果未运行，启动行为树
            if not self.running_flag:
                if self._last_tree_json_path is None:
                    rospy.logerr("[行为树控制] 无法启动行为树：未找到tree_json_path")
                    result.success = False
                    result.message = "无法启动行为树：未找到tree_json_path"
                    self.action_server.set_succeeded(result)
                    return
                self.start_behavior_tree(self._last_tree_json_path, self._last_blackboard_client)

            while not rospy.is_shutdown():
                if self.action_server.is_preempt_requested():
                    result.success = False
                    result.message = "已取消实时反馈"
                    self.action_server.set_preempted(result)
                    return

                if goal.timeout > 0 and (rospy.Time.now() - start_time).to_sec() > goal.timeout:
                    break

                # 发布反馈
                feedback.root_node_status = result.root_node_status
                feedback.updated_node_count = len(self.bt_core.real_time_state)
                current_state = json.dumps(self.bt_core.real_time_state)

                if current_state != prev_state:
                    latest_label = max(self.bt_core.real_time_state.keys(), key=lambda x: self.bt_core.real_time_state[x]["timestamp"])
                    feedback.latest_node_state = f"{latest_label}: {self.bt_core.real_time_state[latest_label]['status']}"
                    self.action_server.publish_feedback(feedback)
                    prev_state = current_state

                rospy.sleep(0.5)

            result.all_nodes_state = json.dumps(self.bt_core.real_time_state)
            result.message = f"实时反馈结束（超时 {goal.timeout} 秒）"
            self.action_server.set_succeeded(result)
            return
        else:
            result.success = False
            result.message = "无效查询类型（仅支持 single/realtime_feedback）"
            self.action_server.set_succeeded(result)
            return

    def set_max_iterations(self, req):
        """设置最大循环次数"""
        self.max_iterations = req.max_iterations
        self.current_iteration = 0  # 重置当前计数
        self.last_root_status = None  # 重置状态记录
        rospy.loginfo(f"[行为树控制] 设置最大循环次数: {self.max_iterations} (-1表示无限循环)")
        return SetMaxIterationsResponse(success=True, message=f"最大循环次数已设置为: {self.max_iterations}")

    def enable_performance_log(self, req):
        """启用/禁用性能日志"""
        self.enable_performance_log = req.data
        # 同时启用/禁用性能监控模块
        try:
            from node.performance_monitor import enable_performance_monitoring
            enable_performance_monitoring(req.data)
        except Exception as e:
            rospy.logwarn(f"[性能监控] 启用性能监控模块失败: {e}")
        status = "启用" if self.enable_performance_log else "禁用"
        rospy.loginfo(f"[性能监控] 性能日志已{status}")
        return SetBoolResponse(success=True, message=f"性能日志已{status}")

    def print_performance_stats(self, req=None):
        """打印性能统计信息"""
        self._print_performance_summary()
        return EmptyResponse() if req is not None else None

    def _print_complete_execution_analysis(self):
        """打印完整的执行流程分析"""
        try:
            from node.performance_monitor import get_execution_timeline, get_execution_phases

            timeline = get_execution_timeline()

            if not timeline:
                rospy.logwarn("[执行流程分析] 未收集到执行时间线数据")
                return

            # 使用实际测量的总执行时间（从 current_execution_start_time 计算）
            # 这个时间应该和日志中显示的"完整执行总时间"一致
            if self.total_execution_times:
                actual_total_time = self.total_execution_times[-1]  # 最后一次执行的总时间
            else:
                # 如果没有，则从时间线计算
                start_events = [e for e in timeline if e['event_type'] == 'EXECUTION_START']
                end_events = [e for e in timeline if e['event_type'] == 'EXECUTION_END']
                if start_events and end_events:
                    actual_total_time = end_events[-1]['relative_time'] - start_events[0]['relative_time']
                else:
                    actual_total_time = 0

            rospy.loginfo("\n" + "="*80)
            rospy.loginfo("[完整执行流程分析]")
            rospy.loginfo("="*80)
            rospy.loginfo(f"总执行时间: {actual_total_time:.3f}s ({actual_total_time*1000:.2f}ms)")

            # 分析节点执行时间线（这是最重要的，显示每个节点的执行情况）
            self._print_execution_timeline(timeline, actual_total_time)

            # 生成可视化图表
            if MATPLOTLIB_AVAILABLE:
                try:
                    from node.performance_monitor import get_execution_phases
                    phases = get_execution_phases()
                    self._save_execution_timeline_visualization(timeline, actual_total_time, phases)
                except Exception as e:
                    rospy.logwarn(f"[性能可视化] 生成可视化图表失败: {e}")
                    import traceback
                    traceback.print_exc()

            # 输出传统节点统计作为补充
            if self.node_execution_times:
                self._print_node_performance_summary()

        except Exception as e:
            import traceback
            traceback.print_exc()
            rospy.logwarn(f"[执行流程分析] 完整分析失败: {e}")
            # 降级到基础统计
            if self.node_execution_times:
                self._print_node_performance_summary()

    def _print_execution_phases(self, phases):
        """打印执行阶段分析"""
        if not phases:
            return

        # 过滤出完成的阶段
        completed_phases = [p for p in phases if 'duration' in p and 'start_time' in p]

        if not completed_phases:
            rospy.loginfo("\n[执行阶段时间线] 未收集到完成的执行阶段数据")
            return

        # 按开始时间排序
        completed_phases.sort(key=lambda x: x['start_time'])

        # 计算总持续时间（从第一个阶段开始到最后一个阶段结束）
        if len(completed_phases) > 0:
            first_start = completed_phases[0]['start_time']
            last_end = max(p['start_time'] + p['duration'] for p in completed_phases)
            total_duration = last_end - first_start
        else:
            total_duration = sum(p['duration'] for p in completed_phases)

        rospy.loginfo("\n[执行阶段时间线]")
        if total_duration <= 0:
            rospy.logwarn("  总持续时间为0，无法显示时间线")
            return

        for phase in completed_phases:
            phase_start = phase['start_time'] - first_start
            duration = phase['duration']
            percentage = (duration / total_duration * 100) if total_duration > 0 else 0

            # 创建时间轴可视化
            timeline_bar = self._create_timeline_bar(0, phase_start, duration, total_duration)

            if "TICK_" in phase['phase_name']:
                # Tick阶段特殊处理
                tick_num = phase['phase_name'].replace("TICK_", "")
                rospy.loginfo(f"{phase_start:6.2f}s {timeline_bar}  Tick_{tick_num} ({percentage:5.2f}%)")
            else:
                # 其他阶段
                rospy.loginfo(f"{phase_start:6.2f}s {timeline_bar}  {phase['phase_name']} ({percentage:5.2f}%)")

    def _print_execution_timeline(self, timeline, total_duration):
        """打印执行时间线详情，显示每个节点的执行情况"""
        if not timeline:
            return

        # 按时间排序
        sorted_timeline = sorted(timeline, key=lambda x: x['timestamp'])

        # 找到第一个节点执行事件作为时间基准（确保时间从0开始）
        first_node_start = None
        for event in sorted_timeline:
            if event['event_type'] == 'NODE_START':
                first_node_start = event
                break

        # 如果没有节点事件，使用 EXECUTION_START
        if first_node_start is None:
            start_events = [e for e in sorted_timeline if e['event_type'] == 'EXECUTION_START']
            if not start_events:
                rospy.logwarn("[节点执行时间线] 未找到执行开始事件")
                return
            time_base = start_events[0]['relative_time']
        else:
            time_base = first_node_start['relative_time']

        # 收集所有节点执行事件（只显示 NODE_START 和 NODE_END）
        node_events = []
        node_stats = defaultdict(lambda: {'duration': 0, 'count': 0, 'executions': []})
        current_executions = {}  # node_name -> start_event

        for event in sorted_timeline:
            if event['event_type'] == 'NODE_START':
                node_name = event['node_name']
                current_executions[node_name] = event
                node_events.append(event)
            elif event['event_type'] == 'NODE_END':
                node_name = event['node_name']
                if node_name in current_executions:
                    start_event = current_executions[node_name]
                    duration = event['relative_time'] - start_event['relative_time']
                    node_stats[node_name]['duration'] += duration
                    node_stats[node_name]['count'] += 1
                    node_stats[node_name]['executions'].append({
                        'start': start_event['relative_time'] - time_base,
                        'end': event['relative_time'] - time_base,
                        'duration': duration,
                        'status': event.get('status', 'COMPLETED')
                    })
                    node_events.append(event)
                    del current_executions[node_name]

        # 补充统计initialise方法的时间（时间线事件只记录update方法）
        # 从node_execution_times中获取initialise方法的时间并加到节点总时间中
        if self.node_execution_times:
            for full_name, times in self.node_execution_times.items():
                if '.' in full_name:
                    node_name, method_name = full_name.rsplit('.', 1)
                    if method_name == 'initialise':
                        # 将initialise方法的时间加到节点总时间中
                        init_total_time = sum(times)
                        node_stats[node_name]['duration'] += init_total_time
                        # 如果节点没有update方法（没有时间线事件），确保节点被统计
                        # initialise通常只执行一次，但如果有多次，也要计入count
                        # 只增加时间，不增加count
                    elif method_name == 'update':
                        # 如果节点只有update方法但没有时间线事件
                        if node_name not in node_stats or node_stats[node_name]['duration'] == 0:
                            # 如果节点没有时间线事件，但从node_execution_times中有数据
                            update_total_time = sum(times)
                            node_stats[node_name]['duration'] += update_total_time
                            node_stats[node_name]['count'] += len(times)
                else:
                    # 如果没有方法名分隔符，直接使用节点名
                    # 兼容性处理
                    node_name = full_name
                    total_time = sum(times)
                    node_stats[node_name]['duration'] += total_time
                    node_stats[node_name]['count'] += len(times)

        # 修正时间范围：对于有 node_method_times 的节点，如果 timeline 事件的时间范围明显小于实际执行时间，使用实际执行时间修正
        node_method_times = defaultdict(lambda: {'initialise': [], 'update': []})
        if self.node_execution_times:
            for full_name, times in self.node_execution_times.items():
                if '.' in full_name:
                    node_name, method_name = full_name.rsplit('.', 1)
                    if method_name in ['initialise', 'update']:
                        node_method_times[node_name][method_name].extend(times)

        for node_name, stats in node_stats.items():
            if node_name in node_method_times:
                init_times = node_method_times[node_name]['initialise']
                update_times = node_method_times[node_name]['update']
                actual_total_time = sum(init_times) + sum(update_times)

                # 计算 timeline 事件的总时间
                timeline_total_time = sum(e['duration'] for e in stats['executions'])

                # 如果 timeline 时间明显小于实际时间（小于10%），修正时间范围
                if actual_total_time > 0 and timeline_total_time < actual_total_time * 0.1:
                    rospy.loginfo(f"[节点执行时间线] 修正 '{node_name}' 的时间范围: timeline={timeline_total_time*1000:.2f}ms, actual={actual_total_time*1000:.2f}ms")

                    # 重新构建 executions 列表，使用实际执行时间
                    corrected_executions = []
                    current_time = stats['executions'][0]['start'] if stats['executions'] else 0

                    # 添加 initialise 周期
                    for init_time in init_times:
                        if init_time > 0:
                            corrected_executions.append({
                                'start': current_time,
                                'end': current_time + init_time,
                                'duration': init_time,
                                'status': 'COMPLETED'
                            })
                            current_time += init_time

                    # 添加 update 周期（尽量保持 timeline 事件中的状态）
                    for i, update_time in enumerate(update_times):
                        if update_time > 0:
                            # 尝试从 timeline 事件中获取状态
                            status = 'COMPLETED'
                            if i < len(stats['executions']):
                                status = stats['executions'][i].get('status', 'COMPLETED')
                            corrected_executions.append({
                                'start': current_time,
                                'end': current_time + update_time,
                                'duration': update_time,
                                'status': status
                            })
                            current_time += update_time

                    if corrected_executions:
                        stats['executions'] = corrected_executions
                        # 更新总时间（已经包含了 initialise 和 update 的时间）
                        stats['duration'] = actual_total_time

        if not node_stats:
            rospy.loginfo("\n[节点执行时间线] 未收集到节点执行事件")
            return

        # 显示节点执行时间线
        rospy.loginfo("\n[节点执行时间线]")
        rospy.loginfo("=" * 80)

        # 按节点分组，显示每个节点的所有执行
        node_execution_map = defaultdict(list)
        for event in node_events:
            if event['event_type'] == 'NODE_START':
                node_execution_map[event['node_name']].append(('START', event))
            elif event['event_type'] == 'NODE_END':
                node_execution_map[event['node_name']].append(('END', event))

        # 按节点总执行时间排序（现在包含了initialise和update的总时间）
        sorted_nodes = sorted(
            node_stats.items(),
            key=lambda x: x[1]['duration'],
            reverse=True
        )

        # 显示所有节点
        for node_name, stats in sorted_nodes:
            total_time = stats['duration']
            call_count = stats['count']
            percentage = (total_time / total_duration * 100) if total_duration > 0 else 0

            rospy.loginfo(f"\n{node_name} (调用{call_count}次, 总计{total_time*1000:.2f}ms, 占比{percentage:.1f}%):")

            # 显示每次执行的详细信息（合并连续的RUNNING状态）
            self._print_executions_with_compression(stats['executions'], total_duration)

        # 显示节点执行统计摘要
        if sorted_nodes:
            total_node_time = sum(stats['duration'] for _, stats in sorted_nodes)
            rospy.loginfo("\n[节点执行统计摘要]")
            # 说明：占比基于完整执行时间计算，节点总时间可能小于完整执行时间（因为包含tick间隔、框架开销等）
            if total_node_time > 0 and total_duration > 0:
                node_time_percentage = (total_node_time / total_duration * 100)
                rospy.loginfo(f"  节点总执行时间: {total_node_time*1000:.2f}ms ({node_time_percentage:.1f}% 完整执行时间)")
            for node_name, stats in sorted_nodes:
                # 占比基于完整执行时间
                percentage = (stats['duration'] / total_duration * 100) if total_duration > 0 else 0
                avg_duration = stats['duration'] / stats['count'] if stats['count'] > 0 else 0

                suggestion = ""
                if stats['count'] > 50:
                    suggestion = " [高频调用]"
                elif stats['duration'] > 1.0:
                    suggestion = " [执行较慢]"

                rospy.loginfo(f"  {node_name:25}: 总计={stats['duration']*1000:7.2f}ms, 调用={stats['count']:3d}次, 占比={percentage:5.1f}%, 平均={avg_duration*1000:6.2f}ms{suggestion}")

    def _print_executions_with_compression(self, executions, total_duration):
        """打印执行详情，合并连续的RUNNING状态"""
        if not executions:
            return

        # 合并连续的RUNNING状态
        i = 0
        while i < len(executions):
            exec_info = executions[i]
            start_time = exec_info['start']
            duration = exec_info['duration']
            status = exec_info['status']

            # 如果是RUNNING状态，检查后面有多少连续的RUNNING
            if status == 'RUNNING':
                running_start_idx = i
                running_count = 1
                running_total_duration = duration

                # 统计连续的RUNNING
                while i + 1 < len(executions) and executions[i + 1]['status'] == 'RUNNING':
                    i += 1
                    running_count += 1
                    running_total_duration += executions[i]['duration']

                # 如果RUNNING数量较少（<=5），全部显示
                if running_count <= 5:
                    for j in range(running_start_idx, i + 1):
                        exec_info = executions[j]
                        timeline_bar = self._create_timeline_bar(0, exec_info['start'], exec_info['duration'], total_duration)
                        rospy.loginfo(f"  执行{j+1}: {exec_info['start']:6.2f}s {timeline_bar}  耗时={exec_info['duration']*1000:6.2f}ms  状态={exec_info['status']}")
                else:
                    # 显示第一个RUNNING
                    first_exec = executions[running_start_idx]
                    timeline_bar = self._create_timeline_bar(0, first_exec['start'], first_exec['duration'], total_duration)
                    rospy.loginfo(f"  执行{running_start_idx+1}: {first_exec['start']:6.2f}s {timeline_bar}  耗时={first_exec['duration']*1000:6.2f}ms  状态={first_exec['status']}")

                    # 显示中间的几个RUNNING（显示1-2个中间样本）
                    displayed_mid_count = 0
                    if running_count > 2:
                        step = max(1, running_count // 3)  # 显示中间的几个
                        for j in range(running_start_idx + step, i, step):
                            if displayed_mid_count >= 2:  # 最多显示2个中间样本
                                break
                            exec_info = executions[j]
                            timeline_bar = self._create_timeline_bar(0, exec_info['start'], exec_info['duration'], total_duration)
                            rospy.loginfo(f"  执行{j+1}: {exec_info['start']:6.2f}s {timeline_bar}  耗时={exec_info['duration']*1000:6.2f}ms  状态={exec_info['status']}")
                            displayed_mid_count += 1

                    # 显示最后一个RUNNING（如果不是第一个）
                    if i > running_start_idx:
                        last_exec = executions[i]
                        timeline_bar = self._create_timeline_bar(0, last_exec['start'], last_exec['duration'], total_duration)
                        rospy.loginfo(f"  执行{i+1}: {last_exec['start']:6.2f}s {timeline_bar}  耗时={last_exec['duration']*1000:6.2f}ms  状态={last_exec['status']}")

                    # 显示省略信息
                    displayed_count = 1 + displayed_mid_count + (1 if i > running_start_idx else 0)  # 第一个 + 中间样本 + 最后一个
                    skipped = running_count - displayed_count
                    if skipped > 0:
                        rospy.loginfo(f"  ... (省略{skipped}个RUNNING状态, 总耗时={running_total_duration*1000:.2f}ms)")
            else:
                # 非RUNNING状态（SUCCESS/FAILURE等）全部显示
                timeline_bar = self._create_timeline_bar(0, start_time, duration, total_duration)
                rospy.loginfo(f"  执行{i+1}: {start_time:6.2f}s {timeline_bar}  耗时={duration*1000:6.2f}ms  状态={status}")

            i += 1

    def _create_timeline_bar(self, current_pos, phase_start, duration, total_duration, width=40):
        """创建时间轴可视化条"""
        if total_duration <= 0:
            return ""

        # 计算位置
        start_pos = int((phase_start / total_duration) * width)
        bar_length = max(1, int((duration / total_duration) * width))

        # 创建条形图
        bar = ['─'] * width
        for i in range(start_pos, min(start_pos + bar_length, width)):
            bar[i] = '█'

        return ''.join(bar)

    def _save_execution_timeline_visualization(self, timeline, total_duration, phases=None):
        """Generate and save execution timeline visualization (Gantt chart)"""
        if not MATPLOTLIB_AVAILABLE:
            return

        # Parse timeline to extract node execution periods
        sorted_timeline = sorted(timeline, key=lambda x: x['timestamp'])

        # Find time base
        time_base = None
        execution_start_time = None
        for event in sorted_timeline:
            if event['event_type'] == 'EXECUTION_START':
                time_base = event['relative_time']
                execution_start_time = event.get('timestamp', 0)
                rospy.loginfo(f"[Performance Visualization] Found EXECUTION_START: time_base={time_base}, timestamp={execution_start_time}")
                break
        if time_base is None:
            time_base = 0
            execution_start_time = sorted_timeline[0]['timestamp'] if sorted_timeline else 0
            rospy.logwarn(f"[Performance Visualization] No EXECUTION_START found, using time_base=0, execution_start_time={execution_start_time}")

        # Collect all node execution periods from timeline events (update and initialise methods)
        node_executions = defaultdict(list)  # node_name -> [(start, end, status, method), ...]
        # Use stacks to handle multiple concurrent executions of the same node
        current_executions = defaultdict(list)  # node_name -> [start_event, ...] (stack)
        current_methods = defaultdict(list)  # node_name -> [start_event, ...] (stack)

        for event in sorted_timeline:
            if event['event_type'] == 'NODE_START':
                node_name = event['node_name']
                # Debug: Log NODE_START for problematic nodes
                if node_name == 'PrepareArmForGrasp':
                    rospy.loginfo(f"[Performance Visualization] NODE_START for '{node_name}': "
                                f"timestamp={event.get('timestamp', 0):.6f}, "
                                f"relative_time={event.get('relative_time', 0):.6f}, "
                                f"time_base={time_base:.6f}")
                # Push to stack to handle multiple concurrent executions
                current_executions[node_name].append(event)
            elif event['event_type'] == 'NODE_END':
                node_name = event['node_name']
                # Pop from stack (LIFO - last in first out, matches nested/parallel execution)
                if current_executions[node_name]:
                    start_event = current_executions[node_name].pop()
                    start_time = start_event['relative_time'] - time_base
                    # Use duration from event if available (more accurate), otherwise calculate from relative_time
                    event_duration = event.get('duration')
                    # Debug: Always log for PrepareArmForGrasp to diagnose the issue
                    if node_name == 'PrepareArmForGrasp':
                        rospy.loginfo(f"[Performance Visualization] NODE_END for '{node_name}': "
                                    f"event keys={list(event.keys())}, "
                                    f"event duration={event_duration}, "
                                    f"start_event relative_time={start_event['relative_time']:.6f}, "
                                    f"end_event relative_time={event['relative_time']:.6f}, "
                                    f"time_base={time_base:.6f}")

                    # Always prefer event duration if available (even if small), as it's more accurate
                    # Only fall back to relative_time if duration is None
                    if event_duration is not None:
                        # Use the actual duration recorded in the event
                        end_time = start_time + event_duration
                        if node_name == 'PrepareArmForGrasp':
                            rospy.loginfo(f"[Performance Visualization] Using event duration: {event_duration:.6f}s, "
                                        f"start={start_time:.6f}s, end={end_time:.6f}s, final_duration={end_time-start_time:.6f}s")
                    else:
                        # Fall back to relative_time difference (may be inaccurate for parallel nodes)
                        end_time = event['relative_time'] - time_base
                        if node_name == 'PrepareArmForGrasp':
                            rospy.logwarn(f"[Performance Visualization] Event duration not available (None), using relative_time difference: "
                                        f"start={start_time:.6f}s, end={end_time:.6f}s, duration={end_time-start_time:.6f}s")
                    status = event.get('status', 'COMPLETED')
                    node_executions[node_name].append((start_time, end_time, status, 'update'))
                else:
                    # If no matching START found, log warning but continue
                    rospy.logwarn(f"[Performance Visualization] NODE_END for '{node_name}' without matching NODE_START")
            elif event['event_type'] == 'METHOD_START':
                # Record initialise method start
                node_name = event['node_name']
                # Push to stack to handle multiple concurrent executions
                current_methods[node_name].append(event)
            elif event['event_type'] == 'METHOD_END':
                # Record initialise method end
                node_name = event['node_name']
                # Pop from stack (LIFO)
                if current_methods[node_name]:
                    start_event = current_methods[node_name].pop()
                    start_time = start_event['relative_time'] - time_base
                    # Use duration from event if available (more accurate), otherwise calculate from relative_time
                    event_duration = event.get('duration')
                    # Always prefer event duration if available (even if small), as it's more accurate
                    if event_duration is not None:
                        # Use the actual duration recorded in the event
                        end_time = start_time + event_duration
                    else:
                        # Fall back to relative_time difference (may be inaccurate for parallel nodes)
                        end_time = event['relative_time'] - time_base
                    status = event.get('status', 'COMPLETED')
                    node_executions[node_name].append((start_time, end_time, status, 'initialise'))
                else:
                    # If no matching START found, log warning but continue
                    rospy.logwarn(f"[Performance Visualization] METHOD_END for '{node_name}' without matching METHOD_START")

        # Log any unmatched START events (shouldn't happen in normal execution)
        for node_name, start_events in current_executions.items():
            if start_events:
                rospy.logwarn(f"[Performance Visualization] Found {len(start_events)} unmatched NODE_START events for '{node_name}'")
        for node_name, start_events in current_methods.items():
            if start_events:
                rospy.logwarn(f"[Performance Visualization] Found {len(start_events)} unmatched METHOD_START events for '{node_name}'")

        # Debug: Count timeline events for problematic nodes
        for node_name in ['PrepareArmForGrasp', 'MoveToTarget', 'PrepareCalcLeg', 'CalcLeg']:
            node_start_events = [e for e in sorted_timeline if e.get('event_type') == 'NODE_START' and e.get('node_name') == node_name]
            node_end_events = [e for e in sorted_timeline if e.get('event_type') == 'NODE_END' and e.get('node_name') == node_name]
            method_start_events = [e for e in sorted_timeline if e.get('event_type') == 'METHOD_START' and e.get('node_name') == node_name]
            method_end_events = [e for e in sorted_timeline if e.get('event_type') == 'METHOD_END' and e.get('node_name') == node_name]
            if node_start_events or node_end_events or method_start_events or method_end_events:
                rospy.loginfo(f"[Performance Visualization] Timeline events for '{node_name}': "
                            f"NODE_START={len(node_start_events)}, NODE_END={len(node_end_events)}, "
                            f"METHOD_START={len(method_start_events)}, METHOD_END={len(method_end_events)}")
                if node_start_events and node_end_events:
                    for i, (start, end) in enumerate(zip(node_start_events, node_end_events)):
                        duration = end.get('relative_time', 0) - start.get('relative_time', 0)
                        rospy.loginfo(f"  Execution {i+1}: start={start.get('relative_time', 0):.6f}s, "
                                    f"end={end.get('relative_time', 0):.6f}s, duration={duration:.6f}s")

        # Collect all nodes from node_execution_times (including those without timeline events)
        all_nodes_from_data = set()
        node_method_times = defaultdict(lambda: {'initialise': [], 'update': []})

        if self.node_execution_times:
            for full_name, times in self.node_execution_times.items():
                if '.' in full_name:
                    node_name, method_name = full_name.rsplit('.', 1)
                    all_nodes_from_data.add(node_name)
                    if method_name in ['initialise', 'update']:
                        node_method_times[node_name][method_name].extend(times)

        # Sort all executions by start time for each node (initialise and update are now both from timeline)
        for node_name in node_executions.keys():
            node_executions[node_name].sort(key=lambda x: x[0])

        # Fix time ranges for nodes with incorrect timeline event timestamps
        # For nodes with node_method_times, use actual execution time to correct timeline ranges
        for node_name, executions in node_executions.items():
            if node_name in node_method_times:
                init_times = node_method_times[node_name]['initialise']
                update_times = node_method_times[node_name]['update']
                actual_total_time = sum(init_times) + sum(update_times)

                # Calculate timeline total time
                timeline_total_time = sum(e[1] - e[0] for e in executions)

                # If timeline time is much smaller than actual time (likely incorrect timestamps)
                # Adjust the timeline execution periods to match actual execution time
                if actual_total_time > 0 and timeline_total_time < actual_total_time * 0.1:
                    rospy.loginfo(f"[Performance Visualization] Correcting time range for '{node_name}': "
                                f"timeline={timeline_total_time*1000:.2f}ms, actual={actual_total_time*1000:.2f}ms")

                    # Find the earliest start time and adjust all executions
                    if executions:
                        earliest_start = min(e[0] for e in executions)
                        corrected_executions = []
                        current_time = earliest_start

                        # Add initialise periods
                        for init_time in init_times:
                            if init_time > 0:
                                corrected_executions.append((current_time, current_time + init_time, 'COMPLETED', 'initialise'))
                                current_time += init_time

                        # Add update periods (use timeline status if available)
                        for i, update_time in enumerate(update_times):
                            if update_time > 0:
                                # Try to preserve status from timeline if available
                                status = 'COMPLETED'
                                if i < len(executions):
                                    status = executions[i][2] if len(executions[i]) > 2 else 'COMPLETED'
                                corrected_executions.append((current_time, current_time + update_time, status, 'update'))
                                current_time += update_time

                        if corrected_executions:
                            node_executions[node_name] = corrected_executions
                            rospy.loginfo(f"[Performance Visualization] Corrected '{node_name}': "
                                        f"{len(corrected_executions)} periods, total={sum(e[1]-e[0] for e in corrected_executions)*1000:.2f}ms")

        # Check for nodes that have execution times but no timeline events
        # These nodes might have been executed but didn't record timeline events
        nodes_without_timeline = all_nodes_from_data - set(node_executions.keys())
        if nodes_without_timeline:
            rospy.logwarn(f"[Performance Visualization] Nodes with execution times but no timeline events: {nodes_without_timeline}")
            rospy.logwarn("  These nodes may not be using @performance_monitor decorator on update method, or update executes too quickly")
            # Log their execution times for debugging
            for node_name in nodes_without_timeline:
                init_times = node_method_times[node_name]['initialise']
                update_times = node_method_times[node_name]['update']
                total_time = sum(init_times) + sum(update_times)
                rospy.logwarn(f"    {node_name}: initialise={len(init_times)} calls ({sum(init_times)*1000:.2f}ms), update={len(update_times)} calls ({sum(update_times)*1000:.2f}ms), total={total_time*1000:.2f}ms")

        # Add TICK phases as a special category
        tick_executions = []
        if phases and self.current_execution_start_time:
            for phase in phases:
                if 'TICK_' in phase.get('phase_name', ''):
                    if 'start_time' in phase and 'end_time' in phase:
                        # Convert absolute timestamps to relative time
                        start_time = phase['start_time'] - self.current_execution_start_time
                        end_time = phase['end_time'] - self.current_execution_start_time
                        # Ensure times are within valid range
                        start_time = max(0, start_time)
                        end_time = min(total_duration, end_time)
                        if start_time < end_time and end_time <= total_duration:
                            tick_executions.append((start_time, end_time))

        if not node_executions and not tick_executions:
            rospy.logwarn("[Performance Visualization] No node execution data found")
            return

        # Debug: Log node execution statistics with time ranges
        rospy.loginfo(f"[Performance Visualization] Found {len(node_executions)} nodes with execution data")
        rospy.loginfo(f"[Performance Visualization] Total nodes in execution_times: {len(all_nodes_from_data)}")
        rospy.loginfo(f"[Performance Visualization] Nodes with timeline events: {set(node_executions.keys())}")
        rospy.loginfo(f"[Performance Visualization] Nodes in execution_times: {all_nodes_from_data}")

        for node_name, executions in node_executions.items():
            # Use actual method execution time from node_method_times if available (more accurate)
            # Otherwise fall back to timeline event time range
            if node_name in node_method_times:
                init_times = node_method_times[node_name]['initialise']
                update_times = node_method_times[node_name]['update']
                total_time = sum(init_times) + sum(update_times)
            else:
                total_time = sum(e[1] - e[0] for e in executions)

            if executions:
                min_start = min(e[0] for e in executions)
                max_end = max(e[1] for e in executions)
                rospy.loginfo(f"  {node_name}: {len(executions)} timeline executions, total {total_time*1000:.2f}ms, time range [{min_start:.3f}s, {max_end:.3f}s]")
                # Also log method execution counts for comparison
                if node_name in node_method_times:
                    init_count = len(node_method_times[node_name]['initialise'])
                    update_count = len(node_method_times[node_name]['update'])
                    rospy.loginfo(f"    Method calls: initialise={init_count}, update={update_count}")
            else:
                rospy.loginfo(f"  {node_name}: {len(executions)} executions, total {total_time*1000:.2f}ms")

        # Merge consecutive RUNNING states for each node
        merged_node_executions = {}
        for node_name, executions in node_executions.items():
            if not executions:
                continue

            # Sort executions by start time
            sorted_execs = sorted(executions, key=lambda x: x[0])
            merged = []

            i = 0
            while i < len(sorted_execs):
                start_time, end_time, status, method = sorted_execs[i]

                # If this is RUNNING, try to merge with consecutive RUNNING states
                if status == 'RUNNING':
                    merged_start = start_time
                    merged_end = end_time
                    merged_count = 1

                    # Merge consecutive RUNNING states
                    j = i + 1
                    while j < len(sorted_execs):
                        next_start, next_end, next_status, next_method = sorted_execs[j]
                        # Only merge if status is RUNNING and times are close (within 0.1s gap)
                        if next_status == 'RUNNING' and next_start - merged_end < 0.1:
                            merged_end = max(merged_end, next_end)
                            merged_count += 1
                            j += 1
                        else:
                            break

                    # Add merged RUNNING period
                    if merged_count > 1:
                        merged.append((merged_start, merged_end, f'RUNNING ({merged_count} calls)', 'update'))
                    else:
                        merged.append((merged_start, merged_end, status, method))

                    i = j
                else:
                    # Non-RUNNING states are kept as-is
                    merged.append((start_time, end_time, status, method))
                    i += 1

            if merged:
                merged_node_executions[node_name] = merged

        # Prepare plotting data - only nodes, no TICK phases row
        all_items = []

        # Add node executions
        nodes = list(merged_node_executions.keys())
        # Sort by total execution time (use actual method execution time if available)
        def get_node_total_time(node_name):
            if node_name in node_method_times:
                init_times = node_method_times[node_name]['initialise']
                update_times = node_method_times[node_name]['update']
                return sum(init_times) + sum(update_times)
            else:
                return sum(e[1] - e[0] for e in merged_node_executions[node_name])

        nodes.sort(key=lambda n: get_node_total_time(n), reverse=True)

        # Filter out nodes with zero total time (they won't be visible anyway)
        # Use a very small threshold (0.0001s = 0.1ms) to include almost all nodes
        filtered_nodes = []
        filtered_out = []
        for n in nodes:
            total_time = get_node_total_time(n)
            if total_time > 0.0001:  # > 0.1ms (very small threshold to include almost all nodes)
                filtered_nodes.append(n)
            else:
                filtered_out.append((n, total_time*1000))

        if filtered_out:
            rospy.loginfo(f"[Performance Visualization] Filtered out {len(filtered_out)} nodes with total time < 0.1ms:")
            for node_name, time_ms in filtered_out:
                rospy.loginfo(f"  {node_name}: {time_ms:.3f}ms")

        for node_name in filtered_nodes:
            all_items.append((node_name, merged_node_executions[node_name]))

        if not all_items:
            rospy.logwarn("[Performance Visualization] No data to plot")
            return

        # Create figure
        fig, ax = plt.subplots(figsize=(16, max(8, len(all_items) * 0.5)))

        # Color mapping: different statuses use different colors
        status_colors = {
            'RUNNING': '#3498db',  # Blue
            'SUCCESS': '#2ecc71',   # Green
            'FAILURE': '#e74c3c',   # Red
            'COMPLETED': '#95a5a6',  # Gray
            'TICK': '#f39c12'  # Orange for TICK phases
        }

        # Draw execution periods for each item
        # y_pos should match the order: first item (longest time) at top, last item at bottom
        # After invert_yaxis(), y_pos=0 will be at top, y_pos=len-1 will be at bottom
        y_positions = {}
        for i, (item_name, executions) in enumerate(all_items):
            y_pos = i  # First item (index 0, longest time) will be at top after invert_yaxis()
            y_positions[item_name] = y_pos

            # Draw node executions
            total_node_time = sum(e[1] - e[0] for e in executions)
            exec_count = len(executions)

            for exec_idx, exec_data in enumerate(executions):
                if len(exec_data) >= 4:
                    start_time, end_time, status, method = exec_data
                else:
                    start_time, end_time, status = exec_data[:3]
                    method = 'update'

                duration = end_time - start_time
                # Use status color
                color = status_colors.get(status.split('(')[0].strip() if '(' in status else status, '#95a5a6')
                # Make initialise methods slightly different (lighter)
                alpha = 0.6 if method == 'initialise' else 0.8
                ax.barh(y_pos, duration, left=start_time, height=0.6,
                       color=color, alpha=alpha, edgecolor='black', linewidth=0.8)

                # Show duration and status for periods longer than threshold
                if duration > total_duration * 0.005:  # If > 0.5% of total time
                    mid_time = start_time + duration / 2
                    # Show duration
                    duration_text = f"{duration*1000:.0f}ms"
                    # Choose text color based on background
                    text_color = 'white' if status.startswith('RUNNING') or status.startswith('FAILURE') else 'black'
                    ax.text(mid_time, y_pos, duration_text, ha='center', va='center',
                           fontsize=7, fontweight='bold', color=text_color)

        # Set y-axis labels (with statistics)
        y_labels = []
        for item_name, executions in all_items:
            # Use actual method execution time if available (more accurate)
            if item_name in node_method_times:
                init_times = node_method_times[item_name]['initialise']
                update_times = node_method_times[item_name]['update']
                total_node_time = sum(init_times) + sum(update_times)
            else:
                total_node_time = sum(e[1] - e[0] for e in executions)
            exec_count = len(executions)
            # Count RUNNING calls (including merged ones)
            running_count = sum(1 for e in executions if e[2].startswith('RUNNING'))
            y_labels.append(f"{item_name} ({exec_count} periods, {total_node_time*1000:.0f}ms)")

        ax.set_yticks(range(len(all_items)))
        ax.set_yticklabels(y_labels, fontsize=9)
        ax.invert_yaxis()  # After inversion, y_pos=0 (first item, longest time) will be at top

        # Set x-axis
        ax.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
        ax.set_xlim(0, total_duration)

        # Add x-axis time ticks
        max_ticks = 20
        tick_interval = max(0.1, total_duration / max_ticks)
        x_ticks = [i * tick_interval for i in range(int(total_duration / tick_interval) + 1)]
        x_ticks = [t for t in x_ticks if t <= total_duration]
        ax.set_xticks(x_ticks)
        ax.set_xticklabels([f"{t:.2f}" for t in x_ticks], fontsize=8, rotation=45)

        # Add grid
        ax.grid(True, alpha=0.3, linestyle='--', axis='x')
        ax.set_axisbelow(True)

        # Add title
        iteration = self.current_iteration if hasattr(self, 'current_iteration') else 0
        ax.set_title(f'Behavior Tree Execution Timeline - Iteration {iteration}\nTotal Execution Time: {total_duration:.3f}s ({total_duration*1000:.2f}ms)',
                    fontsize=14, fontweight='bold', pad=20)

        # Add legend
        legend_elements = [
            mpatches.Patch(facecolor=status_colors['RUNNING'], label='RUNNING', alpha=0.8),
            mpatches.Patch(facecolor=status_colors['SUCCESS'], label='SUCCESS', alpha=0.8),
            mpatches.Patch(facecolor=status_colors['FAILURE'], label='FAILURE', alpha=0.8),
            mpatches.Patch(facecolor=status_colors['COMPLETED'], label='COMPLETED (initialise)', alpha=0.6),
        ]
        ax.legend(handles=legend_elements, loc='upper right', fontsize=10)

        # 调整布局
        plt.tight_layout()

        # 保存图片
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        iteration = self.current_iteration if hasattr(self, 'current_iteration') else 0

        # 尝试多个可能的输出目录
        possible_dirs = [
            os.path.join(os.path.expanduser('~'), '.ros', 'bt_performance'),
            os.path.join('/tmp', 'bt_performance'),
            os.path.join(os.getcwd(), 'bt_performance')
        ]

        output_dir = None
        for dir_path in possible_dirs:
            try:
                os.makedirs(dir_path, exist_ok=True)
                # 测试写入权限
                test_file = os.path.join(dir_path, '.test_write')
                with open(test_file, 'w') as f:
                    f.write('test')
                os.remove(test_file)
                output_dir = dir_path
                break
            except (OSError, PermissionError):
                continue

        if output_dir is None:
            rospy.logwarn("[Performance Visualization] Cannot create output directory, skipping chart save")
            plt.close()
            return

        filename = os.path.join(output_dir, f'execution_timeline_iter{iteration}_{timestamp}.png')

        try:
            plt.savefig(filename, dpi=150, bbox_inches='tight', facecolor='white')
            plt.close()
            rospy.loginfo(f"[Performance Visualization] Timeline chart saved: {filename}")
        except Exception as e:
            rospy.logwarn(f"[Performance Visualization] Failed to save chart: {e}")
            plt.close()

    def _print_node_performance_summary(self):
        """打印节点性能摘要（详细版，每次执行完成时调用）"""
        # 输出循环耗时统计
        if self.total_execution_times:
            total_all_loops = sum(self.total_execution_times)
            avg_loop_time = total_all_loops / len(self.total_execution_times)
            max_loop_time = max(self.total_execution_times)
            min_loop_time = min(self.total_execution_times)

            rospy.loginfo("\n[完整执行统计]")
            rospy.loginfo(f"  已完成循环次数: {len(self.total_execution_times)}")
            rospy.loginfo(f"  平均每次循环: {avg_loop_time*1000:.2f}ms ({avg_loop_time:.3f}s)")
            rospy.loginfo(f"  最长循环: {max_loop_time*1000:.2f}ms ({max_loop_time:.3f}s)")
            rospy.loginfo(f"  最短循环: {min_loop_time*1000:.2f}ms ({min_loop_time:.3f}s)")

        if not self.node_execution_times:
            rospy.logwarn("[节点执行时间] 暂无数据")
            return

        # 计算节点总执行时间和调用统计
        node_total_times = defaultdict(float)
        node_call_counts = defaultdict(int)
        node_method_breakdown = defaultdict(lambda: {'initialise': [], 'update': []})

        for full_name, times in self.node_execution_times.items():
            if '.' in full_name:
                node_name, method_name = full_name.rsplit('.', 1)
                if method_name in ['initialise', 'update']:
                    node_method_breakdown[node_name][method_name].extend(times)
                    node_total_times[node_name] += sum(times)
                    node_call_counts[node_name] += len(times)

        # 按总执行时间排序
        sorted_nodes = sorted(node_total_times.items(), key=lambda x: x[1], reverse=True)

        rospy.loginfo("\n[节点执行时间统计 (按总时间排序)]")
        for node_name, total_time in sorted_nodes:
            methods = node_method_breakdown[node_name]
            call_count = node_call_counts[node_name]

            rospy.loginfo(f"  {node_name}:")
            rospy.loginfo(f"    总执行时间: {total_time*1000:.2f}ms, 调用次数: {call_count}")

            # initialise 统计
            if methods.get('initialise'):
                init_times = methods['initialise']
                init_total = sum(init_times)
                init_avg = init_total / len(init_times)
                init_max = max(init_times)
                init_min = min(init_times)
                rospy.loginfo(f"    initialise: 总计={init_total*1000:.2f}ms, 平均={init_avg*1000:.2f}ms, "
                            f"最大={init_max*1000:.2f}ms, 最小={init_min*1000:.2f}ms, "
                            f"次数={len(init_times)}")

            # update 统计
            if methods.get('update'):
                update_times = methods['update']
                update_total = sum(update_times)
                update_avg = update_total / len(update_times)
                update_max = max(update_times)
                update_min = min(update_times)
                rospy.loginfo(f"    update: 总计={update_total*1000:.2f}ms, 平均={update_avg*1000:.2f}ms, "
                            f"最大={update_max*1000:.2f}ms, 最小={update_min*1000:.2f}ms, "
                            f"次数={len(update_times)}")

        # 输出时间占比分析
        if sorted_nodes:
            total_node_time = sum(node_total_times.values())
            if total_node_time > 0:
                rospy.loginfo("\n[节点时间占比分析]")
                for node_name, node_time in sorted_nodes[:5]:  # 只显示前5个
                    percentage = (node_time / total_node_time) * 100
                    rospy.loginfo(f"  {node_name}: {percentage:.1f}%")
        rospy.loginfo("")  # 空行分隔

    def _print_performance_summary(self):
        """打印性能统计摘要（完整版）"""
        try:
            from node.performance_monitor import get_tick_performance_data
            tick_times = get_tick_performance_data()

            rospy.loginfo("=" * 70)
            rospy.loginfo("[性能统计] 行为树执行性能报告")
            rospy.loginfo("=" * 70)

            # 总执行次数统计
            if self.total_execution_times:
                total_count = len(self.total_execution_times)
                total_time = sum(self.total_execution_times)
                avg_time = total_time / total_count
                max_time = max(self.total_execution_times)
                min_time = min(self.total_execution_times)
                rospy.loginfo(f"\n[完整执行统计]")
                rospy.loginfo(f"  执行次数: {total_count}")
                rospy.loginfo(f"  总执行时间: {total_time*1000:.2f}ms ({total_time:.3f}s)")
                rospy.loginfo(f"  平均每次执行: {avg_time*1000:.2f}ms")
                rospy.loginfo(f"  最长执行: {max_time*1000:.2f}ms")
                rospy.loginfo(f"  最短执行: {min_time*1000:.2f}ms")

            # Tick 级别统计
            if tick_times:
                total_tick_time = sum(tick_times)
                avg_tick_time = total_tick_time / len(tick_times)
                max_tick_time = max(tick_times)
                min_tick_time = min(tick_times)
                tick_frequency = 1.0 / avg_tick_time if avg_tick_time > 0 else 0

                rospy.loginfo(f"\n[Tick 级别统计]")
                rospy.loginfo(f"  Tick 总数: {len(tick_times)}")
                rospy.loginfo(f"  Tick 总时间: {total_tick_time*1000:.2f}ms")
                rospy.loginfo(f"  平均 Tick 时间: {avg_tick_time*1000:.2f}ms")
                rospy.loginfo(f"  Tick 频率: {tick_frequency:.1f} Hz")
                rospy.loginfo(f"  最慢 Tick: {max_tick_time*1000:.2f}ms")
                rospy.loginfo(f"  最快 Tick: {min_tick_time*1000:.2f}ms")

            # 调用详细统计
            self._print_node_performance_summary()

        except Exception as e:
            rospy.logwarn(f"[性能统计] 完整报告生成失败，使用基础统计: {e}")
            # 降级到基础统计
            self._print_node_performance_summary()

        finally:
            rospy.loginfo("=" * 70)

if __name__ == "__main__":
    rospy.logerr("behavior_tree_controller.py 不应直接运行，请通过 BT_action.py 调用")