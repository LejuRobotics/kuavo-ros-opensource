"""
性能监控工具，用于记录行为树完整执行流程
支持记录行为树从开始到结束的所有阶段执行时间
"""
import time
from functools import wraps
from collections import defaultdict

# 全局性能数据存储
_node_performance_data = defaultdict(list)  # 节点名.方法名 -> [执行时间列表]
_execution_timeline = []  # 执行时间线：[(timestamp, event_type, node_name, duration, status)]
_execution_stack = []  # 执行栈：跟踪当前正在执行的节点
_execution_phases = []  # 执行阶段：[(phase_name, start_time, end_time, duration)]
_enable_performance_monitoring = False
_execution_start_time = None


def enable_performance_monitoring(enable=True):
    """启用/禁用性能监控"""
    global _enable_performance_monitoring, _execution_start_time
    _enable_performance_monitoring = enable
    if enable:
        _execution_start_time = time.time()
        _record_timeline_event("EXECUTION_START", "Root", 0, "STARTED")
    else:
        if _execution_start_time:
            total_duration = time.time() - _execution_start_time
            _record_timeline_event("EXECUTION_END", "Root", total_duration, "COMPLETED")


def reset_execution_start_time(start_time=None):
    """重置执行开始时间（用于在行为树真正开始执行时重置时间基准）

    Args:
        start_time: 指定的开始时间（如果为None，则使用当前时间）
    """
    global _execution_start_time
    if start_time is None:
        start_time = time.time()
    _execution_start_time = start_time
    # 清空之前的时间线事件，重新开始记录
    _execution_timeline.clear()
    _record_timeline_event("EXECUTION_START", "Root", 0, "STARTED")


def record_execution_end(status="COMPLETED"):
    """记录执行结束事件（不关闭监控）"""
    global _execution_start_time
    if _execution_start_time:
        total_duration = time.time() - _execution_start_time
        _record_timeline_event("EXECUTION_END", "Root", total_duration, status)


def _record_timeline_event(event_type, node_name, duration=None, status=None, parent=None):
    """记录时间线事件"""
    if not _enable_performance_monitoring:
        return

    timestamp = time.time()
    event = {
        'timestamp': timestamp,
        'event_type': event_type,
        'node_name': node_name,
        'duration': duration,
        'status': status,
        'parent': parent,
        'relative_time': timestamp - _execution_start_time if _execution_start_time else 0
    }
    _execution_timeline.append(event)


def get_performance_data():
    """获取性能数据"""
    return dict(_node_performance_data)


def get_execution_timeline():
    """获取执行时间线"""
    return list(_execution_timeline)


def start_node_execution(node_name, parent=None):
    """开始记录节点执行"""
    if not _enable_performance_monitoring:
        return

    _execution_stack.append({
        'node_name': node_name,
        'start_time': time.time(),
        'parent': parent
    })
    _record_timeline_event("NODE_START", node_name, 0, "RUNNING", parent)


def end_node_execution(node_name, status="COMPLETED"):
    """结束记录节点执行"""
    if not _enable_performance_monitoring or not _execution_stack:
        return

    # 找到对应的开始记录
    for i in range(len(_execution_stack) - 1, -1, -1):
        if _execution_stack[i]['node_name'] == node_name:
            start_time = _execution_stack[i]['start_time']
            parent = _execution_stack[i]['parent']
            duration = time.time() - start_time
            _record_timeline_event("NODE_END", node_name, duration, status, parent)
            _execution_stack.pop(i)
            break


def record_phase_start(phase_name):
    """记录执行阶段开始"""
    if not _enable_performance_monitoring:
        return

    _execution_phases.append({
        'phase_name': phase_name,
        'start_time': time.time()
    })


def record_phase_end(phase_name):
    """记录执行阶段结束"""
    if not _enable_performance_monitoring:
        return

    for phase in reversed(_execution_phases):
        if phase['phase_name'] == phase_name and 'end_time' not in phase:
            phase['end_time'] = time.time()
            phase['duration'] = phase['end_time'] - phase['start_time']
            break


def get_execution_phases():
    """获取执行阶段数据"""
    return list(_execution_phases)


def clear_performance_data():
    """清空性能数据"""
    global _node_performance_data, _execution_timeline, _execution_stack, _execution_phases, _execution_start_time
    _node_performance_data.clear()
    _execution_timeline.clear()
    _execution_stack.clear()
    _execution_phases.clear()
    _execution_start_time = None


def get_node_performance_summary():
    """获取节点性能摘要（按节点名聚合）"""
    node_summary = defaultdict(lambda: {'initialise': [], 'update': []})

    for full_name, times in _node_performance_data.items():
        if '.' in full_name:
            node_name, method_name = full_name.rsplit('.', 1)
            if method_name in ['initialise', 'update']:
                node_summary[node_name][method_name].extend(times)
        else:
            # 如果没有方法名，默认归类到 update
            node_summary[full_name]['update'].extend(times)

    return dict(node_summary)


def performance_monitor(node_name=None, method_name=None):
    """
    性能监控装饰器，用于记录节点执行时间和时间线

    使用方法：
        @performance_monitor("节点名称", "update")
        def update(self):
            # 节点执行逻辑
            pass

        @performance_monitor("节点名称", "initialise")
        def initialise(self):
            # 初始化逻辑
            pass
    """
    def decorator(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if not _enable_performance_monitoring:
                return func(self, *args, **kwargs)

            # 获取节点名称和方法名称
            base_name = node_name if node_name else getattr(self, 'name', func.__name__)
            method = method_name if method_name else func.__name__
            # 组合名称：节点名_方法名
            full_name = f"{base_name}.{method}"

            # 对于 update 方法，记录节点级别的执行时间线
            # initialise 方法也记录时间线事件，但使用METHOD_START/METHOD_END类型
            record_node_timeline = (method == "update")
            record_method_timeline = (method == "initialise")  # Also record initialise timeline

            if record_node_timeline:
                start_node_execution(base_name)

            # Record method start for initialise
            method_start_time = None
            if record_method_timeline:
                method_start_time = time.time()
                _record_timeline_event("METHOD_START", base_name, 0, "RUNNING", None)

            # 记录方法级别的开始执行
            start_time = time.time()
            result = None
            node_status = "COMPLETED"
            try:
                result = func(self, *args, **kwargs)

                # 根据返回值判断节点状态
                if record_node_timeline:
                    if hasattr(result, 'value'):
                        node_status = str(result).split('.')[-1]  # 提取状态名，如 SUCCESS, FAILURE, RUNNING
                    elif result is None:
                        node_status = "RUNNING"
                    else:
                        node_status = "COMPLETED"

                return result
            except Exception as e:
                if record_node_timeline:
                    node_status = "FAILED"
                raise e
            finally:
                end_time = time.time()
                execution_time = end_time - start_time
                _node_performance_data[full_name].append(execution_time)

                # 记录节点执行结束
                if record_node_timeline:
                    end_node_execution(base_name, node_status)

                # Record method end for initialise
                if record_method_timeline and method_start_time:
                    method_duration = time.time() - method_start_time
                    _record_timeline_event("METHOD_END", base_name, method_duration, "COMPLETED", None)

        return wrapper
    return decorator


def tree_node_monitor(node_name=None):
    """
    行为树节点监控装饰器，用于记录节点的完整生命周期

    使用方法：
        @tree_node_monitor("节点名称")
        def update(self):
            # 节点执行逻辑
            pass
    """
    def decorator(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if not _enable_performance_monitoring:
                return func(self, *args, **kwargs)

            # 获取节点名称
            name = node_name if node_name else getattr(self, 'name', func.__name__)

            # 记录节点开始执行
            start_node_execution(name)

            try:
                result = func(self, *args, **kwargs)
                # 根据返回值判断节点状态
                if hasattr(result, 'value'):
                    status = str(result).split('.')[-1]  # 提取状态名
                else:
                    status = "COMPLETED"
                end_node_execution(name, status)
                return result
            except Exception as e:
                end_node_execution(name, "FAILED")
                raise e

        return wrapper
    return decorator

