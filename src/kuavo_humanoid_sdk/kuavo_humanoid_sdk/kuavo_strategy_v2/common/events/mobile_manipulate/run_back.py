import threading
import time
from typing import Callable, Any, List, Tuple, Dict, Optional
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import BaseEvent, EventStatus

class ParallelTaskExecutor:
    """
    并行任务执行器类
    
    负责管理和执行多个并行任务，支持任务添加、启动、状态监控和结果获取。
    """
    
    def __init__(self, 
                 max_workers: int = 10,
                 default_timeout: float = 60.0,
                 auto_reset: bool = True):
        """
        初始化并行任务执行器
        
        参数:
        max_workers (int): 最大并发任务数
        default_timeout (float): 默认任务超时时间(秒)
        auto_reset (bool): 任务完成后是否自动重置
        """
        self.max_workers = max_workers
        self.default_timeout = default_timeout
        self.auto_reset = auto_reset
        
        # 成员变量
        self.tasks: Dict[str, TaskInfo] = {}  # 任务字典
        self.active_tasks: List[str] = []      # 当前活动任务列表
        self.task_counter = 0                  # 任务计数器
        
        # 线程同步
        self.lock = threading.RLock()
        self.task_complete_event = threading.Event()
    
    def reset(self):
        """
        重置执行器状态，清除所有任务
        """
        with self.lock:
            self.tasks.clear()
            self.active_tasks.clear()
            self.task_counter = 0
            self.task_complete_event.clear()
    
    def add_task(self,
                 name: str,
                 func: Callable[..., Any],
                 *args,
                 **kwargs) -> str:
        """
        添加一个新任务
        
        参数:
        name (str): 任务名称
        func (Callable): 要执行的函数
        args: 函数位置参数
        kwargs: 函数关键字参数
        
        返回:
        str: 任务ID
        """
        with self.lock:
            task_id = f"task-{self.task_counter}"
            self.task_counter += 1
            
            self.tasks[task_id] = {
                'id': task_id,
                'name': name,
                'func': func,
                'args': args,
                'kwargs': kwargs,
                'status': 'pending',  # pending, running, completed, failed
                'result': None,
                'exception': None,
                'thread': None,
                'start_time': None,
                'end_time': None
            }
            return task_id
    
    def start_task(self, task_id: str):
        """
        启动指定任务
        
        参数:
        task_id (str): 任务ID
        """
        with self.lock:
            if task_id not in self.tasks:
                raise ValueError(f"任务ID {task_id} 不存在")
                
            if self.tasks[task_id]['status'] != 'pending':
                raise RuntimeError(f"任务 {task_id} 状态为 {self.tasks[task_id]['status']}, 无法启动")
                
            if len(self.active_tasks) >= self.max_workers:
                raise RuntimeError("已达到最大工作线程数")
                
            print("===================task start=====================")
            # 创建并启动线程
            task_info = self.tasks[task_id]
            thread = threading.Thread(
                target=self._task_wrapper, 
                args=(task_id,),
                daemon=True
            )
            
            task_info['thread'] = thread
            task_info['status'] = 'running'
            task_info['start_time'] = time.time()
            
            self.active_tasks.append(task_id)
            thread.start()
    
    def start_all_tasks(self):
        """
        启动所有待处理任务
        """
        with self.lock:
            pending_tasks = [
                task_id for task_id, task in self.tasks.items()
                if task['status'] == 'pending'
            ]
            
            for task_id in pending_tasks:
                if len(self.active_tasks) < self.max_workers:
                    self.start_task(task_id)
    
    def _task_wrapper(self, task_id: str):
        """
        任务执行包装器
        """
        try:
            task_info = self.tasks[task_id]
            result = task_info['func'](
                *task_info['args'], 
                **task_info['kwargs']
            )
            task_info['result'] = result
            task_info['status'] = 'completed'
        except Exception as e:
            task_info['exception'] = e
            task_info['status'] = 'failed'
        finally:
            with self.lock:
                task_info['end_time'] = time.time()
                if task_id in self.active_tasks:
                    self.active_tasks.remove(task_id)
                
                # 检查是否所有任务都完成
                if self.auto_reset and self.all_tasks_completed():
                    self.task_complete_event.set()
    
    def wait_task(self, task_id: str, timeout: Optional[float] = None) -> bool:
        """
        等待指定任务完成
        
        参数:
        task_id (str): 任务ID
        timeout (float): 超时时间(秒)
        
        返回:
        bool: 是否在超时前完成
        """
        if task_id not in self.tasks:
            raise ValueError(f"任务ID {task_id} 不存在")
            
        task_info = self.tasks[task_id]
        if task_info['status'] == 'completed' or task_info['status'] == 'failed':
            return True
            
        if task_info['thread']:
            task_info['thread'].join(timeout=timeout)
            return not task_info['thread'].is_alive()
            
        return False
    
    def wait_all_tasks(self, timeout: Optional[float] = None) -> bool:
        """
        等待所有任务完成
        
        参数:
        timeout (float): 超时时间(秒)
        
        返回:
        bool: 是否在超时前所有任务都完成
        """
        start_time = time.time()
        remaining_timeout = timeout
        
        while True:
            # 检查是否所有任务都已完成
            if self.all_tasks_completed():
                return True
                
            # 计算剩余超时时间
            if timeout is not None:
                elapsed = time.time() - start_time
                remaining_timeout = timeout - elapsed
                if remaining_timeout <= 0:
                    return False
                    
            # 等待任务完成事件
            if not self.task_complete_event.wait(timeout=min(0.1, remaining_timeout) if timeout is not None else 0.1):
                # 超时检查
                if timeout is not None and (time.time() - start_time) >= timeout:
                    return False
    
    def get_task_result(self, task_id: str, raise_exception: bool = True) -> Any:
        """
        获取任务结果
        
        参数:
        task_id (str): 任务ID
        raise_exception (bool): 如果任务失败，是否抛出异常
        
        返回:
        Any: 任务结果
        """
        if task_id not in self.tasks:
            raise ValueError(f"任务ID {task_id} 不存在")
            
        task_info = self.tasks[task_id]
        
        if task_info['status'] == 'running' or task_info['status'] == 'pending':
            raise RuntimeError("任务尚未完成")
            
        if task_info['status'] == 'failed' and raise_exception:
            raise task_info['exception'] or RuntimeError("任务执行失败")
            
        return task_info['result']
    
    def all_tasks_completed(self) -> bool:
        """
        检查是否所有任务都已完成
        
        返回:
        bool: 是否所有任务都已完成（成功或失败）
        """
        with self.lock:
            for task_id, task in self.tasks.items():
                if task['status'] not in ['completed', 'failed']:
                    return False
            return True
    
    def get_task_status(self, task_id: str) -> str:
        """
        获取任务状态
        
        返回:
        str: 任务状态 (pending, running, completed, failed)
        """
        if task_id not in self.tasks:
            raise ValueError(f"任务ID {task_id} 不存在")
        return self.tasks[task_id]['status']
    
    def close(self):
        """
        关闭执行器，清理资源
        """
        self.reset()
        # 不需要额外清理，因为使用守护线程