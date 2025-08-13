import time
from enum import Enum
from typing import Any
import logging
import colorlog


class EventStatus(Enum):
    IDLE = "idle"  # 空闲状态，事件未开始
    RUNNING = "running"  # 事件正在进行中
    SUCCESS = "success"

    FAILED = "failed"
    CLOSED = "closed"  # 事件已停止
    TIMEOUT = "timeout"  # 事件超时


class BaseEvent:
    """
    事件：
    单一的输入
    有状态判断
    """

    def __init__(self,
                 event_name,
                 ):
        """
        初始化事件，设置事件名称并将初始状态设置为IDLE。

        参数：
            event_name (str): 事件名称。
        """
        self.event_name = event_name
        self.status = EventStatus.IDLE  # 事件状态，初始为IDLE
        self.start_time = None  # 事件初始状态
        self.target = None  # 事件目标，初始为None

        self.logger = logging.getLogger(self.event_name)
        self.logger.setLevel(logging.INFO)
        # 避免重复添加 handler
        if not self.logger.handlers:
            handler = colorlog.StreamHandler()
            formatter = colorlog.ColoredFormatter(
                fmt='%(log_color)s[%(asctime)s] [事件：%(name)s] %(levelname)s: %(message)s',
                datefmt='%H:%M:%S',
                log_colors={
                    'DEBUG':    'cyan',
                    'INFO':     'green',
                    'WARNING':  'yellow',
                    'ERROR':    'red',
                    'CRITICAL': 'bold_red',
                }
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

    def set_timeout(self, timeout):
        """
        设置事件的超时时间。

        参数：
            timeout (int): 超时时间（秒），必须大于0。
        """
        assert timeout > 0, "超时时间必须大于0 !!!"
        self.timeout = timeout
        self.logger.info(f"事件 {self.event_name} 超时时间设置为 {self.timeout} 秒")

    def set_target(self, target: Any, *args, **kwargs):
        """
        设置事件的目标，可以在事件执行期间动态更新。

        参数：
            target (Any): 事件目标，例如位置或ID。
            `*args`: 额外的参数。
            `**kwargs`: 额外的关键字参数。

        返回：
            bool: 如果目标设置成功返回True，否则返回False。
        """

        if self.status == EventStatus.CLOSED:
            self.logger.error(f"事件 {self.event_name} 已关闭，无法设置目标 !!! 请先调用open() 方法开始事件")
            return False

        if self.status != EventStatus.RUNNING and self.status != EventStatus.IDLE:
            self.logger.error(f"事件 {self.event_name} 不是运行中或空闲状态，无法设置目标 !!!")
            return False

        is_valid = self._check_target_valid(target, *args, **kwargs)

        if not is_valid:
            self.logger.error(f"事件 {self.event_name} 的目标无效，无法设置目标 !!!")
            return False

        self.target = target

        self.logger.info(f"目标已设置为：\n {self.target}")

        return True

    def open(self, *args, **kwargs):
        """
        开始事件，将状态更改为RUNNING并记录开始时间。

        参数：
            `*args`: 额外的参数。
            `**kwargs`: 额外的关键字参数。
        """
        self.status = EventStatus.RUNNING
        self.start_time = time.time()  # 记录事件开始时间
        self.logger.info(f"🔵 事件开始啦")

    def close(self):
        """
        停止事件，将状态更改为CLOSED。
        """
        self.status = EventStatus.CLOSED
        self.logger.info(f"🔵 事件关闭啦")

    def step(self):
        """
        抽象方法，需要在子类中实现以定义事件的每一步行为。

        异常：
            NotImplementedError: 如果在子类中未实现。
        """
        raise NotImplementedError("请在子类中实现 step 方法")

    def _check_target_valid(self, target: Any, *args, **kwargs) -> bool:
        """
        抽象方法，需要在子类中实现以验证事件的目标。

        参数：
            target (Any): 需要验证的目标。
            `*args`: 额外的参数。
            `**kwargs`: 额外的关键字参数。

        返回：
            bool: 如果目标有效返回True，否则返回False。

        异常：
            NotImplementedError: 如果在子类中未实现。
        """
        raise NotImplementedError("请在子类中实现 _check_target_valid 方法")

    def get_status(self):
        """
        返回更新后的事件状态。

        返回：
            EventStatus: 当前事件状态。
        """
        self._update_status()
        return self.status

    def _update_status(self):
        """
        根据当前状态更新事件的状态，检查是否失败、成功或超时。
        """
        # 如果已经是failed或者success或者timeout，则不再变化状态
        if self.status in [EventStatus.FAILED, EventStatus.SUCCESS, EventStatus.TIMEOUT]:
            return

        if self._check_failed():
            self.logger.error(f"❌ 当前状态为：失败 !!!")
            self.status = EventStatus.FAILED
        elif self._check_success():
            self.logger.info(f"✅ 当前状态为：成功 !!!")
            self.status = EventStatus.SUCCESS
        elif self._check_timeout():
            self.logger.error(f"❌ 当前状态为：超时 !!!")
            self.status = EventStatus.TIMEOUT

    def _check_timeout(self):
        """
        检查事件是否超时。

        返回：
            bool: 如果事件超时返回True，否则返回False。

        异常：
            AssertionError: 如果事件未开始或未设置超时时间。
        """
        assert self.start_time is not None, "事件未开始，无法检查超时 !!!"
        assert self.timeout is not None, "事件超时时间未设置 !!!"

        elapsed_time = time.time() - self.start_time
        if elapsed_time > self.timeout:
            return True
        return False

    def _check_failed(self):
        """
        抽象方法，需要在子类中实现以判断事件是否失败。

        返回：
            bool: 如果事件失败返回True，否则返回False。

        异常：
            NotImplementedError: 如果在子类中未实现。
        """
        raise NotImplementedError("请在子类中实现 _check_failed 方法")

    def _check_success(self):
        """
        抽象方法，需要在子类中实现以判断事件是否成功。

        返回：
            bool: 如果事件成功返回True，否则返回False。

        异常：
            NotImplementedError: 如果在子类中未实现。
        """
        raise NotImplementedError("请在子类中实现 _check_success 方法")
