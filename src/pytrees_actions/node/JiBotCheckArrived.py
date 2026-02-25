from py_trees.behaviour import Behaviour
from py_trees.common import Status
import rospy
import py_trees
from std_srvs.srv import SetBool
from leju_mobile_base_msgs.srv import CheckArrived


class JiBotCheckArrived(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params):
        super(JiBotCheckArrived, self).__init__(name)
        self.params = params
        self.label = label.split('/', -1)[-1]

        # 服务名称
        self.service_name = self.params.get('service_name', '/move_base/check_arrived')

        # 可选参数（带默认值）
        self.timeout = float(self.params.get('timeout', 0.0))
        self.blocking = self.params.get('blocking', True)
        if isinstance(self.blocking, str):
            self.blocking = self.blocking.lower() in ('true', '1', 'yes', 'on')
        else:
            self.blocking = bool(self.blocking)

        # 状态变量
        self.service_called = False
        self.service_response = None
        self.service_proxy = None
        self.enable_vel_control_proxy = None
        self.vel_control_enabled = False

        # 黑板客户端
        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)
        self.global_blackboard = self.attach_blackboard_client()

        self.logger.info(f"JiBotCheckArrived initialized with timeout={self.timeout}")

    def initialise(self):
        """初始化节点"""
        self.service_called = False
        self.service_response = None
        self.vel_control_enabled = False

        # 重新读取可选参数（允许在 initialise 时更新）
        if 'timeout' in self.params:
            self.timeout = float(self.params.get('timeout'))
        else:
            self.timeout = 0.0  # 默认值
        
        if 'blocking' in self.params:
            val = self.params.get('blocking')
            if isinstance(val, str):
                self.blocking = val.lower() in ('true', '1', 'yes', 'on')
            else:
                self.blocking = bool(val)

        self.feedback_message = f"准备调用检查到达服务 {self.service_name}, timeout={self.timeout}, blocking={self.blocking}"
        self.logger.info(self.feedback_message)

        # 等待 enable_vel_control 服务可用（在 update 中调用）
        try:
            rospy.wait_for_service('/enable_vel_control', timeout=5.0)
            self.enable_vel_control_proxy = rospy.ServiceProxy('/enable_vel_control', SetBool)
            self.logger.info("服务 /enable_vel_control 已就绪")
        except rospy.ROSException as e:
            self.logger.warn(f"等待服务 /enable_vel_control 超时: {e}，将在 update 中重试")

        # 等待服务可用
        try:
            rospy.wait_for_service(self.service_name, timeout=5.0)
            self.service_proxy = rospy.ServiceProxy(self.service_name, CheckArrived)
            self.logger.info(f"服务 {self.service_name} 已就绪")
        except rospy.ROSException as e:
            self.logger.error(f"等待服务 {self.service_name} 超时: {e}")
            self.feedback_message = f"服务不可用: {e}"
            return

    def update(self):
        """更新节点状态"""
        try:
            # 如果还没有调用服务，则调用
            if not self.service_called:
                # 调用服务
                if self.service_proxy is None:
                    self.logger.error(f"服务代理未初始化")
                    self.feedback_message = "服务代理未初始化"
                    return Status.FAILURE

                # 从参数服务器获取 task_id（JiBotWaitAndMoveToTarget 可能将 task_id 存储在某个地方）
                # 或者从最近的移动任务响应中获取，这里先尝试从参数服务器获取
                task_id = ''
                try:
                    # 尝试从参数服务器获取最新的 task_id
                    if rospy.has_param('/move_task_id'):
                        task_id = rospy.get_param('/move_task_id', '')
                except:
                    pass
                
                # 如果参数服务器没有，尝试从日志中提取（这不是最佳方案，但作为备选）
                # 更好的方案是通过黑板传递，但用户拒绝了之前的修改
                if not task_id:
                    self.logger.warn("未找到 task_id，使用空字符串（服务可能使用最近的任务）")
                
                self.logger.info(f"调用检查到达服务: task_id={task_id}, blocking={self.blocking}, timeout={self.timeout}")

                # 调用服务（传入 task_id, blocking, timeout）
                self.service_response = self.service_proxy(
                    task_id=task_id,
                    blocking=self.blocking,
                    timeout=self.timeout
                )
                self.service_called = True

                # 检查响应：当 arrived=True 且 success=True 时认为成功
                if (self.service_response.arrived and
                    self.service_response.success):
                    # 在返回 SUCCESS 之前，调用 /enable_vel_control 服务，设置 data=True
                    if not self.vel_control_enabled:
                        # 延时 1 秒后再调用 enable_vel_control
                        rospy.sleep(1.0)
                        self.logger.info("延时 1 秒完成，准备调用 /enable_vel_control")
                        try:
                            if self.enable_vel_control_proxy is None:
                                rospy.wait_for_service('/enable_vel_control', timeout=2.0)
                                self.enable_vel_control_proxy = rospy.ServiceProxy('/enable_vel_control', SetBool)

                            vel_control_response = self.enable_vel_control_proxy(data=True)
                            if vel_control_response.success:
                                self.logger.info("成功调用 /enable_vel_control 服务，设置 data=True")
                                self.vel_control_enabled = True
                            else:
                                self.logger.warn(f"/enable_vel_control 服务返回失败: {vel_control_response.message}")
                        except (rospy.ROSException, rospy.ServiceException) as e:
                            self.logger.warn(f"调用 /enable_vel_control 服务失败: {e}，继续返回 SUCCESS")

                    self.feedback_message = f"已到达目标: {self.service_response.message} (status: {self.service_response.status})"
                    self.logger.info(self.feedback_message)
                    return Status.SUCCESS
                else:
                    status_msg = {
                        0: "到达",
                        1: "超时",
                        2: "被中断",
                        3: "错误"
                    }
                    msg = status_msg.get(self.service_response.status, "未知")
                    self.feedback_message = f"未到达目标: arrived={self.service_response.arrived}, success={self.service_response.success}, message={self.service_response.message}, status={self.service_response.status}={msg}"
                    self.logger.warning(self.feedback_message)
                    return Status.FAILURE
            else:
                # 服务已调用，返回之前的结果
                if (self.service_response and
                    self.service_response.arrived and
                    self.service_response.success):
                    return Status.SUCCESS
                else:
                    return Status.FAILURE

        except rospy.ServiceException as e:
            self.logger.error(f"服务调用失败: {e}")
            self.feedback_message = f"服务调用失败: {e}"
            return Status.FAILURE
        except Exception as e:
            self.logger.error(f"节点执行出错: {e}")
            self.feedback_message = f"节点执行出错: {e}"
            return Status.FAILURE

    def terminate(self, new_status):
        """节点终止时的清理工作"""
        self.logger.info(f"JiBotCheckArrived 节点终止，状态: {new_status}")

