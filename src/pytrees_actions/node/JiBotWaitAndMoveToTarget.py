from py_trees.behaviour import Behaviour
from py_trees.common import Status
import rospy
import py_trees
import threading
from leju_mobile_base_msgs.srv import MoveToTarget
from leju_mobile_base_msgs.msg import MoveToTargetOptions
from pytrees_actions.srv import SetMoveTarget, SetMoveTargetRequest, SetMoveTargetResponse
from .utils import filter_tree_path
from std_srvs.srv import SetBool

class JiBotWaitAndMoveToTarget(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params):
        super(JiBotWaitAndMoveToTarget, self).__init__(name)
        self.params = params
        self.label = label.split('/', -1)[-1]

        # 底盘移动服务名称（使用新的服务）
        self.base_move_service_name = self.params.get('service_name', '/move_base/move_to_target')

        # 服务的名称（用于接收外部请求）
        self.move_service_name = self.params.get('move_service_name', '/jibot_set_move_target')

        # 可选参数（带默认值）
        avoid_enabled_val = self.params.get('avoid_enabled', False)
        if isinstance(avoid_enabled_val, str):
            self.avoid_enabled = avoid_enabled_val.lower() in ('true', '1', 'yes', 'on')
        else:
            self.avoid_enabled = bool(avoid_enabled_val)

        self.avoid_distance = float(self.params.get('avoid_distance', 0.0))
        self.linear_velocity = float(self.params.get('linear_velocity', 0.5))
        self.angular_velocity = float(self.params.get('angular_velocity', 0.5))
        self.position_threshold = float(self.params.get('position_threshold', 0.05))
        self.angle_threshold = float(self.params.get('angle_threshold', 0.0175))

        allow_rotation_val = self.params.get('allow_rotation', True)
        if isinstance(allow_rotation_val, str):
            self.allow_rotation = allow_rotation_val.lower() in ('true', '1', 'yes', 'on')
        else:
            self.allow_rotation = bool(allow_rotation_val)

        # 状态变量
        self.waiting = True  # 是否在等待
        self.move_service_called = False  # 移动服务是否已调用
        self.service_called = False  # 是否已调用底盘移动服务
        self.service_response = None
        self.move_service_proxy = None
        self.move_service = None  # 移动服务（接收x, y, theta）

        # 接收到的参数（从服务请求中获取）
        self.target_x = None
        self.target_y = None
        self.target_theta = None

        # 线程锁，保护共享状态
        self.lock = threading.Lock()

        # 黑板客户端
        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)
        self.global_blackboard = self.attach_blackboard_client()

        self.logger.info(f"JiBotWaitAndMoveToTarget initialized, move_service={self.move_service_name}")

    def _handle_move_target_service(self, req):
        """处理移动目标服务请求（接收x, y, theta）"""
        try:
            if len(req.target_pose.data) != 3:
                self.logger.error(f"移动目标服务参数错误: 需要3个参数(x, y, theta)，收到{len(req.target_pose.data)}个")
                return SetMoveTargetResponse(success=False, message=f"参数错误: 需要3个参数，收到{len(req.target_pose.data)}个")

            with self.lock:
                self.target_x = req.target_pose.data[0]
                self.target_y = req.target_pose.data[1]
                self.target_theta = req.target_pose.data[2]
                self.move_service_called = True  # 标记移动服务已调用

            self.logger.info(f"收到移动目标: x={self.target_x}, y={self.target_y}, theta={self.target_theta}")
            return SetMoveTargetResponse(success=True, message=f"移动目标已设置: x={self.target_x}, y={self.target_y}, theta={self.target_theta}")
        except Exception as e:
            self.logger.error(f"处理移动目标服务时出错: {e}")
            return SetMoveTargetResponse(success=False, message=f"处理请求时出错: {str(e)}")

    def initialise(self):
        """初始化节点"""
        with self.lock:
            self.waiting = True
            self.move_service_called = False
            self.service_called = False
            self.service_response = None
            self.target_x = None
            self.target_y = None
            self.target_theta = None
            self.move_service_proxy = None  # 在update中初始化
            self.vel_control_disabled = False

        # 重新读取可选参数（允许在 initialise 时更新）
        if 'avoid_enabled' in self.params:
            val = self.params.get('avoid_enabled')
            if isinstance(val, str):
                self.avoid_enabled = val.lower() in ('true', '1', 'yes', 'on')
            else:
                self.avoid_enabled = bool(val)
        if 'avoid_distance' in self.params:
            self.avoid_distance = float(self.params.get('avoid_distance'))
        if 'linear_velocity' in self.params:
            self.linear_velocity = float(self.params.get('linear_velocity'))
        if 'angular_velocity' in self.params:
            self.angular_velocity = float(self.params.get('angular_velocity'))
        if 'position_threshold' in self.params:
            self.position_threshold = float(self.params.get('position_threshold'))
        if 'angle_threshold' in self.params:
            self.angle_threshold = float(self.params.get('angle_threshold'))
        if 'allow_rotation' in self.params:
            val = self.params.get('allow_rotation')
            if isinstance(val, str):
                self.allow_rotation = val.lower() in ('true', '1', 'yes', 'on')
            else:
                self.allow_rotation = bool(val)

        # 创建服务
        try:
            # 服务1：接收移动目标参数 (x, y, theta)
            self.move_service = rospy.Service(
                self.move_service_name,
                SetMoveTarget,
                self._handle_move_target_service
            )
            self.logger.info(f"移动目标服务已创建: {self.move_service_name}")

        except Exception as e:
            self.logger.error(f"创建服务失败: {e}")
            self.feedback_message = f"创建服务失败: {e}"
            return

        self.feedback_message = f"等待移动请求，服务: {self.move_service_name}"
        self.logger.info(self.feedback_message)

        try:
            rospy.wait_for_service('/enable_vel_control', timeout=5.0)
            self.enable_vel_control_proxy = rospy.ServiceProxy('/enable_vel_control', SetBool)
            self.logger.info("服务 /enable_vel_control 已就绪")
        except rospy.ROSException as e:
            self.logger.warn(f"等待服务 /enable_vel_control 超时: {e}，将在 update 中重试")

    def update(self):
        """更新节点状态"""
        try:
            # 在update中等待底盘移动服务可用（不报错，一直等待）
            if self.move_service_proxy is None:
                try:
                    # 非阻塞检查服务是否可用（使用很短的超时，避免阻塞）
                    rospy.wait_for_service(self.base_move_service_name, timeout=0.1)
                    self.move_service_proxy = rospy.ServiceProxy(self.base_move_service_name, MoveToTarget)
                    self.logger.info(f"底盘移动服务 {self.base_move_service_name} 已就绪")
                except rospy.ROSException:
                    # 服务不可用，继续等待（不报错，返回RUNNING）
                    self.feedback_message = f"等待底盘移动服务 {self.base_move_service_name} 可用..."
                    self.logger.debug(self.feedback_message)
                    return Status.RUNNING

            with self.lock:
                # 检查移动服务是否已调用
                if not self.move_service_called:
                    # 还在等待服务调用
                    self.feedback_message = "等待移动服务调用..."
                    self.logger.debug(self.feedback_message)
                    return Status.RUNNING

                # 检查是否有目标位置
                if self.target_x is None or self.target_y is None or self.target_theta is None:
                    self.feedback_message = "等待移动目标参数设置..."
                    self.logger.debug(self.feedback_message)
                    return Status.RUNNING

                # 如果已经调用过底盘移动服务，返回之前的结果
                if self.service_called:
                    if self.service_response and self.service_response.success:
                        return Status.SUCCESS
                    else:
                        return Status.FAILURE

                # 标记不再等待，准备调用底盘移动服务
                self.waiting = False

            # 创建选项消息（在锁外创建，避免长时间持有锁）
            options = MoveToTargetOptions()
            options.avoid_enabled = self.avoid_enabled
            options.avoid_distance = self.avoid_distance
            options.linear_velocity = self.linear_velocity
            options.angular_velocity = self.angular_velocity
            options.position_threshold = self.position_threshold
            options.angle_threshold = self.angle_threshold
            options.allow_rotation = self.allow_rotation

            # 获取目标位置（在锁外读取，避免长时间持有锁）
            target_x = self.target_x
            target_y = self.target_y
            target_theta = self.target_theta

            self.logger.info(f"调用底盘移动服务 {self.base_move_service_name}: x={target_x}, y={target_y}, theta={target_theta}")
            self.logger.info(f"选项: avoid_enabled={self.avoid_enabled}, "
                           f"avoid_distance={self.avoid_distance}, "
                           f"linear_velocity={self.linear_velocity}, "
                           f"angular_velocity={self.angular_velocity}, "
                           f"position_threshold={self.position_threshold}, "
                           f"angle_threshold={self.angle_threshold}, "
                           f"allow_rotation={self.allow_rotation}")

            if not self.vel_control_disabled:
                try:
                    if self.enable_vel_control_proxy is None:
                        rospy.wait_for_service('/enable_vel_control', timeout=2.0)
                        self.enable_vel_control_proxy = rospy.ServiceProxy('/enable_vel_control', SetBool)

                    vel_control_response = self.enable_vel_control_proxy(data=False)
                    if vel_control_response.success:
                        self.logger.info("成功调用 /enable_vel_control 服务，设置 data=False")
                        self.vel_control_disabled = True
                        # 延时 1 秒后再执行导航
                        rospy.sleep(1.0)
                        self.logger.info("延时 1 秒完成，准备执行导航")
                    else:
                        self.feedback_message = f"/enable_vel_control 服务返回失败: {vel_control_response.message}，重试中..."
                        self.logger.warn(self.feedback_message)
                        return Status.RUNNING
                except (rospy.ROSException, rospy.ServiceException) as e:
                    self.feedback_message = f"调用 /enable_vel_control 服务失败: {e}，重试中..."
                    self.logger.warn(self.feedback_message)
                    return Status.RUNNING

            # 如果 enable_vel_control 还未成功，继续等待
            if not self.vel_control_disabled:
                return Status.RUNNING

            # 调用服务
            self.service_response = self.move_service_proxy(
                x=target_x,
                y=target_y,
                theta=target_theta,
                options=options
            )

            with self.lock:
                self.service_called = True

            # 检查响应
            if self.service_response.success:
                # 将 task_id 写入参数服务器，供后续节点使用
                try:
                    rospy.set_param('/move_task_id', self.service_response.task_id)
                    self.logger.info(f"已将 task_id={self.service_response.task_id} 写入参数服务器")
                except Exception as e:
                    self.logger.warn(f"写入 task_id 到参数服务器失败: {e}")
                
                self.feedback_message = f"导航发送成功: {self.service_response.message}, task_id={self.service_response.task_id}"
                self.logger.info(self.feedback_message)
                return Status.SUCCESS
            else:
                self.feedback_message = f"导航发送失败: {self.service_response.message}"
                self.logger.error(self.feedback_message)
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
        # 关闭服务
        if self.move_service is not None:
            try:
                self.move_service.shutdown()
                self.logger.info(f"移动目标服务已关闭: {self.move_service_name}")
            except Exception as e:
                self.logger.warning(f"关闭移动目标服务时出错: {e}")

        # 在节点运行结束后，将参数服务器 /task_status 的值置为 none
        try:
            if rospy.has_param('/task_status'):
                rospy.delete_param('/task_status')
            rospy.set_param('/task_status', 'running')
            self.logger.info("已将参数服务器 /task_status 设置为 'running'")
        except Exception as e:
            self.logger.warning(f"设置参数服务器 /task_status 时出错: {e}")

        self.logger.info(f"JiBotWaitAndMoveToTarget 节点终止，状态: {new_status}")

