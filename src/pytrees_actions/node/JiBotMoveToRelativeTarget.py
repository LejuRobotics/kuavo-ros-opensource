from py_trees.behaviour import Behaviour
from py_trees.common import Status
import rospy
import py_trees
import math
from std_srvs.srv import SetBool
from leju_mobile_base_msgs.srv import RelativeBaseMove
from leju_mobile_base_msgs.msg import MoveToTargetOptions
from .utils import filter_tree_path
import time


class JiBotMoveToRelativeTarget(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params):
        super(JiBotMoveToRelativeTarget, self).__init__(name)
        self.params = params
        self.label = label.split('/', -1)[-1]

        # 服务名称
        self.service_name = self.params.get('service_name', '/5w_move_to_relative_target')

        # 必要参数（在 initialise 中验证）
        self.x = None
        self.y = None
        self.theta = None
        self.params_validated = False

        # 可选参数（带默认值）
        avoid_enabled_val = self.params.get('avoid_enabled', False)
        if isinstance(avoid_enabled_val, str):
            self.avoid_enabled = avoid_enabled_val.lower() in ('true', '1', 'yes', 'on')
        else:
            self.avoid_enabled = bool(avoid_enabled_val)

        self.linear_velocity = float(self.params.get('linear_velocity', 0.5))
        self.angular_velocity = float(self.params.get('angular_velocity', 0.5))
        self.position_threshold = float(self.params.get('position_threshold', 0.01))
        self.angle_threshold = float(self.params.get('angle_threshold', 0.01))

        allow_rotation_val = self.params.get('allow_rotation', True)
        if isinstance(allow_rotation_val, str):
            self.allow_rotation = allow_rotation_val.lower() in ('true', '1', 'yes', 'on')
        else:
            self.allow_rotation = bool(allow_rotation_val)

        # 状态变量
        self.service_called = False
        self.service_response = None
        self.service_proxy = None
        self.enable_vel_control_proxy = None
        self.vel_control_disabled = False

        # 黑板客户端
        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)
        self.global_blackboard = self.attach_blackboard_client()

        self.logger.info(f"JiBotMoveToRelativeTarget initialized (参数将在 initialise 中验证)")

    def initialise(self):
        """初始化节点"""
        self.service_called = False
        self.service_response = None
        self.params_validated = False
        self.vel_control_disabled = False

        # 验证必要参数
        if 'x' not in self.params:
            self.feedback_message = "错误: 必要参数 'x' 未提供"
            self.logger.error(self.feedback_message)
            return
        if 'y' not in self.params:
            self.feedback_message = "错误: 必要参数 'y' 未提供"
            self.logger.error(self.feedback_message)
            return
        if 'theta' not in self.params:
            self.feedback_message = "错误: 必要参数 'theta' 未提供"
            self.logger.error(self.feedback_message)
            return

        # 解析必要参数
        try:
            self.x = float(self.params.get('x'))
            self.y = float(self.params.get('y'))
            # theta 输入为角度制，转换为弧度制
            theta_degrees = float(self.params.get('theta'))
            self.theta = math.radians(theta_degrees)
            self.params_validated = True
        except (ValueError, TypeError) as e:
            self.feedback_message = f"错误: 参数类型无效 - {e}"
            self.logger.error(self.feedback_message)
            return

        # 重新读取可选参数（允许在 initialise 时更新）
        if 'avoid_enabled' in self.params:
            val = self.params.get('avoid_enabled')
            # 处理字符串类型的布尔值
            if isinstance(val, str):
                self.avoid_enabled = val.lower() in ('true', '1', 'yes', 'on')
            else:
                self.avoid_enabled = bool(val)
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
            # 处理字符串类型的布尔值
            if isinstance(val, str):
                self.allow_rotation = val.lower() in ('true', '1', 'yes', 'on')
            else:
                self.allow_rotation = bool(val)

        self.feedback_message = f"准备调用导航服务 {self.service_name}"
        self.logger.info(self.feedback_message)
        theta_degrees = math.degrees(self.theta)
        self.logger.info(f"目标位置: x={self.x}, y={self.y}, theta={theta_degrees}° (弧度: {self.theta:.4f})")

        # 等待 enable_vel_control 服务可用（在 update 中调用）
        try:
            rospy.wait_for_service('/enable_vel_control', timeout=5.0)
            self.enable_vel_control_proxy = rospy.ServiceProxy('/enable_vel_control', SetBool)
            self.logger.info("服务 /enable_vel_control 已就绪")
        except rospy.ROSException as e:
            self.logger.warn(f"等待服务 /enable_vel_control 超时: {e}，将在 update 中重试")

        # 等待导航服务可用
        try:
            rospy.wait_for_service(self.service_name, timeout=5.0)
            self.service_proxy = rospy.ServiceProxy(self.service_name, RelativeBaseMove)
            self.logger.info(f"服务 {self.service_name} 已就绪")
        except rospy.ROSException as e:
            self.logger.error(f"等待服务 {self.service_name} 超时: {e}")
            self.feedback_message = f"服务不可用: {e}"
            return

    def update(self):
        """更新节点状态"""
        # 检查参数是否已验证
        if not self.params_validated:
            self.logger.error("参数验证失败，无法执行导航")
            return Status.FAILURE

        try:
            # 如果还没有调用服务，则调用
            if not self.service_called:
                # 在调用导航服务之前，先调用 /enable_vel_control 服务，设置 data=False
                # 必须成功后才能执行导航
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

                # 创建选项消息
                options = MoveToTargetOptions()
                options.avoid_enabled = self.avoid_enabled
                options.linear_velocity = self.linear_velocity
                options.angular_velocity = self.angular_velocity
                options.position_threshold = self.position_threshold
                options.angle_threshold = self.angle_threshold
                options.allow_rotation = self.allow_rotation

                # 调用服务
                if self.service_proxy is None:
                    self.logger.error(f"服务代理未初始化")
                    self.feedback_message = "服务代理未初始化"
                    return Status.FAILURE

                theta_degrees = math.degrees(self.theta)
                self.logger.info(f"调用导航服务: x={self.x}, y={self.y}, theta={theta_degrees}° (弧度: {self.theta:.4f})")
                self.logger.info(f"选项: avoid_enabled={self.avoid_enabled}, "
                               f"linear_velocity={self.linear_velocity}, "
                               f"angular_velocity={self.angular_velocity}, "
                               f"position_threshold={self.position_threshold}, "
                               f"angle_threshold={self.angle_threshold}, "
                               f"allow_rotation={self.allow_rotation}")

                # 调用服务（ROS 1 中可以直接使用关键字参数）
                self.service_response = self.service_proxy(
                    x=self.x,
                    y=self.y,
                    theta=self.theta,
                    options=options
                )
                self.service_called = True

                # 检查响应：只有当 success=True, message="success", error_code=0 时才认为成功
                if (self.service_response.success and
                    self.service_response.message == "success" and
                    self.service_response.error_code == 0):
                    self.feedback_message = f"导航发送成功: {self.service_response.message}"
                    self.logger.info(self.feedback_message)
                    time.sleep(2)
                    return Status.SUCCESS
                else:
                    self.feedback_message = f"导航发送失败: {self.service_response.message} (错误代码: {self.service_response.error_code}, success: {self.service_response.success})"
                    self.logger.error(self.feedback_message)
                    return Status.FAILURE
            else:
                # 服务已调用，返回之前的结果
                if (self.service_response and
                    self.service_response.success and
                    self.service_response.message == "success" and
                    self.service_response.error_code == 0):
                    time.sleep(2)
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
        self.logger.info(f"JiBotMoveToRelativeTarget 节点终止，状态: {new_status}")

