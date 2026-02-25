from py_trees.behaviour import Behaviour
from py_trees.common import Status
import rospy
import py_trees


class SetTaskStatus(Behaviour):
    """设置任务状态到参数服务器的节点"""

    def __init__(self, name: str, label: str, namespace: str, params):
        super(SetTaskStatus, self).__init__(name)
        self.params = params
        self.label = label.split('/', -1)[-1]

        # 黑板客户端
        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)
        self.global_blackboard = self.attach_blackboard_client()

        # 从参数中获取要设置的值，默认为 "success"
        self.status_value = self.params.get('status_value', 'success')
        self.param_name = self.params.get('param_name', '/task_status')

        self.logger.info(f"SetTaskStatus initialized: param_name={self.param_name}, status_value={self.status_value}")

    def initialise(self):
        """初始化节点"""
        self.feedback_message = f"准备设置参数 {self.param_name} = {self.status_value}"
        self.logger.info(self.feedback_message)

    def update(self):
        """更新节点状态 - 设置参数并返回 SUCCESS"""
        try:
            # 检查参数是否存在，如果存在则删除
            if rospy.has_param(self.param_name):
                rospy.delete_param(self.param_name)

            # 设置新值
            rospy.set_param(self.param_name, self.status_value)
            self.feedback_message = f"已设置参数 {self.param_name} = {self.status_value}"
            self.logger.info(self.feedback_message)
            return Status.SUCCESS

        except Exception as e:
            self.feedback_message = f"设置参数失败: {e}"
            self.logger.error(self.feedback_message)
            return Status.FAILURE

    def terminate(self, new_status):
        """节点终止时的清理工作"""
        self.logger.info(f"SetTaskStatus 节点终止，状态: {new_status}")





