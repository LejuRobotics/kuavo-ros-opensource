import rospy
from geometry_msgs.msg import PoseStamped

class ObjectPose:
    def __init__(self):
        # 定义物体名到 topic 的映射
        self.name_to_topic = {
            "box_grab": "/mujoco/box_grab/pose",
            "plate": "/mujoco/plate/pose",
            "shampoo1": "/mujoco/shampoo1/pose",
            "shampoo2": "/mujoco/shampoo2/pose",
            "shampoo3": "/mujoco/shampoo3/pose",
            "shampoo4": "/mujoco/shampoo4/pose",
            "item1": "/mujoco/item1/pose",
            "item2": "/mujoco/item2/pose"
        }

        self.pose_data = {name: None for name in self.name_to_topic}
        self.subscribers = []

        for name, topic in self.name_to_topic.items():
            sub = rospy.Subscriber(topic, PoseStamped, self._callback, callback_args=name)
            self.subscribers.append(sub)

    def _callback(self, msg, object_name):
        self.pose_data[object_name] = msg.pose

    def get_position(self, object_name):
        """
        获取指定物体的位置 (x, y, z)。无数据则返回 None。
        """
        pose = self.pose_data.get(object_name)
        if pose is None:
            return None
        pos = pose.position
        return (pos.x, pos.y, pos.z)

    def get_orientation(self, object_name):
        """
        获取指定物体的姿态四元数 (x, y, z, w)。无数据则返回 None。
        """
        pose = self.pose_data.get(object_name)
        if pose is None:
            return None
        ori = pose.orientation
        return (ori.x, ori.y, ori.z, ori.w)
