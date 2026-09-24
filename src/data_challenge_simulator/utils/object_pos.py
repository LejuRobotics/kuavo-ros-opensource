import rospy, time
from geometry_msgs.msg import PoseStamped

class ObjectPose:
    def __init__(self):
        # 定义物体名到 topic 的映射
        self.name_to_topic = {
            "cylinder_1": "/mujoco/cylinder_1/pose",
            "cylinder_2": "/mujoco/cylinder_2/pose",
            "cylinder_3": "/mujoco/cylinder_3/pose",
            "target_bin": "/mujoco/target_bin/pose",
            "waist_yaw_link": "/mujoco/waist_yaw_link/pose",
            "lever": "/mujoco/lever/pose",
            "source_bin": "/mujoco/source_bin/pose",
            "box_1": "/mujoco/box_1/pose",
            "box_2": "/mujoco/box_2/pose",
            "base_link": "/mujoco/base_link/pose",
            "l_hand_base": "/mujoco/l_hand_base/pose",
            "r_hand_base": "/mujoco/r_hand_base/pose",
            "task3_hollow_cylinder": "/mujoco/task3_hollow_cylinder/pose",
            "task3_hollow_cylinder_2": "/mujoco/task3_hollow_cylinder_2/pose",
            "task3_hollow_cylinder_3": "/mujoco/task3_hollow_cylinder_3/pose",
            "task3_destination_table": "/mujoco/task3_destination_table/pose",
        }

        self.pose_data = {name: None for name in self.name_to_topic}
        self.subscribers = []

        for name, topic in self.name_to_topic.items():
            sub = rospy.Subscriber(topic, PoseStamped, self._callback, callback_args=name)
            self.subscribers.append(sub)

    def _callback(self, msg, object_name):
        self.pose_data[object_name] = msg.pose

    def wait_for_position(self, name, timeout=5.0):
        end = time.time() + timeout
        rate = rospy.Rate(20)
        while time.time() < end and not rospy.is_shutdown():
            pos = self.get_position(name)
            if pos is not None:
                return pos
            rate.sleep()
        raise RuntimeError(f"Timeout waiting for {name}")

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
