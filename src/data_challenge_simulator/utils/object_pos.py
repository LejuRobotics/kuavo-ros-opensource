import rospy, time
from geometry_msgs.msg import PoseStamped

class ObjectPose:
    def __init__(self):

        self.name_to_topic = {
            "box_red1": "/mujoco/box_red1/pose",
            "box_red2": "/mujoco/box_red2/pose",
            "box_black1": "/mujoco/box_black1/pose",
            "box_black2": "/mujoco/box_black2/pose",
            "bag_place_root": "/mujoco/bag_place_root/pose",
            "bag_drop_root": "/mujoco/bag_drop_root/pose",
            "basket1": "/mujoco/basket1/pose",
            "basket2": "/mujoco/basket2/pose",
            "dino": "/mujoco/dino/pose",
            "dog": "/mujoco/dog/pose",
            "elephant": "/mujoco/elephant/pose",
            "shark": "/mujoco/shark/pose",
            "sheep": "/mujoco/sheep/pose",
            "fire": "/mujoco/fire/pose",
            "fire_engine": "/mujoco/fire_engine/pose",
            "roller": "/mujoco/roller/pose",
            "school": "/mujoco/school/pose",
            "boat": "/mujoco/boat/pose",
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

        pose = self.pose_data.get(object_name)
        if pose is None:
            return None
        pos = pose.position
        return (pos.x, pos.y, pos.z)

    def get_orientation(self, object_name):

        pose = self.pose_data.get(object_name)
        if pose is None:
            return None
        ori = pose.orientation
        return (ori.x, ori.y, ori.z, ori.w)
