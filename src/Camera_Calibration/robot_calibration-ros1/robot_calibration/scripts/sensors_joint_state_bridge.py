#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

try:
    from kuavo_msgs.msg import sensorsData
except Exception:
    sensorsData = None


class SensorsJointStateBridge:
    """
    将 kuavo_msgs/sensorsData.joint_data.joint_q 的指定索引映射为 sensor_msgs/JointState。

    参数：
      ~input  (str): sensorsData topic（默认 /sensors_data_raw）
      ~output (str): JointState topic（默认 /joint_states）
      ~joint_map (list): [{name: <joint_name>, index: <int>}, ...]
      ~publish_waist_joint (bool): 是否额外发布 waist
      ~waist_joint (str): waist 关节名
      ~waist_joint_index (int): waist 在 joint_q 中索引（默认 12）
      ~rate_hz (float): 发布频率（默认 50）
    """

    def __init__(self):
        if sensorsData is None:
            raise RuntimeError("无法导入 kuavo_msgs/sensorsData，请先 source 工作空间。")

        self.topic_in = rospy.get_param("~input", "/sensors_data_raw")
        self.topic_out = rospy.get_param("~output", "/joint_states")
        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))

        self.publish_waist_joint = bool(rospy.get_param("~publish_waist_joint", False))
        self.waist_joint = rospy.get_param("~waist_joint", "waist_yaw_joint")
        self.waist_joint_index = int(rospy.get_param("~waist_joint_index", 12))

        joint_map_raw = rospy.get_param("~joint_map", [])
        self.joint_map = []
        for item in joint_map_raw:
            if not isinstance(item, dict):
                continue
            name = item.get("name", None)
            index = item.get("index", None)
            if name is None or index is None:
                continue
            self.joint_map.append((str(name), int(index)))

        if not self.joint_map and not self.publish_waist_joint:
            raise RuntimeError("joint_map 为空且 publish_waist_joint=false，桥接不会输出任何关节。")

        self._last_q = None
        self.pub = rospy.Publisher(self.topic_out, JointState, queue_size=10)
        self.sub = rospy.Subscriber(self.topic_in, sensorsData, self.cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.on_timer)

    def cb(self, msg: "sensorsData"):
        try:
            self._last_q = list(msg.joint_data.joint_q)
        except Exception:
            self._last_q = None

    def on_timer(self, _evt):
        q = self._last_q
        if q is None:
            return
        js = JointState()
        js.header.stamp = rospy.Time.now()

        names = []
        pos = []

        for name, idx in self.joint_map:
            if idx < 0 or idx >= len(q):
                continue
            names.append(name)
            pos.append(float(q[idx]))

        if self.publish_waist_joint and 0 <= self.waist_joint_index < len(q):
            names.append(self.waist_joint)
            pos.append(float(q[self.waist_joint_index]))

        if not names:
            return

        js.name = names
        js.position = pos
        js.velocity = [0.0] * len(names)
        js.effort = [0.0] * len(names)
        try:
            self.pub.publish(js)
        except rospy.ROSException:
            return


def main():
    rospy.init_node("sensors_joint_state_bridge")
    _bridge = SensorsJointStateBridge()
    rospy.loginfo("Bridging sensorsData -> JointState: %s -> %s", rospy.get_param("~input", "/sensors_data_raw"), rospy.get_param("~output", "/joint_states"))
    rospy.spin()


if __name__ == "__main__":
    main()

