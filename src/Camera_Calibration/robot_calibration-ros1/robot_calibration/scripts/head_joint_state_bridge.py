#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
import math

try:
    from kuavo_msgs.msg import sensorsData
except Exception as e:
    sensorsData = None


class HeadJointStateBridge:
    def __init__(self):
        if sensorsData is None:
            raise RuntimeError(
                "无法导入 kuavo_msgs/sensorsData。请先 source 工作空间的 devel/setup.bash。"
            )

        # /sensors_data_raw: kuavo_msgs/sensorsData，joint_data.joint_q 的最后两位是 head_yaw/head_pitch
        self.topic_in = rospy.get_param("~input", "/sensors_data_raw")
        self.topic_out = rospy.get_param("~output", "/joint_states")

        self.yaw_joint = rospy.get_param("~yaw_joint", "zhead_1_joint")
        self.pitch_joint = rospy.get_param("~pitch_joint", "zhead_2_joint")
        self.waist_joint = rospy.get_param("~waist_joint", "waist_yaw_joint")
        # joint_q 索引约定在其它模块里常用：waist_yaw 对应 joint_q[12]
        self.waist_joint_index = int(rospy.get_param("~waist_joint_index", 12))
        self.publish_waist_joint = bool(rospy.get_param("~publish_waist_joint", True))

        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))

        # debug（默认关闭，避免实时刷屏；需要时手动设为 true）
        self.debug = bool(rospy.get_param("~debug", False))
        self.debug_print_period = float(rospy.get_param("~debug_print_period", 1.0))
        self._last_debug_t = 0.0
        self._seq = 0

        self._last_yaw = None
        self._last_pitch = None
        self._last_waist_yaw = None
        self._last_q_tail = None
        self._last_msg_stamp = None

        self.pub = rospy.Publisher(self.topic_out, JointState, queue_size=10)
        self.sub = rospy.Subscriber(self.topic_in, sensorsData, self.cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.on_timer)

    def cb(self, msg: "sensorsData"):
        q = list(msg.joint_data.joint_q)
        if len(q) < 2:
            return
        # 文档约定：最后两位分别为 head_yaw、head_pitch（rad）
        self._last_yaw = float(q[-2])
        self._last_pitch = float(q[-1])
        self._last_q_tail = (self._last_yaw, self._last_pitch)
        if self.publish_waist_joint and len(q) > self.waist_joint_index:
            self._last_waist_yaw = float(q[self.waist_joint_index])
        else:
            self._last_waist_yaw = None
        # sensorsData 通常没有标准 header，这里仅记录接收时刻用于排查“不同步”
        self._last_msg_stamp = rospy.Time.now()

    def on_timer(self, _evt):
        # 持续发布 /joint_states，避免采样瞬间拿不到最新 JointState 导致空数据
        if self._last_yaw is None or self._last_pitch is None:
            return
        if rospy.is_shutdown():
            return
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = [self.yaw_joint, self.pitch_joint]
        js.position = [self._last_yaw, self._last_pitch]
        # robot_calibration 的 ChainManager 要求 position/velocity 数组长度一致
        js.velocity = [0.0, 0.0]
        js.effort = [0.0, 0.0]
        if self._last_waist_yaw is not None:
            js.name.append(self.waist_joint)
            js.position.append(self._last_waist_yaw)
            js.velocity.append(0.0)
            js.effort.append(0.0)
        try:
            self.pub.publish(js)
        except rospy.ROSException:
            # 节点退出/roslaunch 关闭时 publisher 可能已关闭，避免抛异常刷屏
            return

        # Debug printing to verify FK input is correct (order/unit/sync).
        if self.debug:
            now_s = rospy.get_time()
            if (now_s - self._last_debug_t) >= self.debug_print_period:
                self._last_debug_t = now_s
                yaw_deg = self._last_yaw * 180.0 / math.pi
                pitch_deg = self._last_pitch * 180.0 / math.pi
                waist_deg = None if self._last_waist_yaw is None else self._last_waist_yaw * 180.0 / math.pi
                age_ms = None
                if self._last_msg_stamp is not None:
                    age_ms = (js.header.stamp - self._last_msg_stamp).to_sec() * 1000.0
                rospy.loginfo(
                    "[bridge dbg] seq=%d pub_stamp=%.3f q_tail(rad)=[%.6f, %.6f] -> %s=%.6f rad (%.2f deg), %s=%.6f rad (%.2f deg)%s age_ms=%s",
                    self._seq,
                    js.header.stamp.to_sec(),
                    self._last_q_tail[0] if self._last_q_tail else float("nan"),
                    self._last_q_tail[1] if self._last_q_tail else float("nan"),
                    self.yaw_joint,
                    self._last_yaw,
                    yaw_deg,
                    self.pitch_joint,
                    self._last_pitch,
                    pitch_deg,
                    (", %s=%.6f rad (%.2f deg)" % (self.waist_joint, self._last_waist_yaw, waist_deg)) if self._last_waist_yaw is not None else "",
                    ("%.1f" % age_ms) if age_ms is not None else "NA",
                )
            self._seq += 1


def main():
    rospy.init_node("head_joint_state_bridge")
    bridge = HeadJointStateBridge()
    rospy.loginfo(
        "Bridging sensorsData(joint_q[-2:]) -> JointState: %s -> %s",
        rospy.get_param("~input", "/sensors_data_raw"),
        rospy.get_param("~output", "/joint_states"),
    )
    rospy.spin()


if __name__ == "__main__":
    main()

