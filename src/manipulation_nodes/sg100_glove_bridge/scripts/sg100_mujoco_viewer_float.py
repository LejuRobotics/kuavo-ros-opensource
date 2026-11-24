
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SG100 MuJoCo Viewer.

订阅：
    /sg100_sim_hand_command        右手 11 维
    /sg100_sim_left_hand_command   左手 11 维
    std_msgs/Float32MultiArray

左右手放在同一个窗口里，各自跟随对应话题。
没有新命令的那只手保持最后一帧。
不向真实机器人发送控制指令。
"""

import os
import re
import threading
import time

import numpy as np
import rospy

from std_msgs.msg import Float32MultiArray


JOINTS = [
    "thumb_j1",
    "thumb_j2",
    "thumb_j3",
    "index_j1",
    "index_j2",
    "index_j3",
    "middle_j1",
    "middle_j2",
    "little_j1",
    "little_j2",
    "little_j3",
]


# 两只手 URDF 里重名、合并进同一模型前必须加前缀的坐标系。
_SHARED_FRAME_NAMES = (
    "sg100_retarget_tip_thumb_joint",
    "sg100_retarget_tip_index_joint",
    "sg100_retarget_tip_middle_joint",
    "sg100_retarget_tip_little_joint",
    "sg100_retarget_tip_thumb",
    "sg100_retarget_tip_index",
    "sg100_retarget_tip_middle",
    "sg100_retarget_tip_little",
    "retarget_palm_joint",
    "retarget_palm",
)

# 左右手在同一个窗口里的平移，单位米。
_HAND_OFFSETS = {
    "left": (-0.18, 0.0, 0.12),
    "right": (0.18, 0.0, 0.12),
}


def _load_mujoco_helpers():
    """
    加载现有 MuJoCo URDF 工具。
    """

    try:
        from mujoco_viz import (
            _inject_meshdir,
            apply_joint_positions,
        )

        return (
            apply_joint_positions,
            _inject_meshdir,
        )

    except ImportError:
        pass

    try:
        import rospkg

        root = rospkg.RosPack().get_path(
            "motion_capture_ik"
        )

    except Exception as exc:
        raise RuntimeError(
            "Cannot locate motion_capture_ik "
            "MuJoCo helper package"
        ) from exc

    helper_dir = os.path.join(
        root,
        "scripts",
        "tools",
        "lingqiaozhineng",
        "our-retargeting",
        "src",
    )

    if helper_dir not in os.sys.path:
        os.sys.path.insert(0, helper_dir)

    from mujoco_viz import (
        _inject_meshdir,
        apply_joint_positions,
    )

    return (
        apply_joint_positions,
        _inject_meshdir,
    )


def _name_materials(xml, side):
    """给空材质名编号，避免 MuJoCo 把多个 geom 都叫做空字符串。"""

    counter = 0

    def repl(match):
        nonlocal counter
        counter += 1
        return '%sname="%s_mat_%d"' % (
            match.group(1),
            side,
            counter,
        )

    return re.sub(
        r'(<material\s+)name=""',
        repl,
        xml,
    )


def _name_unnamed_elements(xml, side):
    """给没有 name 的 visual/collision 起名，消掉重复空 geom 警告。"""

    counter = 0

    def repl(match):
        nonlocal counter
        counter += 1
        return '<%s name="%s_%s_%d"' % (
            match.group(1),
            side,
            match.group(1),
            counter,
        )

    return re.sub(
        r"<(visual|collision)(?![^>]*\bname=)",
        repl,
        xml,
    )


def _prefix_shared_frames(xml, side):
    """左右手共用的 retarget 坐标系加前缀，合并模型时才不会重名。"""

    for name in _SHARED_FRAME_NAMES:
        xml = xml.replace(
            'name="%s"' % name,
            'name="%s_%s"' % (side, name),
        )
        xml = xml.replace(
            'link="%s"' % name,
            'link="%s_%s"' % (side, name),
        )
    return xml


def _load_both_hands(
    mujoco,
    inject_meshdir,
    urdf_dir,
    mesh_dir,
):
    """把左右手合成一个模型。MuJoCo 同一进程只能开一个被动窗口。"""

    scene = mujoco.MjSpec()
    scene.modelname = "sg100_hands"
    # 父模型默认把角度当 degree。URDF 限位是弧度，不关掉会被缩小约 57 倍。
    scene.degree = False

    for side in ("left", "right"):
        urdf = os.path.join(
            urdf_dir,
            "heiman_%s_hand.urdf" % side,
        )
        with open(urdf, encoding="utf-8") as stream:
            xml = stream.read()
        xml = _name_materials(xml, side)
        xml = _name_unnamed_elements(xml, side)
        xml = _prefix_shared_frames(xml, side)
        xml = inject_meshdir(xml, mesh_dir)

        spec = mujoco.MjSpec()
        spec.from_string(xml)
        root = spec.worldbody.first_body()
        if root is None:
            raise RuntimeError(
                "URDF has no root body: %s" % urdf
            )

        frame = scene.worldbody.add_frame()
        frame.name = "%s_mount" % side
        frame.pos[:] = _HAND_OFFSETS[side]
        frame.attach_body(root, "", "")

    model = scene.compile()
    return model, mujoco.MjData(model)


class SG100MuJoCoViewer:

    def __init__(self):

        import mujoco
        import mujoco.viewer
        import rospkg

        self.mujoco = mujoco

        (
            self.apply_joint_positions,
            inject_meshdir,
        ) = _load_mujoco_helpers()

        asset_root = rospkg.RosPack().get_path(
            "kuavo_assets"
        )

        urdf_dir = os.path.join(
            asset_root,
            "models",
            "heiman_sg100",
            "urdf",
        )

        mesh_dir = os.path.join(
            asset_root,
            "models",
            "heiman_sg100",
            "meshes",
        )

        self.joint_map = {}

        self.lock = threading.Lock()

        self.latest = {
            "left": np.zeros(
                11, dtype=np.float64
            ),
            "right": np.zeros(
                11, dtype=np.float64
            ),
        }

        self.last_msg_time = {
            "left": time.monotonic(),
            "right": time.monotonic(),
        }

        self.timeout = float(
            rospy.get_param(
                "~command_timeout",
                1.0,
            )
        )

        self.command_topics = {
            "right": rospy.get_param(
                "~command_topic",
                "/sg100_sim_hand_command",
            ),
            "left": rospy.get_param(
                "~left_command_topic",
                "/sg100_sim_left_hand_command",
            ),
        }

        for side, prefix in (
            ("left", "l_"),
            ("right", "r_"),
        ):
            self.joint_map[side] = [
                prefix + name
                for name in JOINTS
            ]

        # 一个模型、一个窗口。两个 launch_passive 会在 GLFW 里段错误。
        self.model, self.data = _load_both_hands(
            mujoco,
            inject_meshdir,
            urdf_dir,
            mesh_dir,
        )

        zero_positions = {}
        for names in self.joint_map.values():
            zero_positions.update(
                dict.fromkeys(names, 0.0)
            )
        self.apply_joint_positions(
            self.model,
            self.data,
            zero_positions,
            clip=True,
        )

        self.viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
        )

        rospy.loginfo(
            "[sg100_mujoco] loaded both hands from %s",
            urdf_dir,
        )

        # 左右手各订一条 11 维话题，避免混在同一个数组里。
        self.subscribers = []
        for side, topic in self.command_topics.items():
            self.subscribers.append(
                rospy.Subscriber(
                    topic,
                    Float32MultiArray,
                    self.on_command,
                    callback_args=side,
                    queue_size=1,
                )
            )
            rospy.loginfo(
                "[sg100_mujoco] subscribing %s -> %s",
                topic,
                side,
            )

    def on_command(self, msg, side):
        """
        接收一只手的 11 维关节角。
        """

        q = np.asarray(
            msg.data,
            dtype=np.float64,
        )

        if (
            q.shape != (11,)
            or not np.all(np.isfinite(q))
        ):
            rospy.logwarn_throttle(
                2.0,
                "[sg100_mujoco] %s expected "
                "11 finite joint values",
                side,
            )
            return

        with self.lock:

            self.latest[side][:] = q

            self.last_msg_time[side] = (
                time.monotonic()
            )

    def run(self):
        """
        60 Hz 更新 MuJoCo 模型。
        """

        # /use_sim_time 为真且没有 /clock 时，rospy.Rate 会一直等仿真时间。
        period = 1.0 / 60.0
        next_tick = time.monotonic()

        try:
            while not rospy.is_shutdown():

                with self.lock:

                    poses = {
                        side: q.copy()
                        for side, q
                        in self.latest.items()
                    }

                    now = time.monotonic()
                    stale = [
                        side
                        for side, stamp
                        in self.last_msg_time.items()
                        if now - stamp > self.timeout
                    ]

                if stale:

                    rospy.logwarn_throttle(
                        5.0,
                        "[sg100_mujoco] "
                        "no fresh command for %s; "
                        "holding pose",
                        ",".join(stale),
                    )

                for side in (
                    "left",
                    "right",
                ):

                    joint_positions = dict(
                        zip(
                            self.joint_map[side],
                            poses[side],
                        )
                    )

                    self.apply_joint_positions(
                        self.model,
                        self.data,
                        joint_positions,
                        clip=True,
                    )

                self.mujoco.mj_forward(
                    self.model,
                    self.data,
                )

                if self.viewer.is_running():
                    self.viewer.sync()
                else:
                    return

                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()

        finally:

            self.viewer.close()


def main():

    rospy.init_node(
        "sg100_mujoco_viewer"
    )

    viewer = SG100MuJoCoViewer()

    viewer.run()


if __name__ == "__main__":
    main()
