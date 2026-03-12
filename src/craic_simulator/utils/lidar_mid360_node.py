#!/usr/bin/env python3
"""
LiDAR mid360 仿真节点：订阅 /mujoco/qpos，与当前仿真状态同步后做射线追踪，发布点云。
需先启动 MuJoCo 仿真（如 load_kuavo_mujoco_craic.launch），并确保场景 XML 中含 lidar_site（biped_s47 已包含）。
"""
from __future__ import division

import sys
import os
import traceback

# rosrun/roslaunch 有时未继承 pip 的 Python 路径，先尝试把工作空间内 MuJoCo-LiDAR 加入 path
_script_dir = os.path.dirname(os.path.abspath(__file__))
_ws_lib = os.path.dirname(_script_dir)
_devel = os.path.dirname(_ws_lib)
_ws_root = os.path.dirname(_devel)
_mujoco_lidar = os.path.join(_ws_root, "src", "MuJoCo-LiDAR")
if os.path.isdir(_mujoco_lidar) and _mujoco_lidar not in sys.path:
    sys.path.insert(0, _mujoco_lidar)

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as R

# MuJoCo & MuJoCo-LiDAR（需安装或工作空间内含 src/MuJoCo-LiDAR）
try:
    import mujoco
    from mujoco_lidar import MjLidarWrapper
    from mujoco_lidar import scan_gen
except ImportError:
    print("lidar_mid360: 缺少 mujoco/mujoco_lidar。请在工作空间内保留 src/MuJoCo-LiDAR，或执行: pip install 'mujoco-lidar[taichi]'", file=sys.stderr)
    traceback.print_exc(file=sys.stderr)
    sys.exit(1)


def _publish_point_cloud(pub, points, frame_id, stamp=None):
    if stamp is None:
        stamp = rospy.Time.now()
    if points is None or points.size == 0:
        return
    if len(points.shape) == 2 and points.shape[1] == 3:
        pts = points
    else:
        return
    intensity = np.ones((pts.shape[0], 1), dtype=np.float32)
    pts_with_i = np.hstack([pts, intensity])
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
    ]
    msg = pc2.create_cloud(
        rospy.Header(stamp=stamp, frame_id=frame_id),
        fields,
        pts_with_i,
    )
    pub.publish(msg)


def main():
    rospy.init_node("lidar_mid360_node", anonymous=False)

    # 场景路径：与 MuJoCo 仿真一致（由 launch 设置 legged_robot_scene_param）
    scene_path = rospy.get_param(
        "legged_robot_scene_param",
        rospy.get_param(
            "scene_path",
            None,
        ),
    )
    if not scene_path:
        # 若未设置则用 craic_simulator 默认场景
        import os
        pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        scene_path = os.path.join(
            pkg_path,
            "..",
            "models",
            "biped_s47",
            "xml",
            "scene.xml",
        )
        scene_path = os.path.normpath(os.path.abspath(scene_path))
        if not os.path.isfile(scene_path):
            rospy.logerr(
                "lidar_mid360: 未找到场景 XML。请先启动 MuJoCo 仿真或设置 legged_robot_scene_param。"
            )
            sys.exit(1)

    rospy.loginfo("lidar_mid360: 加载场景 %s", scene_path)
    try:
        mj_model = mujoco.MjModel.from_xml_path(scene_path)
        mj_data = mujoco.MjData(mj_model)
    except Exception as e:
        rospy.logerr("lidar_mid360: 加载场景失败: %s", e)
        sys.exit(1)

    site_name = "lidar_site"
    # 检查 site 是否存在（使用 mj_data.site 访问，若不存在会抛异常）
    try:
        _ = mj_data.site(site_name)
    except Exception:
        rospy.logerr("lidar_mid360: 场景中未找到 site '%s'，请使用含 lidar_site 的模型（如 biped_s47）", site_name)
        sys.exit(1)

    # 后端选择：cpu（稳定）或 taichi（GPU，需兼容的 taichi/tibvh/pytorch 版本）
    backend = rospy.get_param("~backend", "cpu")
    cutoff_dist = rospy.get_param("~cutoff_dist", 50.0)

    # 排除机器人自身的方式：
    # 方式 1: bodyexclude（只能排除单个 body）
    # 方式 2: geomgroup（推荐，可排除整个机器人）
    #   - 机器人自身 geom 通常是 group=1（视觉），场景障碍物是 group=0
    #   - geomgroup 数组：索引 i 为 1 表示检测 group=i 的 geom
    use_geomgroup = rospy.get_param("~use_geomgroup", True)
    if use_geomgroup:
        # 只检测 group=0 的 geom（场景/障碍物），排除 group=1-5（机器人自身）
        geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        lidar_args = {"geomgroup": geomgroup}
        rospy.loginfo("lidar_mid360: 使用 geomgroup 过滤，只检测 group=0 的 geom")
    else:
        # 使用 bodyexclude（只排除 base_link）
        try:
            base_body_id = mj_model.body("base_link").id
        except Exception:
            base_body_id = -1
        lidar_args = {"bodyexclude": base_body_id}
        rospy.loginfo("lidar_mid360: 使用 bodyexclude=%d", base_body_id)

    lidar = MjLidarWrapper(
        mj_model,
        site_name=site_name,
        backend=backend,
        cutoff_dist=cutoff_dist,
        args=lidar_args,
    )
    livox = scan_gen.LivoxGenerator("mid360")

    pub = rospy.Publisher(
        "/lidar/points",
        PointCloud2,
        queue_size=1,
    )
    # 雷达坐标系名称（点云发布在此坐标系下，需与 URDF 中的 link 名称一致）
    lidar_frame = rospy.get_param("~lidar_frame", "lidar")
    rate_hz = rospy.get_param("~rate", 10.0)
    last_qpos = None
    nq = mj_model.nq

    # TF 广播器（可选，默认禁用，因为 radar frame 已通过 URDF 链路发布）
    publish_tf = rospy.get_param("~publish_tf", False)
    tf_broadcaster = None
    parent_frame = None
    if publish_tf:
        tf_broadcaster = tf2_ros.TransformBroadcaster()
        # 父坐标系（仅在 publish_tf=True 时使用）
        parent_frame = rospy.get_param("~parent_frame", "odom")

    def on_qpos(msg):
        nonlocal last_qpos
        if len(msg.data) >= nq:
            last_qpos = np.array(msg.data[:nq], dtype=np.float64)

    rospy.Subscriber("/mujoco/qpos", Float64MultiArray, on_qpos, queue_size=1)

    rate = rospy.Rate(rate_hz)
    rospy.loginfo("lidar_mid360: 使用 mid360，site=%s，%g Hz，topic=/lidar/points，frame=%s，publish_tf=%s", 
                  site_name, rate_hz, lidar_frame, publish_tf)

    while not rospy.is_shutdown():
        # ROS 1 的回调在后台自动处理，无需手动 spin_once
        if last_qpos is not None:
            mj_data.qpos[:] = last_qpos
            mujoco.mj_forward(mj_model, mj_data)

            # mid360 每帧采样（非重复扫描）
            rays_theta, rays_phi = livox.sample_ray_angles()
            rays_theta = np.ascontiguousarray(rays_theta.astype(np.float32))
            rays_phi = np.ascontiguousarray(rays_phi.astype(np.float32))
            # trace_rays 会更新 lidar.sensor_position 和 lidar.sensor_rotation
            lidar.trace_rays(mj_data, rays_theta, rays_phi)
            points_local = lidar.get_hit_points()  # (N, 3) 雷达局部坐标系

            # 广播 TF: parent_frame -> lidar_frame（可选，默认禁用）
            stamp = rospy.Time.now()
            if tf_broadcaster is not None:
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = parent_frame
                t.child_frame_id = lidar_frame
                # 雷达位置
                t.transform.translation.x = float(lidar.sensor_position[0])
                t.transform.translation.y = float(lidar.sensor_position[1])
                t.transform.translation.z = float(lidar.sensor_position[2])
                # 雷达姿态（旋转矩阵 -> 四元数）
                quat = R.from_matrix(lidar.sensor_rotation).as_quat()  # [x, y, z, w]
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                tf_broadcaster.sendTransform(t)

            # 发布雷达局部坐标系的点云，通过 TF 变换到世界坐标系
            if points_local is not None and points_local.size > 0:
                _publish_point_cloud(pub, points_local, lidar_frame, stamp)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception:
        traceback.print_exc()
        sys.exit(1)
