#!/usr/bin/env python3
"""
MuJoCo 可视化 PICO 重定向 bin/npz 文件

用法:
  # 使用 npz 文件（推荐，包含世界坐标 root 位姿）
  python visualize_retarget_bin.py --npz ~/pico_recordings/pico_data_xxx_retarget.npz

  # 使用 bin 文件（root 位置仅有高度，xy 固定为 0）
  python visualize_retarget_bin.py --bin ~/pico_recordings/pico_data_xxx_retarget.bin

  # 指定播放帧率
  python visualize_retarget_bin.py --npz xxx.npz --fps 50

  # 录制视频
  python visualize_retarget_bin.py --npz xxx.npz --record output.mp4

数据格式 (77维 float32):
  [0]       : h (机身高度)
  [1-6]     : theta (旋转矩阵前2列 6D, column-major: R00,R10,R20,R01,R11,R21)
  [7-12]    : v (机身速度 6D)
  [13-38]   : q (关节位置 26D, 无 waist 无 head)
  [39-64]   : q_dot (关节速度 26D)
  [65-76]   : p (末端位置 12D)

npz 额外包含:
  root: [N, 7] 世界坐标位姿 (x, y, z, qx, qy, qz, qw)
"""

import argparse
import sys
import os
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

# 添加上级目录到 path，以便导入 kuavo_gmr
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, ".."))

from kuavo_gmr.params import ROBOT_XML_DICT


def rotation_6d_to_quat_wxyz(theta_6d):
    """
    将 6D 旋转表示转换为 MuJoCo 格式四元数 (w, x, y, z)
    
    输入 theta_6d: [R00, R10, R20, R01, R11, R21] (column-major, 前2列)
    """
    # 恢复前两列
    col0 = np.array(theta_6d[:3])
    col1 = np.array(theta_6d[3:6])
    
    # Gram-Schmidt 正交化
    col0 = col0 / (np.linalg.norm(col0) + 1e-8)
    col1 = col1 - np.dot(col1, col0) * col0
    col1 = col1 / (np.linalg.norm(col1) + 1e-8)
    
    # 叉积得到第三列
    col2 = np.cross(col0, col1)
    
    # 组装旋转矩阵
    rot_mat = np.column_stack([col0, col1, col2])
    
    # 转四元数 (scipy 输出 xyzw)
    quat_xyzw = R.from_matrix(rot_mat).as_quat()
    # 转 MuJoCo 格式 wxyz
    quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
    
    return quat_wxyz


def load_data(args):
    """
    加载 npz 或 bin 文件，返回 (vmp_data, root_poses)
    
    Returns:
        vmp_data: [N, 77] float32
        root_poses: [N, 7] (x, y, z, qx, qy, qz, qw) 世界坐标，或 None
    """
    if args.npz:
        data = np.load(args.npz)
        vmp_data = data['vmp_data']
        root_poses = data['root'] if 'root' in data else None
        print(f"加载 npz: {args.npz}")
        print(f"  vmp_data: {vmp_data.shape}")
        if root_poses is not None:
            print(f"  root: {root_poses.shape}")
        return vmp_data, root_poses
    
    elif args.bin:
        raw = np.fromfile(args.bin, dtype=np.float32)
        n_frames = len(raw) // 77
        if len(raw) % 77 != 0:
            print(f"⚠ 文件大小 {len(raw)} 不是 77 的整数倍，截断处理")
        vmp_data = raw[:n_frames * 77].reshape(n_frames, 77)
        print(f"加载 bin: {args.bin}")
        print(f"  帧数: {n_frames}, 形状: {vmp_data.shape}")
        return vmp_data, None
    
    else:
        print("错误: 请指定 --npz 或 --bin 文件")
        sys.exit(1)


def vmp_frame_to_qpos(frame_77d, root_pose=None):
    """
    将 77 维 VMP 数据转换为 MuJoCo qpos
    
    MuJoCo kuavo_s45 qpos layout (35维):
      [0:3]   root position (x, y, z)
      [3:7]   root quaternion (w, x, y, z)  -- MuJoCo scalar-first
      [7:13]  left leg joints (6)
      [13:19] right leg joints (6)
      [19:26] left arm joints (7)
      [26:33] right arm joints (7)
      [33:35] head joints (2) -- bin 文件中无此数据，置 0
    
    VMP 77维 joint 布局 (26 DOF):
      [13:19]  left leg (6)
      [19:25]  right leg (6)
      [25:32]  left arm (7)
      [32:39]  right arm (7)
    """
    qpos = np.zeros(35, dtype=np.float64)
    
    # Root position & orientation
    if root_pose is not None:
        # 使用 npz 中记录的世界坐标 root 位姿
        qpos[0] = root_pose[0]  # x
        qpos[1] = root_pose[1]  # y
        qpos[2] = root_pose[2]  # z
        # root_pose 中四元数是 xyzw，转为 wxyz
        qpos[3] = root_pose[6]  # w
        qpos[4] = root_pose[3]  # x
        qpos[5] = root_pose[4]  # y
        qpos[6] = root_pose[5]  # z
    else:
        # 仅有 bin 文件时，从 77 维数据恢复
        qpos[0] = 0.0           # x 未知
        qpos[1] = 0.0           # y 未知
        qpos[2] = frame_77d[0]  # h = 机身高度
        quat_wxyz = rotation_6d_to_quat_wxyz(frame_77d[1:7])
        qpos[3:7] = quat_wxyz
    
    # Joint positions (26 DOF → 28 DOF MuJoCo slots)
    joint_pos_26 = frame_77d[13:39]
    
    # 左腿 6 joints
    qpos[7:13] = joint_pos_26[0:6]
    # 右腿 6 joints
    qpos[13:19] = joint_pos_26[6:12]
    # 左臂 7 joints
    qpos[19:26] = joint_pos_26[12:19]
    # 右臂 7 joints
    qpos[26:33] = joint_pos_26[19:26]
    # 头部 2 joints (bin 中无数据，保持 0)
    qpos[33:35] = 0.0
    
    return qpos


def main():
    parser = argparse.ArgumentParser(
        description='MuJoCo 可视化 PICO 重定向 bin/npz 文件',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python visualize_retarget_bin.py --npz ~/pico_recordings/pico_data_xxx_retarget.npz
  python visualize_retarget_bin.py --bin ~/pico_recordings/pico_data_xxx_retarget.bin
  python visualize_retarget_bin.py --npz xxx.npz --fps 30 --record output.mp4
        """)
    
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--npz', type=str, help='npz 文件路径（推荐，含世界坐标 root 位姿）')
    group.add_argument('--bin', type=str, help='bin 文件路径（仅 77 维 VMP 数据）')
    
    parser.add_argument('--robot', type=str, default='kuavo_s45',
                        choices=list(ROBOT_XML_DICT.keys()),
                        help='机器人类型 (默认: kuavo_s45)')
    parser.add_argument('--fps', type=int, default=100,
                        help='播放帧率 (默认: 100, 与录制频率一致)')
    parser.add_argument('--loop', action='store_true',
                        help='循环播放')
    parser.add_argument('--start', type=int, default=0,
                        help='起始帧 (默认: 0)')
    parser.add_argument('--end', type=int, default=None,
                        help='结束帧 (默认: 最后一帧)')
    parser.add_argument('--stride', type=int, default=1,
                        help='帧步长 (默认: 1)')
    parser.add_argument('--record', type=str, default=None,
                        help='录制视频到指定路径 (如 output.mp4)')
    
    args = parser.parse_args()
    
    # 加载数据
    vmp_data, root_poses = load_data(args)
    n_frames = len(vmp_data)
    
    # 帧范围
    start = max(0, args.start)
    end = min(n_frames, args.end) if args.end else n_frames
    stride = max(1, args.stride)
    frame_indices = list(range(start, end, stride))
    
    print(f"\n播放设置:")
    print(f"  帧范围: {start} → {end} (步长 {stride})")
    print(f"  有效帧数: {len(frame_indices)}")
    print(f"  帧率: {args.fps} FPS")
    print(f"  预计时长: {len(frame_indices) / args.fps:.1f} 秒")
    print(f"  循环: {'是' if args.loop else '否'}")
    if root_poses is None:
        print(f"  ⚠ 无世界坐标 root 数据，机器人将原地播放 (xy=0)")
    print()
    
    # 加载 MuJoCo 模型
    import mujoco as mj
    import mujoco.viewer as mjv
    
    xml_path = str(ROBOT_XML_DICT[args.robot])
    print(f"加载模型: {xml_path}")
    model = mj.MjModel.from_xml_path(xml_path)
    data = mj.MjData(model)
    
    print(f"模型 nq={model.nq}, nv={model.nv}")
    assert model.nq == 35, f"期望 nq=35 (kuavo_s45), 实际 nq={model.nq}"
    
    # 视频录制
    mp4_writer = None
    renderer = None
    if args.record:
        import imageio
        video_dir = os.path.dirname(args.record)
        if video_dir and not os.path.exists(video_dir):
            os.makedirs(video_dir)
        mp4_writer = imageio.get_writer(args.record, fps=args.fps)
        renderer = mj.Renderer(model, height=720, width=1280)
        print(f"录制视频到: {args.record}")
    
    # 启动 viewer
    viewer = mjv.launch_passive(
        model=model,
        data=data,
        show_left_ui=False,
        show_right_ui=False,
    )
    
    frame_interval = 1.0 / args.fps
    
    print("开始播放... (关闭窗口退出)")
    
    try:
        playing = True
        while playing and viewer.is_running():
            for idx, fi in enumerate(frame_indices):
                if not viewer.is_running():
                    playing = False
                    break
                
                t_start = time.time()
                
                # 转换为 qpos
                root = root_poses[fi] if root_poses is not None else None
                qpos = vmp_frame_to_qpos(vmp_data[fi], root)
                
                # 设置 MuJoCo 状态
                data.qpos[:] = qpos
                mj.mj_forward(model, data)
                
                # 相机跟随
                base_body_id = model.body("base_link").id
                viewer.cam.lookat[:] = data.xpos[base_body_id]
                viewer.cam.distance = 3.0
                viewer.cam.elevation = -15
                
                viewer.sync()
                
                # 录制
                if mp4_writer is not None:
                    renderer.update_scene(data, camera=viewer.cam)
                    img = renderer.render()
                    mp4_writer.append_data(img)
                
                # 帧率控制
                elapsed = time.time() - t_start
                sleep_time = frame_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                # 进度显示
                if (idx + 1) % 100 == 0 or idx == len(frame_indices) - 1:
                    progress = (idx + 1) / len(frame_indices) * 100
                    print(f"\r  播放进度: {idx+1}/{len(frame_indices)} ({progress:.0f}%) "
                          f"帧 {fi}/{n_frames}", end="", flush=True)
            
            print()
            if not args.loop:
                print("播放完成！窗口保持打开，关闭窗口退出。")
                while viewer.is_running():
                    time.sleep(0.1)
                break
            else:
                print("循环播放...")
    
    except KeyboardInterrupt:
        print("\n用户中断")
    
    finally:
        viewer.close()
        if mp4_writer is not None:
            mp4_writer.close()
            print(f"视频已保存: {args.record}")
    
    print("完成")


if __name__ == '__main__':
    main()
