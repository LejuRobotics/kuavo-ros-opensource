import os
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from utils.ros_control_utils import fk_srv_client
import glob
import rospy
import atexit
from datetime import datetime
from kuavo_msgs.msg import sensorsData

def mocap_to_world(pos, quat):
    rot_x_90 = R.from_euler('x', 90, degrees=True).as_quat()
    r_mocap = R.from_quat(quat)
    r_x90 = R.from_quat(rot_x_90)
    r_world = r_x90 * r_mocap
    quat_world = r_world.as_quat()
    pos_world = r_x90.apply(pos)
    return pos_world.tolist(), quat_world.tolist()

def quaternion_mean(quats):
    if not quats:
        return None
    Q = np.array(quats, dtype=float)
    if Q.ndim != 2 or Q.shape[1] != 4:
        return None
    norms = np.linalg.norm(Q, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    Q = Q / norms
    A = Q.T @ Q
    eigvals, eigvecs = np.linalg.eigh(A)
    mean_q = eigvecs[:, np.argmax(eigvals)]
    if mean_q[-1] < 0:
        mean_q = -mean_q
    mean_q = mean_q / np.linalg.norm(mean_q)
    return mean_q.tolist()

def extract_topics_in_time_range(bag_path, start_time, end_time, target_topics):
    import rosbag
    import rospy
    bag = rosbag.Bag(bag_path)
    result_data = {
        "bag_file": os.path.basename(bag_path),
        "start_time": start_time,
        "end_time": end_time,
        "topics": {}
    }
    raw_data = {t: [] for t in target_topics}

    RAW_POS_MAX = 1e6  # 对应 1000 m，判断动捕丢点

    for topic, msg, t in bag.read_messages(start_time=rospy.Time(start_time), end_time=rospy.Time(end_time)):
        if topic not in target_topics:
            continue
        if topic in ['/righthand_pose', '/lefthand_pose', '/base_pose']:
            # 动捕丢点，直接弃用该窗口
            try:
                px = float(msg.position.x)
                py = float(msg.position.y)
                pz = float(msg.position.z)
            except Exception:
                bag.close()
                print(
                    "[extract_topics_in_time_range] skip window for bag "
                    f"{os.path.basename(bag_path)} [{start_time}, {end_time}] "
                    f"due to invalid mocap position message on topic {topic}"
                )
                return None

            arr_raw = np.array([px, py, pz], dtype=float)
            if (not np.all(np.isfinite(arr_raw))) or np.any(np.abs(arr_raw) > RAW_POS_MAX):
                bag.close()
                print(
                    "[extract_topics_in_time_range] skip window for bag "
                    f"{os.path.basename(bag_path)} [{start_time}, {end_time}] "
                    f"because mocap position too large or non-finite on topic {topic}, "
                    f"pos_raw={arr_raw.tolist()}"
                )
                return None

            raw_data[topic].append({
                "pos": [px/1000.0, py/1000.0, pz/1000.0],
                "quat": [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            })
        elif topic == '/sensors_data_raw':
            raw_data[topic].append({"pos": list(msg.joint_data.joint_q[12:26])})
        elif topic == '/mm_kuavo_arm_traj':
            try:
                raw_data[topic].append({"pos": list(msg.position)})
            except Exception:
                pass
        elif topic == '/kuavo_arm_traj':
            raw_data[topic].append({"pos": list(msg.position)})
        elif topic == '/humanoid_mpc_target_arm':
                try:
                    if len(msg.stateTrajectory) > 1:
                        mpc_state = msg.stateTrajectory[1]
                    elif len(msg.stateTrajectory) == 1:
                        mpc_state = msg.stateTrajectory[0]
                    else:
                        mpc_state = None
                    if mpc_state is not None:
                        joint_rad = list(getattr(mpc_state, 'value', []))
                        raw_data[topic].append({"pos": joint_rad})
                except Exception:
                    pass
        elif topic == '/joint_cmd':
            try:
                joints = list(getattr(msg, 'joint_q', []))
                raw_data[topic].append({"pos": list(joints[12:26])})
            except Exception:
                pass
    bag.close()
    for topic in ['/righthand_pose', '/lefthand_pose', '/base_pose']:
        poses = [entry['pos'] for entry in raw_data[topic] if 'pos' in entry]
        if not poses:
            continue
        arr = np.array(poses, dtype=float)
        mean_xyz = np.mean(arr, axis=0).tolist()
        quats = [entry['quat'] for entry in raw_data[topic] if 'quat' in entry]
        mean_quat = quaternion_mean(quats)
        if mean_quat is None:
            continue
        mean_xyz_world, mean_quat_world = mocap_to_world(mean_xyz, mean_quat)
        result_data['topics'][topic] = {
            'position_mean': mean_xyz_world,
            'quat_mean': mean_quat_world
        }
    for hand_topic in ['/lefthand_pose', '/righthand_pose']:
        base_topic = '/base_pose'
        if (hand_topic in result_data['topics'] and base_topic in result_data['topics']):
            hand_pos = result_data['topics'][hand_topic]['position_mean']
            base_pos = result_data['topics'][base_topic]['position_mean']
            hand_quat = result_data['topics'][hand_topic]['quat_mean']
            base_quat = result_data['topics'][base_topic]['quat_mean']
            if hand_pos is not None and base_pos is not None and hand_quat is not None and base_quat is not None:
                r_base = R.from_quat(base_quat)
                r_hand = R.from_quat(hand_quat)
                r_rel = r_base.inv() * r_hand
                quat_rel = r_rel.as_quat()
                pos_rel = (np.array(hand_pos) - np.array(base_pos))
                result_data['topics'][hand_topic+'_rel_base'] = {
                    'position_mean': pos_rel.tolist(),
                    'quat_mean': quat_rel.tolist(),
                }
    for topic in ['/sensors_data_raw', '/mm_kuavo_arm_traj', '/kuavo_arm_traj', '/humanoid_mpc_target_arm', '/joint_cmd']:
        poses = [entry['pos'] for entry in raw_data[topic] if 'pos' in entry]
        if not poses:
            continue
        arr = np.array(poses, dtype=float)
        mean_joints = np.mean(arr, axis=0).tolist()
        if topic in ['/kuavo_arm_traj', '/mm_kuavo_arm_traj']:
            joints_for_fk = np.deg2rad(mean_joints).tolist()
        else:
            joints_for_fk = mean_joints
        # 使用 fk 服务
        fk_srv = fk_srv_client
        hand_poses = fk_srv(joints_for_fk)
        result_data['topics'][topic] = {
            'joint_mean': joints_for_fk,
            'left_position_mean': hand_poses.left_pose.pos_xyz, 
            'right_position_mean': hand_poses.right_pose.pos_xyz,
            'left_quat_mean': hand_poses.left_pose.quat_xyzw,
            'right_quat_mean': hand_poses.right_pose.quat_xyzw
        }
    return result_data

def generate_position_tables(all_results, csv_dir):
    os.makedirs(csv_dir, exist_ok=True)
    rows = []
    for item in all_results or []:
        bag_file = item.get('bag_file')
        start_time = item.get('start_time')
        end_time = item.get('end_time')
        topics = item.get('topics', {})
        for topic, data in topics.items():
            if 'left_position_mean' in data or 'right_position_mean' in data:
                if 'left_position_mean' in data and data['left_position_mean'] is not None:
                    x, y, z = data['left_position_mean']
                    rows.append([bag_file, start_time, end_time, topic, 'left', x, y, z])
                if 'right_position_mean' in data and data['right_position_mean'] is not None:
                    x, y, z = data['right_position_mean']
                    rows.append([bag_file, start_time, end_time, topic, 'right', x, y, z])
            elif 'position_mean' in data and data['position_mean'] is not None:
                x, y, z = data['position_mean']
                t_lower = topic.lower()
                if 'lefthand' in t_lower:
                    hand = 'left'
                elif 'righthand' in t_lower:
                    hand = 'right'
                else:
                    hand = ''
                rows.append([bag_file, start_time, end_time, topic, hand, x, y, z])
    exclude_topics = {'/base_pose', '/lefthand_pose', '/righthand_pose'}
    rows = [r for r in rows if str(r[3]) not in exclude_topics]
    aligned_rows = sorted(rows, key=lambda r: (str(r[0]), str(r[3]), str(r[4])))
    return {'rows': aligned_rows}



def _load_keyframes(config_path: str):
    from utils.load_keyframes import load_keyframes
    try:
        return load_keyframes(config_path)
    except Exception:
        return []

def _match_kf_index_by_time(keyframes, t_val: float, tol: float = 1e-3):
    if not keyframes:
        return None
    for i, kf in enumerate(keyframes):
        try:
            if abs(float(kf['time']) - float(t_val)) <= tol:
                return i
        except Exception:
            continue
    return None

def _ref_pos_for_kf(keyframes, idx):
    try:
        if idx is None or idx < 0 or idx >= len(keyframes):
            return None
        lpos = keyframes[idx]['left']['pos']
        rpos = keyframes[idx]['right']['pos']
        return {
            'left': [float(lpos[0]), float(lpos[1]), float(lpos[2])],
            'right': [float(rpos[0]), float(rpos[1]), float(rpos[2])],
        }
    except Exception:
        return None

def _bag_infos_in_folder(bag_folder: str):
    import rosbag
    paths = sorted(glob.glob(os.path.join(bag_folder, '*.bag')))
    infos = []
    for p in paths:
        try:
            b = rosbag.Bag(p)
            b_start = float(b.get_start_time()) if hasattr(b, 'get_start_time') else None
            b_end = float(b.get_end_time()) if hasattr(b, 'get_end_time') else None
        finally:
            try:
                b.close()
            except Exception:
                pass
        infos.append((p, b_start, b_end))
    return infos

def _collect_labels(bag_folder: str, flag_topic: str, keyframes):
    import rosbag
    labels = []  # list of (t_label, kf_idx)
    bag_infos = _bag_infos_in_folder(bag_folder)
    for bag_path, _, _ in bag_infos:
        try:
            bag = rosbag.Bag(bag_path)
            for _, msg, t in bag.read_messages(topics=[flag_topic]):
                try:
                    fval = float(getattr(msg, 'data', None))
                except Exception:
                    fval = None
                if fval is None or fval <= 0.0:
                    continue
                kf_idx = _match_kf_index_by_time(keyframes, fval)
                if kf_idx is None:
                    continue
                labels.append((t.to_sec(), kf_idx))
        finally:
            try:
                bag.close()
            except Exception:
                pass
    labels.sort(key=lambda x: x[0])
    return bag_infos, labels

def _choose_bag_for_window(bag_infos, t_label, clipped_start, clipped_end):
    chosen = None
    for info in bag_infos:
        bag_path, b_start, b_end = info
        if b_start is None or b_end is None:
            continue
        if b_start - 1e-9 <= t_label <= b_end + 1e-9:
            chosen = info
            break
    if chosen is None:
        for info in bag_infos:
            bag_path, b_start, b_end = info
            if b_start is None or b_end is None:
                continue
            if not (b_end < clipped_start - 1e-9 or b_start > clipped_end + 1e-9):
                chosen = info
                break
    return chosen

def _bag_infos_in_folder(bag_folder: str):
    import rosbag
    paths = sorted(glob.glob(os.path.join(bag_folder, '*.bag')))
    infos = []
    for p in paths:
        try:
            b = rosbag.Bag(p)
            b_start = float(b.get_start_time()) if hasattr(b, 'get_start_time') else None
            b_end = float(b.get_end_time()) if hasattr(b, 'get_end_time') else None
        finally:
            try:
                b.close()
            except Exception:
                pass
        infos.append((p, b_start, b_end))
    return infos



def load_keyframes_config(config_path: str):
    """外部调用：加载关键帧配置（失败返回空列表）。"""
    return _load_keyframes(config_path)

def collect_keyframe_labels(bag_folder: str, flag_topic: str, keyframes):
    """返回 (bag_infos, labels)。labels: list[(t_label, kf_idx)] 按时间升序。"""
    bag_infos, labels = _collect_labels(bag_folder, flag_topic, keyframes)
    return bag_infos, labels

def compute_windows(bag_infos, labels, offset_sec: float, window_sec: float):
    """根据标签时间构造并裁剪窗口，返回 list[dict]，每个包含 bag_path/bag_start/bag_end/kf_idx。"""
    if not bag_infos or not labels:
        return []
    global_start = min(b[1] for b in bag_infos if b[1] is not None)
    global_end = max(b[2] for b in bag_infos if b[2] is not None)
    windows = []
    for i, (t_label, kf_idx) in enumerate(labels):
        start = t_label + offset_sec
        end = start + window_sec
        clipped_start = max(start, global_start)
        clipped_end = min(end, global_end)
        if clipped_end <= clipped_start:
            continue
        chosen = _choose_bag_for_window(bag_infos, t_label, clipped_start, clipped_end)
        if chosen is None:
            continue
        bag_path, b_start, b_end = chosen
        bag_start = max(clipped_start, b_start)
        bag_end = min(clipped_end, b_end)
        if bag_end <= bag_start:
            continue
        windows.append({
            'index': i,
            'bag_path': bag_path,
            'bag_start': bag_start,
            'bag_end': bag_end,
            'kf_idx': kf_idx,
            'label_time': t_label,
        })
    return windows

def extract_window_data(windows, target_topics: list, keyframes):
    """批量提取各窗口话题均值，返回 (results, refs, kf_indices)。"""
    results = []
    refs = []
    kf_indices = []
    for w in windows:
        res = extract_topics_in_time_range(
            w['bag_path'], w['bag_start'], w['bag_end'], target_topics
        )
        # 若在该窗口中检测到 mocap 异常数据，extract_topics_in_time_range 会返回 None；
        # 此时直接丢弃该窗口，不参与后续误差聚合。
        if res is None:
            continue
        ref = _ref_pos_for_kf(keyframes, w['kf_idx'])
        results.append(res)
        refs.append(ref)
        kf_indices.append(w['kf_idx'])
    return results, refs, kf_indices

def aggregate_window_errors(results, refs, kf_indices, keyframes, out_dir: str):
    """按关键帧聚合输出汇总 CSV（纯内存计算，无临时目录/中间文件），返回 CSV 路径。"""
    os.makedirs(out_dir, exist_ok=True)
    from collections import defaultdict
    import numpy as _np

    grouped = defaultdict(lambda: defaultdict(list))
    overall = defaultdict(list)

    # 用于记录关节角度误差（按关键帧 + 误差标签聚合，向量维度为关节数）
    joint_grouped = defaultdict(lambda: defaultdict(list))
    joint_overall = defaultdict(list)

    # 用于记录每个关键帧、每个话题的“原始”统计量（位置、关节）
    # 按 (hand, topic) 聚合，每个元素为 3 维 XYZ
    pos_vals_grouped = defaultdict(lambda: defaultdict(list))
    # 按 topic 聚合，每个元素为 N 维关节角向量
    joint_vals_grouped = defaultdict(lambda: defaultdict(list))

    T_MM      = '/mm_kuavo_arm_traj'
    T_KUAVO   = '/kuavo_arm_traj'
    T_MPC     = '/humanoid_mpc_target_arm'
    T_SENSOR  = '/sensors_data_raw'
    T_JCMD    = '/joint_cmd'
    T_LEFTREL = '/lefthand_pose_rel_base'
    T_RIGHTREL= '/righthand_pose_rel_base'

    def _kf_meta(idx):
        name = None; t = None
        try:
            if 0 <= idx < len(keyframes):
                t = float(keyframes[idx].get('time', None))
                name = keyframes[idx].get('name', None)
        except Exception:
            pass
        return name, t

    for res, ref, kf_idx in zip(results or [], refs or [], kf_indices or []):
        if ref is None:
            continue
        paths = generate_position_tables([res], out_dir)
        rows = paths.get('rows') or []
        by_topic = {'left': {}, 'right': {}}
        for r in rows:
            try:
                _, _, _, topic, hand, x, y, z = r
                x = float(x); y = float(y); z = float(z)
            except Exception:
                continue
            if hand not in ('left','right'):
                continue
            by_topic[hand].setdefault(topic, []).append([x, y, z])

        means_by_topic = {'left': {}, 'right': {}}
        for hand in ['left','right']:
            for topic, pts in by_topic[hand].items():
                try:
                    A = _np.array(pts, dtype=float)
                    means_by_topic[hand][topic] = _np.mean(A, axis=0)
                except Exception:
                    pass

        import numpy as _np2
        ref_np = {
            'left':  _np2.array(ref.get('left')  or [], dtype=float),
            'right': _np2.array(ref.get('right') or [], dtype=float),
        }

        # -------- 记录位置原始统计量：每个关键帧 / 话题 / 手 的重复位置 --------
        for _hand in ['left', 'right']:
            for _topic, _mean_pos in means_by_topic[_hand].items():
                if _topic == T_KUAVO:
                    continue
                try:
                    v = _np.array(_mean_pos, dtype=float).reshape(-1)
                    if v.size != 3:
                        continue
                except Exception:
                    continue
                pos_vals_grouped[kf_idx][(_hand, _topic)].append(v)

        # -------- 关节角度误差 & 关节原始统计量收集 --------

        topics_dict = (res or {}).get('topics', {}) or {}

        def _get_joint_vec(topic_name):
            try:
                d = topics_dict.get(topic_name, {})
                jm = d.get('joint_mean', None)
                if jm is None:
                    return None
                v = _np2.array(jm, dtype=float).reshape(-1)
                if v.size == 0:
                    return None
                return v
            except Exception:
                return None

        # 为每个话题缓存关节向量，同时写入“原始关节统计”容器
        joint_means = {}

        for _tn in [T_MM, T_MPC, T_JCMD, T_SENSOR]:
            _v = _get_joint_vec(_tn)
            if _v is not None:
                joint_means[_tn] = _v
                joint_vals_grouped[kf_idx][_tn].append(_v)

        def _add_joint_err(k_idx, label, diff_vec):
            try:
                v = _np2.array(diff_vec, dtype=float).reshape(-1)
                if v.size == 0:
                    return
            except Exception:
                return
            joint_grouped[k_idx][label].append(v)
            joint_overall[label].append(v)

        jm_mpc    = joint_means.get(T_MPC)
        jm_jcmd   = joint_means.get(T_JCMD)
        jm_sensor = joint_means.get(T_SENSOR)

        # 与位置部分一致：记录 /humanoid_mpc_target_arm 与 joint_cmd 之间的关节差
        if jm_mpc is not None and jm_jcmd is not None and jm_mpc.shape == jm_jcmd.shape:
            _add_joint_err(kf_idx, 'joint: /humanoid_mpc_target_arm vs joint_cmd', jm_jcmd - jm_mpc)

        # 与位置部分一致：记录 joint_cmd 与 /sensors_data_raw 之间的关节差
        if jm_jcmd is not None and jm_sensor is not None and jm_jcmd.shape == jm_sensor.shape:
            _add_joint_err(kf_idx, 'joint: joint_cmd vs /sensors_data_raw', jm_sensor - jm_jcmd)

        def _add_err(hand, label, vec_b_minus_a):
            ex, ey, ez = vec_b_minus_a
            en = float((_np2.array([ex,ey,ez])**2).sum() ** 0.5)
            tup = (en, float(ex), float(ey), float(ez))
            grouped[kf_idx][(hand, label)].append(tup)
            overall[(hand, label)].append(tup)

        for hand in ['left','right']:
            topic_rel = T_LEFTREL if hand == 'left' else T_RIGHTREL
            mean_rel  = means_by_topic[hand].get(topic_rel)
            ref_xyz   = ref_np[hand]
            if ref_xyz.size == 3 and mean_rel is not None and len(mean_rel) == 3:
                _add_err(hand, 'ref vs mocap', _np2.array(mean_rel) - ref_xyz)
            if ref_xyz.size == 3:
                mean_mm    = means_by_topic[hand].get(T_MM)
                mean_mpc   = means_by_topic[hand].get(T_MPC)
                mean_kuavo = means_by_topic[hand].get(T_KUAVO)
                if mean_mm is not None and len(mean_mm) == 3:
                    _add_err(hand, 'ref vs /mm_kuavo_arm_traj', _np2.array(mean_mm) - ref_xyz)
                if mean_mpc is not None and len(mean_mpc) == 3:
                    _add_err(hand, 'ref vs /humanoid_mpc_target_arm', _np2.array(mean_mpc) - ref_xyz)
                if mean_kuavo is not None and len(mean_kuavo) == 3:
                    _add_err(hand, 'ref vs /kuavo_arm_traj', _np2.array(mean_kuavo) - ref_xyz)
                mean_jcmd = means_by_topic[hand].get(T_JCMD)
                if mean_jcmd is not None and len(mean_jcmd) == 3:
                    _add_err(hand, 'ref vs joint_cmd', _np2.array(mean_jcmd) - ref_xyz)
                mean_sensor = means_by_topic[hand].get(T_SENSOR)
                if mean_sensor is not None and len(mean_sensor) == 3:
                    _add_err(hand, 'ref vs /sensors_data_raw', _np2.array(mean_sensor) - ref_xyz)
            mean_mpc  = means_by_topic[hand].get(T_MPC)
            mean_jcmd = means_by_topic[hand].get(T_JCMD)
            if mean_mpc is not None and len(mean_mpc) == 3 and mean_jcmd is not None and len(mean_jcmd) == 3:
                _add_err(hand, '/humanoid_mpc_target_arm vs joint_cmd', _np2.array(mean_jcmd) - _np2.array(mean_mpc))
            mean_sensor = means_by_topic[hand].get(T_SENSOR)
            if mean_sensor is not None and len(mean_sensor) == 3 and mean_jcmd is not None and len(mean_jcmd) == 3:
                _add_err(hand, 'joint_cmd vs /sensors_data_raw', _np2.array(mean_sensor) - _np2.array(mean_jcmd))
            if mean_rel is not None and len(mean_rel) == 3 and mean_sensor is not None and len(mean_sensor) == 3:
                _add_err(hand, '/sensors_data_raw vs mocap', _np2.array(mean_rel) - _np2.array(mean_sensor))

    out_csv = os.path.join(out_dir, 'summary_keyframe_errors.csv')
    rows_out = [['keyframe', 'kf_time', 'repeats', 'hand', 'error_label',
                 'mean_err_norm', 'std_err_norm', 'mean_err_x', 'mean_err_y', 'mean_err_z',
                 'std_err_x', 'std_err_y', 'std_err_z']]

    def _repeat_count_for_kf(k):
        cnt = 0
        for vals in grouped[k].values():
            cnt = max(cnt, len(vals))
        return cnt

    for kf_idx in sorted(grouped.keys()):
        name, kf_time = _kf_meta(kf_idx)
        label_name = name if (isinstance(name, str) and len(name) > 0) else f"kf_{kf_idx}"
        repeat_count = _repeat_count_for_kf(kf_idx)
        for (hand, lbl), vals in sorted(grouped[kf_idx].items()):
            A = _np.array(vals, dtype=float)
            if A.size:
                m = A.mean(axis=0)
                s = A.std(axis=0)
            else:
                m = _np.array([_np.nan]*4)
                s = _np.array([_np.nan]*4)
            rows_out.append([
                label_name,
                f"{kf_time:.6f}" if isinstance(kf_time, float) else '',
                str(repeat_count), hand, lbl,
                f"{m[0]:.6f}", f"{s[0]:.6f}",
                f"{m[1]:.6f}", f"{m[2]:.6f}", f"{m[3]:.6f}",
                f"{s[1]:.6f}", f"{s[2]:.6f}", f"{s[3]:.6f}"
            ])

    total_repeats = 0
    for vals in overall.values():
        total_repeats = max(total_repeats, len(vals))
    for (hand, lbl), vals in sorted(overall.items()):
        A = _np.array(vals, dtype=float)
        if A.size:
            m = A.mean(axis=0)
            s = A.std(axis=0)
        else:
            m = _np.array([_np.nan]*4)
            s = _np.array([_np.nan]*4)
        rows_out.append([
            'ALL', '', str(total_repeats), hand, lbl,
            f"{m[0]:.6f}", f"{s[0]:.6f}",
            f"{m[1]:.6f}", f"{m[2]:.6f}", f"{m[3]:.6f}",
            f"{s[1]:.6f}", f"{s[2]:.6f}", f"{s[3]:.6f}"
        ])

    # -------------------------------------------------
    # 追加：关节角度误差统计结果（逐关节均值与标准差）
    # 输出到同一个 summary_keyframe_errors_aligned.txt 末尾
    # -------------------------------------------------

    # 空行作为位置误差块与关节误差块之间的分隔
    rows_out.append([''] * len(rows_out[0]))

    # 关节误差表头说明：
    # keyframe, kf_time, repeats, joint_index, pair_label, mean_err, std_err
    rows_out.append([
        'JOINT_KEYFRAME', 'kf_time', 'repeats',
        'joint_index', 'joint_error_label',
        'mean_err', 'std_err',
        '', '', '', '', '', ''
    ])

    def _joint_repeat_count_for_kf(k):
        cnt = 0
        for vecs in joint_grouped[k].values():
            cnt = max(cnt, len(vecs))
        return cnt

    # 分关键帧统计
    for kf_idx in sorted(joint_grouped.keys()):
        name, kf_time = _kf_meta(kf_idx)
        label_name = name if (isinstance(name, str) and len(name) > 0) else f"kf_{kf_idx}"
        repeat_count = _joint_repeat_count_for_kf(kf_idx)
        for label, vecs in sorted(joint_grouped[kf_idx].items()):
            if not vecs:
                continue
            A = _np.array(vecs, dtype=float)  # (N, num_joints)
            if A.ndim != 2 or A.shape[1] == 0:
                continue
            num_joints = A.shape[1]
            for j_idx in range(num_joints):
                col = A[:, j_idx]
                m = float(col.mean())
                s = float(col.std())
                rows_out.append([
                    label_name,
                    f"{kf_time:.6f}" if isinstance(kf_time, float) else '',
                    str(repeat_count),
                    str(j_idx),
                    label,
                    f"{m:.6f}",
                    f"{s:.6f}",
                    '', '', '', '', '', ''
                ])

    # 所有关键帧的整体关节统计
    total_joint_repeats = 0
    for vecs in joint_overall.values():
        total_joint_repeats = max(total_joint_repeats, len(vecs))

    for label, vecs in sorted(joint_overall.items()):
        if not vecs:
            continue
        A = _np.array(vecs, dtype=float)
        if A.ndim != 2 or A.shape[1] == 0:
            continue
        num_joints = A.shape[1]
        for j_idx in range(num_joints):
            col = A[:, j_idx]
            m = float(col.mean())
            s = float(col.std())
            rows_out.append([
                'ALL',
                '',
                str(total_joint_repeats),
                str(j_idx),
                label,
                f"{m:.6f}",
                f"{s:.6f}",
                '', '', '', '', '', ''
            ])

    aligned_path = os.path.join(out_dir, 'summary_keyframe_errors_aligned.txt')
    col_widths = [max(len(str(r[i])) for r in rows_out) for i in range(len(rows_out[0]))]
    with open(aligned_path, 'w', encoding='utf-8') as ftxt:
        for r in rows_out:
            line = '  '.join(str(r[i]).ljust(col_widths[i]) for i in range(len(r)))
            ftxt.write(line + '\n')

    # -------------------------------------------------
    # 每个关键帧 / 话题 的位置 & 关节角度均值与标准差
    # 输出到单独的 TXT 文件，不影响原有 summary_keyframe_errors_aligned.txt
    # -------------------------------------------------
    topic_stats_rows = []

    # 位置统计表头：
    # keyframe, kf_time, repeats, hand, topic, mean_x, mean_y, mean_z, std_x, std_y, std_z
    topic_stats_rows.append([
        'keyframe', 'kf_time', 'repeats',
        'hand', 'topic',
        'mean_x', 'mean_y', 'mean_z',
        'std_x', 'std_y', 'std_z'
    ])

    # 位置：按关键帧 / 手 / 话题 聚合
    for kf_idx in sorted(pos_vals_grouped.keys()):
        name, kf_time = _kf_meta(kf_idx)
        label_name = name if (isinstance(name, str) and len(name) > 0) else f"kf_{kf_idx}"
        for (hand, topic), vecs in sorted(pos_vals_grouped[kf_idx].items()):
            if not vecs:
                continue
            A = _np.array(vecs, dtype=float)
            if A.ndim != 2 or A.shape[1] != 3:
                continue
            repeats = A.shape[0]
            m = A.mean(axis=0)
            s = A.std(axis=0)
            topic_stats_rows.append([
                label_name,
                f"{kf_time:.6f}" if isinstance(kf_time, float) else '',
                str(repeats),
                hand,
                topic,
                f"{m[0]:.6f}", f"{m[1]:.6f}", f"{m[2]:.6f}",
                f"{s[0]:.6f}", f"{s[1]:.6f}", f"{s[2]:.6f}",
            ])

    # 空行分隔位置块与关节块
    topic_stats_rows.append([''] * len(topic_stats_rows[0]))

    # 关节统计表头：
    # keyframe, kf_time, repeats, topic, joint_index, mean_angle, std_angle, 其余列留空保持列数一致
    topic_stats_rows.append([
        'JOINT_KEYFRAME', 'kf_time', 'repeats',
        'topic', 'joint_index',
        'mean_angle', 'std_angle',
        '', '', '', ''
    ])

    for kf_idx in sorted(joint_vals_grouped.keys()):
        name, kf_time = _kf_meta(kf_idx)
        label_name = name if (isinstance(name, str) and len(name) > 0) else f"kf_{kf_idx}"
        for topic, vecs in sorted(joint_vals_grouped[kf_idx].items()):
            if not vecs:
                continue
            A = _np.array(vecs, dtype=float)
            if A.ndim != 2 or A.shape[1] == 0:
                continue
            repeats = A.shape[0]
            num_j = A.shape[1]
            for j_idx in range(num_j):
                col = A[:, j_idx]
                m = float(col.mean())
                s = float(col.std())
                topic_stats_rows.append([
                    label_name,
                    f"{kf_time:.6f}" if isinstance(kf_time, float) else '',
                    str(repeats),
                    topic,
                    str(j_idx),
                    f"{m:.6f}",
                    f"{s:.6f}",
                    '', '', '', ''
                ])

    topic_stats_path = os.path.join(out_dir, 'summary_keyframe_topic_stats_aligned.txt')
    if topic_stats_rows:
        col_w2 = [max(len(str(r[i])) for r in topic_stats_rows) for i in range(len(topic_stats_rows[0]))]
        with open(topic_stats_path, 'w', encoding='utf-8') as f2:
            for r in topic_stats_rows:
                line = '  '.join(str(r[i]).ljust(col_w2[i]) for i in range(len(r)))
                f2.write(line + '\n')

    return aligned_path


def deg(rad):
    """弧度转角度"""
    return rad * 180.0 / math.pi


class KeyframeJointAngleRecorder:
    """记录关键帧的下发和反馈关节角度，并在退出时写入文件"""
    
    def __init__(self, output_file=None):
        self.records = []  # 存储记录: [{'keyframe_time': float, 'cmd_q': list, 'feedback_q': list, 'loop_index': int}, ...]
        self.latest_sensor_data = None
        
        # 确定输出文件路径
        if output_file is None:
            output_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'output')
            os.makedirs(output_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_file = os.path.join(output_dir, f'keyframe_joint_angles_{timestamp}.txt')
        self.output_file = output_file
        
        # 订阅传感器数据
        self.sensor_sub = rospy.Subscriber('/sensors_data_raw', sensorsData, self._sensor_callback, queue_size=10)
        
        # 注册退出时的写入函数
        atexit.register(self.write_to_file)
        rospy.on_shutdown(self.write_to_file)
        
        rospy.loginfo(f'关键帧关节角度记录器已初始化，输出文件: {self.output_file}')
    
    def _sensor_callback(self, msg):
        """传感器数据回调，保存最新的关节角度"""
        try:
            if hasattr(msg, 'joint_data') and hasattr(msg.joint_data, 'joint_q') and len(msg.joint_data.joint_q) >= 26:
                arm_joints = list(msg.joint_data.joint_q[12:26])  # 14个手臂关节，单位：弧度
                self.latest_sensor_data = arm_joints
        except Exception as e:
            rospy.logwarn(f'传感器数据回调处理失败: {e}')
    
    def record_keyframe(self, keyframe_time, cmd_q_rad, wait_for_feedback=0.15, loop_index=None):
        """
        记录关键帧的下发关节角度，并等待一段时间后获取反馈数据
        
        Args:
            keyframe_time: 关键帧时间（秒）
            cmd_q_rad: 下发的关节角度（弧度），14维列表
            wait_for_feedback: 等待反馈数据的时间（秒），默认0.15秒
            loop_index: 循环索引（可选），用于标识是第几次循环
        """
        # 确保是弧度（q已经是弧度，但这里做验证）
        if len(cmd_q_rad) != 14:
            rospy.logwarn(f'记录关键帧时关节角度数量错误: 期望14，实际{len(cmd_q_rad)}')
            return
        
        # 等待一段时间以获取更新的传感器反馈数据
        if wait_for_feedback > 0:
            rospy.sleep(wait_for_feedback)
        
        # 获取最新的传感器反馈数据
        feedback_q = None
        if self.latest_sensor_data is not None:
            feedback_q = list(self.latest_sensor_data)
        
        # 记录数据
        record = {
            'keyframe_time': float(keyframe_time),
            'cmd_q': list(cmd_q_rad),  # 弧度
            'feedback_q': feedback_q,  # 弧度，可能为None
            'loop_index': loop_index,  # 循环索引
        }
        self.records.append(record)
        
        if feedback_q is not None:
            rospy.loginfo(f'已记录关键帧 t={keyframe_time:.3f}s 的关节角度（下发+反馈）')
        else:
            rospy.logwarn(f'已记录关键帧 t={keyframe_time:.3f}s 的关节角度，但未获取到反馈数据')
    
    def write_to_file(self):
        """将记录写入文件"""
        if not self.records:
            rospy.loginfo('没有关键帧记录，跳过文件写入')
            return
        
        try:
            with open(self.output_file, 'w', encoding='utf-8') as f:
                f.write("=" * 100 + "\n")
                f.write("关键帧关节角度记录\n")
                f.write(f"总关键帧数: {len(self.records)}\n")
                f.write("=" * 100 + "\n\n")
                
                joint_names = [
                    "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
                    "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
                    "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
                    "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
                ]
                
                for idx, rec in enumerate(self.records, 1):
                    f.write("-" * 100 + "\n")
                    loop_info = f" (循环 #{rec.get('loop_index', '?')})" if rec.get('loop_index') is not None else ""
                    f.write(f"关键帧 #{idx} - 时间: {rec['keyframe_time']:.3f}s{loop_info}\n")
                    f.write("-" * 100 + "\n\n")
                    
                    # 下发关节角度（弧度）
                    f.write("下发关节角度 (弧度):\n")
                    cmd_q = rec['cmd_q']
                    for i, (name, q) in enumerate(zip(joint_names, cmd_q)):
                        f.write(f"  {name:20s}: {q:12.6f} rad ({deg(q):10.4f} deg)\n")
                    f.write("\n")
                    
                    # 反馈关节角度（弧度）
                    feedback_q = rec['feedback_q']
                    if feedback_q is not None:
                        f.write("反馈关节角度 (弧度):\n")
                        for i, (name, q) in enumerate(zip(joint_names, feedback_q)):
                            f.write(f"  {name:20s}: {q:12.6f} rad ({deg(q):10.4f} deg)\n")
                        f.write("\n")
                        
                        # 计算误差
                        f.write("误差分析:\n")
                        errors_rad = [cmd_q[i] - feedback_q[i] for i in range(14)]
                        errors_deg = [deg(err) for err in errors_rad]
                        
                        f.write("  关节角度误差 (弧度):\n")
                        for i, (name, err) in enumerate(zip(joint_names, errors_rad)):
                            f.write(f"    {name:20s}: {err:12.6f} rad ({errors_deg[i]:10.4f} deg)\n")
                        
                        # 统计信息
                        abs_errors_rad = [abs(e) for e in errors_rad]
                        abs_errors_deg = [abs(e) for e in errors_deg]
                        max_error_idx = abs_errors_rad.index(max(abs_errors_rad))
                        mean_error = sum(abs_errors_rad) / len(abs_errors_rad)
                        max_error = max(abs_errors_rad)
                        
                        f.write("\n  误差统计:\n")
                        f.write(f"    平均绝对误差: {mean_error:.6f} rad ({deg(mean_error):.4f} deg)\n")
                        f.write(f"    最大绝对误差: {max_error:.6f} rad ({deg(max_error):.4f} deg)\n")
                        f.write(f"    最大误差关节: {joint_names[max_error_idx]} (误差: {errors_rad[max_error_idx]:.6f} rad = {errors_deg[max_error_idx]:.4f} deg)\n")
                        
                        # RMS误差
                        rms_error = math.sqrt(sum([e*e for e in errors_rad]) / len(errors_rad))
                        f.write(f"    RMS误差: {rms_error:.6f} rad ({deg(rms_error):.4f} deg)\n")
                    else:
                        f.write("反馈关节角度: 未获取到数据\n")
                    
                    f.write("\n")
                
                f.write("=" * 100 + "\n")
                f.write("记录结束\n")
                f.write("=" * 100 + "\n")
            
            rospy.loginfo(f'关键帧关节角度记录已写入文件: {self.output_file}')
        except Exception as e:
            rospy.logerr(f'写入关键帧关节角度记录文件失败: {e}')

