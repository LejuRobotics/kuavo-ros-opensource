import os
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import glob
import rospy
import atexit
from datetime import datetime
from kuavo_msgs.msg import sensorsData

def mocap_to_world(pos, quat):
    # 先绕X轴逆时针90度，再绕Z轴顺时针180度
    rot_x_90 = R.from_euler('x', 90, degrees=True)
    rot_z_180 = R.from_euler('z', -180, degrees=True)  # 顺时针180度（负值）
    r_total = rot_z_180 * rot_x_90  # 先应用X轴旋转，再应用Z轴旋转
    
    r_mocap = R.from_quat(quat)
    r_world = r_total * r_mocap
    quat_world = r_world.as_quat()
    pos_world = r_total.apply(pos)
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
        if topic in ['/r_hand_pose', '/l_hand_pose', '/car_pose', '/waist_pose', '/r_shoulder_pose', '/l_shoulder_pose', '/head_pose']:
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
                "pos": [px, py, pz],
                "quat": [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            })
        elif topic == '/sensors_data_raw':
            raw_data[topic].append({"pos": list(msg.joint_data.joint_q[0:19])})
            
        elif topic == '/joint_cmd':
            try:
                joints = list(getattr(msg, 'joint_q', []))
                raw_data[topic].append({"pos": list(joints[0:19])})
            except Exception:
                pass
    bag.close()
    for topic in ['/r_hand_pose', '/l_hand_pose', '/car_pose', '/waist_pose', '/r_shoulder_pose', '/l_shoulder_pose', '/head_pose']:
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
    for hand_topic in ['/r_hand_pose', '/l_hand_pose', '/waist_pose', '/r_shoulder_pose', '/l_shoulder_pose', '/head_pose']:
        base_topic = '/car_pose'
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
                del result_data['topics'][hand_topic]

    for topic in ['/sensors_data_raw',  '/joint_cmd']:
        poses = [entry['pos'] for entry in raw_data[topic] if 'pos' in entry]
        if not poses:
            continue
        arr = np.array(poses, dtype=float)
        mean_joints = np.mean(arr, axis=0).tolist()
        result_data['topics'][topic] = {
            'joint_mean': mean_joints
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

def collect_keyframe_labels(bag_folder: str, flag_topic: str, keyframes):
    """返回 (bag_infos, labels)。labels: list[(t_label, kf_idx)] 按时间升序。"""
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
    """记录并统计窗口数据，输出表格。"""
    import numpy as _np
    os.makedirs(out_dir, exist_ok=True)
    
    # 按关键帧和话题收集数据
    # 结构: data_by_kf_topic[kf_idx][topic] = [数据1, 数据2, ...]
    data_by_kf_topic = {}
    
    # 遍历所有结果，收集数据
    for res, kf_idx in zip(results or [], kf_indices or []):
        if res is None:
            continue
        
        if kf_idx not in data_by_kf_topic:
            data_by_kf_topic[kf_idx] = {}
        
        topics = res.get('topics', {})
        for topic, topic_data in topics.items():
            if topic not in data_by_kf_topic[kf_idx]:
                data_by_kf_topic[kf_idx][topic] = []
            
            # 根据数据类型记录
            if 'position_mean' in topic_data:
                # Pose topic: 记录位置
                pos = topic_data.get('position_mean')
                if pos is not None:
                    data_by_kf_topic[kf_idx][topic].append({
                        'type': 'position',
                        'data': pos
                    })
            elif 'joint_mean' in topic_data:
                # Joint topic: 记录关节角度
                joints = topic_data.get('joint_mean')
                if joints is not None:
                    data_by_kf_topic[kf_idx][topic].append({
                        'type': 'joint',
                        'data': joints
                    })
    
    # 输出表格
    output_file = os.path.join(out_dir, 'raw_data_records.txt')
    with open(output_file, 'w', encoding='utf-8') as f:
        # 表头
        f.write("=" * 100 + "\n")
        f.write("原始数据记录\n")
        f.write("=" * 100 + "\n\n")
        
        # 按关键帧输出
        for kf_idx in sorted(data_by_kf_topic.keys()):
            # 获取关键帧时间
            kf_time = None
            try:
                if 0 <= kf_idx < len(keyframes):
                    kf_time = float(keyframes[kf_idx].get('time', None))
            except Exception:
                pass
            
            f.write(f"\n关键帧 #{kf_idx}")
            if kf_time is not None:
                f.write(f" (时间: {kf_time:.3f}s)")
            f.write(f"\n")
            f.write("-" * 100 + "\n")
            
            # 按话题输出，先输出位置相关的，再输出角度相关的
            topics = list(data_by_kf_topic[kf_idx].keys())
            
            # 定义排序函数：位置相关的在前，角度相关的在后
            def topic_sort_key(topic):
                data_list = data_by_kf_topic[kf_idx][topic]
                if not data_list:
                    return (2, topic)  # 空数据放在最后
                if data_list[0]['type'] == 'position':
                    return (0, topic)  # 位置相关的在前
                elif data_list[0]['type'] == 'joint':
                    return (1, topic)  # 角度相关的在后
                return (2, topic)  # 其他类型放在最后
            
            sorted_topics = sorted(topics, key=topic_sort_key)
            
            for topic in sorted_topics:
                data_list = data_by_kf_topic[kf_idx][topic]
                f.write(f"\n话题: {topic}\n")
                f.write(f"  数据条数: {len(data_list)}\n")
                
                # 输出全部数据
                for idx, item in enumerate(data_list):
                    f.write(f"  记录 #{idx + 1}:\n")
                    if item['type'] == 'position':
                        pos = item['data']
                        f.write(f"    位置: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}]\n")
                    elif item['type'] == 'joint':
                        joints = item['data']
                        f.write(f"    关节角度: {joints}\n")
                
                # 计算并输出该话题的均值统计
                if data_list[0]['type'] == 'position':
                    # 计算位置均值
                    positions = [item['data'] for item in data_list]
                    pos_array = _np.array(positions, dtype=float)
                    mean_pos = _np.mean(pos_array, axis=0)
                    std_pos = _np.std(pos_array, axis=0)
                    
                    f.write("\n")
                    f.write(f"  位置均值: [{mean_pos[0]:.6f}, {mean_pos[1]:.6f}, {mean_pos[2]:.6f}]\n")
                    f.write(f"  位置标准差: [{std_pos[0]:.6f}, {std_pos[1]:.6f}, {std_pos[2]:.6f}]\n")
                    
                elif data_list[0]['type'] == 'joint':
                    # 计算关节角度均值
                    joints_list = [item['data'] for item in data_list]
                    # 确保所有关节向量长度一致
                    try:
                        joints_array = _np.array(joints_list, dtype=float)
                        mean_joints = _np.mean(joints_array, axis=0)
                        std_joints = _np.std(joints_array, axis=0)
                        
                        f.write("\n")
                        f.write(f"  关节角度均值: {mean_joints.tolist()}\n")
                        f.write(f"  关节角度标准差: {std_joints.tolist()}\n")
                    except Exception as e:
                        f.write(f"  计算均值时出错: {e}\n")
                
                f.write("\n")
        
        # 计算误差
        f.write("\n" + "=" * 100 + "\n")
        f.write("误差统计\n")
        f.write("=" * 100 + "\n\n")
        
        # 定义要计算误差的话题对
        error_pairs = [
            ('/r_hand_pose_rel_base', '/r_shoulder_pose_rel_base', 'position'),
            ('/l_hand_pose_rel_base', '/l_shoulder_pose_rel_base', 'position'),
            # ('/waist_pose', '/car_pose', 'position'),
            ('/joint_cmd', '/sensors_data_raw', 'joint'),
        ]
        
        for kf_idx in sorted(data_by_kf_topic.keys()):
            # 获取关键帧时间
            kf_time = None
            try:
                if 0 <= kf_idx < len(keyframes):
                    kf_time = float(keyframes[kf_idx].get('time', None))
            except Exception:
                pass
            
            f.write(f"\n关键帧 #{kf_idx}")
            if kf_time is not None:
                f.write(f" (时间: {kf_time:.3f}s)")
            f.write(f"\n")
            f.write("-" * 100 + "\n")
            
            # 计算每对话题的误差
            for topic1, topic2, data_type in error_pairs:
                # 检查两个话题是否都存在数据
                if (kf_idx not in data_by_kf_topic or 
                    topic1 not in data_by_kf_topic[kf_idx] or 
                    topic2 not in data_by_kf_topic[kf_idx]):
                    continue
                
                data1_list = data_by_kf_topic[kf_idx][topic1]
                data2_list = data_by_kf_topic[kf_idx][topic2]
                
                if not data1_list or not data2_list:
                    continue
                
                # 确保数据条数一致
                min_len = min(len(data1_list), len(data2_list))
                if min_len == 0:
                    continue
                
                f.write(f"\n误差: {topic1} vs {topic2}\n")
                f.write(f"  数据条数: {min_len}\n")
                
                errors = []
                for i in range(min_len):
                    item1 = data1_list[i]
                    item2 = data2_list[i]
                    
                    if data_type == 'position':
                        # 计算位置误差（topic1 - topic2）
                        pos1 = _np.array(item1['data'], dtype=float)
                        pos2 = _np.array(item2['data'], dtype=float)
                        error = pos1 - pos2
                        errors.append(error)
                        f.write(f"  记录 #{i + 1} 误差: [{error[0]:.6f}, {error[1]:.6f}, {error[2]:.6f}]\n")
                        
                    elif data_type == 'joint':
                        # 计算关节角度误差（topic1 - topic2）
                        joints1 = _np.array(item1['data'], dtype=float)
                        joints2 = _np.array(item2['data'], dtype=float)
                        if len(joints1) == len(joints2):
                            error = joints1 - joints2
                            errors.append(error)
                            f.write(f"  记录 #{i + 1} 误差: {error.tolist()}\n")
                
                # 计算误差的均值和标准差
                if errors:
                    errors_array = _np.array(errors, dtype=float)
                    mean_error = _np.mean(errors_array, axis=0)
                    std_error = _np.std(errors_array, axis=0)
                    
                    f.write(f"\n  误差均值: {mean_error.tolist()}\n")
                    f.write(f"  误差标准差: {std_error.tolist()}\n")
                    
                    if data_type == 'position':
                        # 计算误差的模长
                        error_norms = [_np.linalg.norm(e) for e in errors]
                        mean_norm = _np.mean(error_norms)
                        std_norm = _np.std(error_norms)
                        f.write(f"  误差模长均值: {mean_norm:.6f}\n")
                        f.write(f"  误差模长标准差: {std_norm:.6f}\n")
                
                f.write("\n")
    
    return output_file
