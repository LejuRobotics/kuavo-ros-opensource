#!/usr/bin/env bash
set -euo pipefail

# -----------------------------
# Config
# -----------------------------
TOPICS=(
  "/pico/world_bone_poses"
  "/pico/retargeted_pose"
)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${OUTPUT_DIR:-${SCRIPT_DIR}}"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_PATH="${OUTPUT_DIR}/pico_data_${TIMESTAMP}.bag"

EXPECTED_JOINT_DIM="${EXPECTED_JOINT_DIM:-27}"
PREFER_HEADER_STAMP="${PREFER_HEADER_STAMP:-1}"

mkdir -p "${OUTPUT_DIR}"

echo "Recording to: ${BAG_PATH}"
echo "Ctrl+C to stop. After stop: will generate PKL export only."

POST_DONE=0
post_process() {
  if [[ "${POST_DONE}" == "1" ]]; then
    return 0
  fi
  POST_DONE=1

  if [[ ! -f "${BAG_PATH}" ]] || [[ ! -s "${BAG_PATH}" ]]; then
    echo "[Post] Bag not found or empty: ${BAG_PATH}. Skip post-process."
    return 0
  fi

  local base="${BAG_PATH%.bag}"
  local out_pkl="${base}_retarget.pkl"

  echo "[Post] Bag: ${BAG_PATH}"
  echo "[Post] PKL export -> ${out_pkl}"

  python3 - "${BAG_PATH}" "${out_pkl}" "${EXPECTED_JOINT_DIM}" "${PREFER_HEADER_STAMP}" <<'PY'
import sys
import numpy as np
import pickle
import rosbag

bag_path = sys.argv[1]
out_pkl = sys.argv[2]
expected_joint_dim = int(sys.argv[3])
prefer_header = (int(sys.argv[4]) != 0)

TOPIC_PICO_IN  = "/pico/world_bone_poses"
TOPIC_PICO_OUT = "/pico/retargeted_pose"

def to_sec(t):
    return float(t.secs) + float(t.nsecs) * 1e-9

def get_msg_time(msg, bag_t, prefer_header=True):
    if prefer_header and hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        try:
            ts = to_sec(msg.header.stamp)
            if ts > 0:
                return ts
        except Exception:
            pass
    return to_sec(bag_t)

def compute_fps_stats(times):
    times = np.asarray(times, dtype=np.float64)
    if times.size < 2:
        return None
    dts = np.diff(times)
    dts = dts[dts > 0]
    if dts.size == 0:
        return None
    duration = times[-1] - times[0]
    fps_by_duration = (len(times) - 1) / duration if duration > 0 else float("nan")
    mean_dt = float(np.mean(dts))
    med_dt  = float(np.median(dts))
    return dict(
        count=int(times.size),
        duration=float(duration),
        fps_by_duration=float(fps_by_duration),
        fps_mean_dt=float(1.0 / mean_dt) if mean_dt > 0 else float("nan"),
        fps_median_dt=float(1.0 / med_dt) if med_dt > 0 else float("nan"),
        dt_mean=mean_dt,
        dt_min=float(np.min(dts)),
        dt_max=float(np.max(dts)),
        times=times,
    )

def extract_root_pos_rot_xyzw(msg):
    if not hasattr(msg, "base_link_pose"):
        raise RuntimeError(f"{TOPIC_PICO_OUT} has no base_link_pose")
    p = msg.base_link_pose.position
    q = msg.base_link_pose.orientation  # ROS xyzw
    pos = np.array([p.x, p.y, p.z], dtype=np.float64)
    rot_xyzw = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    n = np.linalg.norm(rot_xyzw)
    if n > 1e-12:
        rot_xyzw /= n
    else:
        rot_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return pos, rot_xyzw

def extract_joint_position(msg):
    if not hasattr(msg, "joint_position"):
        raise RuntimeError(f"{TOPIC_PICO_OUT} has no joint_position")
    arr = np.asarray(msg.joint_position, dtype=np.float64).reshape(-1)
    if arr.size < expected_joint_dim:
        raise RuntimeError(
            f"joint_position dim too small: got {arr.size}, expect >= {expected_joint_dim}"
        )
    return arr[:expected_joint_dim]

topic_times = {
    TOPIC_PICO_IN: [],
    TOPIC_PICO_OUT: [],
}

out_times = []
root_pos_list = []
root_rot_xyzw_list = []
joint_pos_list = []

with rosbag.Bag(bag_path, "r") as bag:
    info = bag.get_type_and_topic_info()[1]
    print("=== Topics in bag (type, count) ===")
    for k in [TOPIC_PICO_IN, TOPIC_PICO_OUT]:
        if k in info:
            print(f"{k:24s} | {info[k].msg_type:40s} | {info[k].message_count}")
        else:
            print(f"{k:24s} | (NOT FOUND)")

    for topic, msg, t in bag.read_messages(topics=[TOPIC_PICO_IN, TOPIC_PICO_OUT]):
        ts = get_msg_time(msg, t, prefer_header=prefer_header)
        topic_times[topic].append(ts)

        if topic == TOPIC_PICO_OUT:
            out_times.append(ts)
            pos, rot_xyzw = extract_root_pos_rot_xyzw(msg)
            root_pos_list.append(pos)
            root_rot_xyzw_list.append(rot_xyzw)
            joint_pos_list.append(extract_joint_position(msg))

print("\n=== FPS Stats ===")
stats_map = {}
for topic in [TOPIC_PICO_IN, TOPIC_PICO_OUT]:
    ts = sorted(topic_times[topic])
    st = compute_fps_stats(ts)
    stats_map[topic] = st
    if st is None:
        print(f"{topic:24s}: insufficient messages")
        continue
    print(
        f"{topic:24s}: count={st['count']}, duration={st['duration']:.3f}s, "
        f"fps(duration)={st['fps_by_duration']:.2f}, "
        f"fps(mean_dt)={st['fps_mean_dt']:.2f}, "
        f"fps(median_dt)={st['fps_median_dt']:.2f}, "
        f"dt_mean={st['dt_mean']*1000:.2f}ms, "
        f"dt_min={st['dt_min']*1000:.2f}ms, "
        f"dt_max={st['dt_max']*1000:.2f}ms"
    )

if len(root_pos_list) == 0:
    raise RuntimeError(f"No {TOPIC_PICO_OUT} messages, cannot export.")

out_times = np.asarray(out_times, dtype=np.float64)
root_pos_arr = np.stack(root_pos_list, axis=0).astype(np.float64)
root_rot_xyzw_arr = np.stack(root_rot_xyzw_list, axis=0).astype(np.float64)
joint_pos_arr = np.stack(joint_pos_list, axis=0).astype(np.float64)

order = np.argsort(out_times)
out_times = out_times[order]
root_pos_arr = root_pos_arr[order]
root_rot_xyzw_arr = root_rot_xyzw_arr[order]
joint_pos_arr = joint_pos_arr[order]

st_out = stats_map[TOPIC_PICO_OUT]
fps_out = float(st_out["fps_mean_dt"]) if st_out is not None else 0.0

motion_data = {
    "fps": fps_out,
    "root_pos": root_pos_arr,        # (T,3)
    "root_rot": root_rot_xyzw_arr,   # (T,4), xyzw
    "dof_pos": joint_pos_arr,        # (T,27)
}

with open(out_pkl, "wb") as f:
    pickle.dump(motion_data, f)

print(f"\nSaved PKL -> {out_pkl}")
print(f"  fps       : {motion_data['fps']:.6f}")
print(f"  root_pos  : {motion_data['root_pos'].shape}")
print(f"  root_rot  : {motion_data['root_rot'].shape}  (xyzw)")
print(f"  dof_pos   : {motion_data['dof_pos'].shape}")
PY
}

ROSBAG_PID=""

cleanup() {
  echo
  echo "[Signal] Ctrl+C received. Stopping rosbag..."
  if [[ -n "${ROSBAG_PID}" ]]; then
    kill -INT "${ROSBAG_PID}" 2>/dev/null || true
    wait "${ROSBAG_PID}" 2>/dev/null || true
  fi
  post_process
  exit 0
}

trap cleanup INT TERM

rosbag record -O "${BAG_PATH}" "${TOPICS[@]}" &
ROSBAG_PID=$!

wait "${ROSBAG_PID}" 2>/dev/null || true
post_process
