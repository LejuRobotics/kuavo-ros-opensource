#!/usr/bin/env bash
set -euo pipefail

# -----------------------------
# Config
# -----------------------------
TOPICS=(
  "/vmp/input_data"
  "/gmr/vmp_input"
  "/gmr/skeleton_frame"
  "/gmr/rigid_body_desc"
)

OUTPUT_DIR="/root/kuavo_ws/src/kuavo_mocap_gmr/scripts"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_PATH="${OUTPUT_DIR}/gmr_data_${TIMESTAMP}.bag"

# v52 VMP dim
EXPECTED_VMP_DIM="${EXPECTED_VMP_DIM:-79}"

# Prefer msg.header.stamp when available (recommended)
PREFER_HEADER_STAMP="${PREFER_HEADER_STAMP:-1}"

mkdir -p "${OUTPUT_DIR}"

echo "Recording to: ${BAG_PATH}"
echo "Ctrl+C to stop. After stop: will generate FPS plot + NPZ export."

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
  local out_plot="${base}_fps.png"
  local out_npz="${base}_v52.npz"

  echo "[Post] Bag: ${BAG_PATH}"
  echo "[Post] FPS plot -> ${out_plot}"
  echo "[Post] NPZ export -> ${out_npz} (root + vmp_data(${EXPECTED_VMP_DIM}) + t)"

  python3 - "${BAG_PATH}" "${out_plot}" "${out_npz}" "${EXPECTED_VMP_DIM}" "${PREFER_HEADER_STAMP}" <<'PY'
import sys, math
import numpy as np

# Headless-safe plotting
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

bag_path = sys.argv[1]
out_plot = sys.argv[2]
out_npz  = sys.argv[3]
expected_vmp_dim = int(sys.argv[4])
prefer_header = (int(sys.argv[5]) != 0)

TOPIC_SKELETON = "/gmr/skeleton_frame"
TOPIC_VMP_INPUT = "/gmr/vmp_input"
TOPIC_VMP_DATA = "/vmp/input_data"

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

def extract_float_array(msg):
    for attr in ["data", "vmp_data", "values"]:
        if hasattr(msg, attr):
            return np.asarray(getattr(msg, attr), dtype=np.float32).reshape(-1)
    raise RuntimeError(f"Unknown array field in message type: {type(msg)}")

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
        fps_mean_dt=float(1.0/mean_dt) if mean_dt > 0 else float("nan"),
        fps_median_dt=float(1.0/med_dt) if med_dt > 0 else float("nan"),
        dt_mean=mean_dt,
        dt_min=float(np.min(dts)),
        dt_max=float(np.max(dts)),
        times=times,
    )

def nearest_match_indices(src_times, ref_times):
    src_times = np.asarray(src_times, dtype=np.float64)
    ref_times = np.asarray(ref_times, dtype=np.float64)
    idxs = np.searchsorted(ref_times, src_times, side="left")
    idxs = np.clip(idxs, 0, len(ref_times) - 1)
    prev = np.clip(idxs - 1, 0, len(ref_times) - 1)
    nxt  = idxs
    prev_dist = np.abs(src_times - ref_times[prev])
    next_dist = np.abs(src_times - ref_times[nxt])
    return np.where(prev_dist <= next_dist, prev, nxt)

import rosbag

topic_times = {TOPIC_SKELETON: [], TOPIC_VMP_INPUT: [], TOPIC_VMP_DATA: []}
vmp_data_times, vmp_data_list = [], []
root_times, root_list = [], []

with rosbag.Bag(bag_path, "r") as bag:
    info = bag.get_type_and_topic_info()[1]
    print("=== Topics in bag (type, count) ===")
    for k in [TOPIC_SKELETON, TOPIC_VMP_INPUT, TOPIC_VMP_DATA]:
        if k in info:
            print(f"{k:20s} | {info[k].msg_type:40s} | {info[k].message_count}")
        else:
            print(f"{k:20s} | (NOT FOUND)")

    for topic, msg, t in bag.read_messages(topics=[TOPIC_SKELETON, TOPIC_VMP_INPUT, TOPIC_VMP_DATA]):
        ts = get_msg_time(msg, t, prefer_header=prefer_header)
        topic_times[topic].append(ts)

        if topic == TOPIC_VMP_DATA:
            arr = extract_float_array(msg)
            if arr.size < expected_vmp_dim:
                raise RuntimeError(f"{TOPIC_VMP_DATA} dim too small: got {arr.size}, expect >= {expected_vmp_dim}")
            vmp_data_times.append(ts)
            vmp_data_list.append(arr[:expected_vmp_dim].astype(np.float32, copy=False))

        elif topic == TOPIC_VMP_INPUT:
            if not hasattr(msg, "base_link_pose"):
                raise RuntimeError(f"{TOPIC_VMP_INPUT} has no base_link_pose")
            p = msg.base_link_pose.position
            q = msg.base_link_pose.orientation  # xyzw
            root = np.array([p.x, p.y, p.z, q.w, q.x, q.y, q.z], dtype=np.float32)  # wxyz
            qw, qx, qy, qz = root[3:]
            n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
            if n > 1e-12:
                root[3:] /= n
            root_times.append(ts)
            root_list.append(root)

print("\n=== FPS Stats ===")
stats_map = {}
for topic in [TOPIC_SKELETON, TOPIC_VMP_INPUT, TOPIC_VMP_DATA]:
    ts = sorted(topic_times[topic])
    st = compute_fps_stats(ts)
    stats_map[topic] = st
    if st is None:
        print(f"{topic:20s}: insufficient messages")
        continue
    print(f"{topic:20s}: count={st['count']}, duration={st['duration']:.3f}s, "
          f"fps(duration)={st['fps_by_duration']:.2f}, "
          f"fps(mean_dt)={st['fps_mean_dt']:.2f}, fps(median_dt)={st['fps_median_dt']:.2f}, "
          f"dt_mean={st['dt_mean']*1000:.2f}ms, dt_min={st['dt_min']*1000:.2f}ms, dt_max={st['dt_max']*1000:.2f}ms")

# Plot instantaneous FPS
plt.figure(figsize=(12, 5))
for topic in [TOPIC_SKELETON, TOPIC_VMP_INPUT, TOPIC_VMP_DATA]:
    st = stats_map[topic]
    if st is None:
        continue
    times = st["times"]
    dts = np.diff(times)
    mask = dts > 0
    if mask.sum() == 0:
        continue
    fps_inst = 1.0 / dts[mask]
    t_mid = (times[:-1][mask] + times[1:][mask]) * 0.5
    t_mid = t_mid - t_mid[0]
    plt.plot(t_mid, fps_inst, label=topic)

plt.xlabel("time (s)")
plt.ylabel("instantaneous FPS (1/dt)")
plt.title("Topic FPS check")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(out_plot, dpi=200)
print(f"\nSaved FPS plot -> {out_plot}")

# Export NPZ (v52)
if len(vmp_data_list) == 0:
    raise RuntimeError(f"No {TOPIC_VMP_DATA} messages, cannot export.")
if len(root_list) == 0:
    raise RuntimeError(f"No {TOPIC_VMP_INPUT} messages, cannot export root.")

vmp_data_times = np.asarray(vmp_data_times, dtype=np.float64)
vmp_data = np.stack(vmp_data_list, axis=0).astype(np.float32)

root_times = np.asarray(root_times, dtype=np.float64)
root_arr = np.stack(root_list, axis=0).astype(np.float32)

# Sort + match root to vmp_data by nearest time
vmp_order = np.argsort(vmp_data_times)
vmp_data_times = vmp_data_times[vmp_order]
vmp_data = vmp_data[vmp_order]

root_order = np.argsort(root_times)
root_times = root_times[root_order]
root_arr = root_arr[root_order]

match = nearest_match_indices(vmp_data_times, root_times)
root_matched = root_arr[match]

np.savez_compressed(out_npz, root=root_matched, vmp_data=vmp_data, t=vmp_data_times)
print(f"Saved NPZ -> {out_npz}")
print(f"  root shape     : {root_matched.shape}  (x,y,z,qw,qx,qy,qz)")
print(f"  vmp_data shape : {vmp_data.shape}      (N,{expected_vmp_dim})")
print(f"  t shape        : {vmp_data_times.shape}")
PY
}

# -----------------------------
# Record in background + trap Ctrl+C
# -----------------------------
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

# Wait until rosbag ends (normal end / error / signal)
wait "${ROSBAG_PID}" 2>/dev/null || true

# If it ended without Ctrl+C, still post-process
post_process
