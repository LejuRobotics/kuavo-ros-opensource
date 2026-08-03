#!/usr/bin/env bash
set -euo pipefail

# Record /vmp/input_data (std_msgs/Float32MultiArray) directly to the raw
# float32 .bin format consumed by VMP onlineVRDataSource=bin_file playback.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

TOPIC="${TOPIC:-/vmp/input_data}"
OUTPUT_DIR="${OUTPUT_DIR:-${HOME}/vmp_input_recordings}"
EXPECTED_DIM="${EXPECTED_DIM:-}"
DURATION="${DURATION:-}"
MAX_FRAMES="${MAX_FRAMES:-}"
OUTPUT_PATH=""

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [output.bin] [options]

Options:
  --topic TOPIC           ROS topic to record (default: /vmp/input_data)
  --expected-dim N        Require each frame to have N floats, e.g. 77 or 79
  --duration SEC          Stop automatically after SEC seconds
  --max-frames N          Stop after recording N frames
  -h, --help              Show this help

Environment overrides:
  TOPIC, OUTPUT_DIR, EXPECTED_DIM, DURATION, MAX_FRAMES

Examples:
  $(basename "$0")
  $(basename "$0") /tmp/my_vmp_motion.bin --expected-dim 79
  DURATION=30 $(basename "$0") ./vmp_test.bin
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --topic)
      TOPIC="${2:?--topic requires a value}"
      shift 2
      ;;
    --topic=*)
      TOPIC="${1#*=}"
      shift
      ;;
    --expected-dim)
      EXPECTED_DIM="${2:?--expected-dim requires a value}"
      shift 2
      ;;
    --expected-dim=*)
      EXPECTED_DIM="${1#*=}"
      shift
      ;;
    --duration)
      DURATION="${2:?--duration requires a value}"
      shift 2
      ;;
    --duration=*)
      DURATION="${1#*=}"
      shift
      ;;
    --max-frames)
      MAX_FRAMES="${2:?--max-frames requires a value}"
      shift 2
      ;;
    --max-frames=*)
      MAX_FRAMES="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ -n "${OUTPUT_PATH}" ]]; then
        echo "Unexpected extra argument: $1" >&2
        usage >&2
        exit 2
      fi
      OUTPUT_PATH="$1"
      shift
      ;;
  esac
done

case "${TOPIC}" in
  vmp/inputdata|/vmp/inputdata|vmp/input_data)
    echo "[record_vmp_input_to_bin] Normalizing topic '${TOPIC}' to '/vmp/input_data'"
    TOPIC="/vmp/input_data"
    ;;
esac

if [[ -z "${OUTPUT_PATH}" ]]; then
  TIMESTAMP="$(date +"%Y%m%d_%H%M%S")"
  OUTPUT_PATH="${OUTPUT_DIR}/vmp_input_${TIMESTAMP}.bin"
elif [[ -d "${OUTPUT_PATH}" || "${OUTPUT_PATH}" == */ ]]; then
  TIMESTAMP="$(date +"%Y%m%d_%H%M%S")"
  OUTPUT_PATH="${OUTPUT_PATH%/}/vmp_input_${TIMESTAMP}.bin"
fi

mkdir -p "$(dirname "${OUTPUT_PATH}")"

# Source ROS and the workspace if available. Temporarily relax nounset because
# ROS setup scripts can reference unset shell variables.
set +u
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f "/opt/ros/noetic/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "/opt/ros/noetic/setup.bash"
fi

if [[ -f "${WORKSPACE_DIR}/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_DIR}/devel/setup.bash"
fi
set -u

echo "=========================================="
echo "  VMP Input Data Recorder"
echo "=========================================="
echo "Topic       : ${TOPIC}"
echo "Output bin  : ${OUTPUT_PATH}"
if [[ -n "${EXPECTED_DIM}" ]]; then
  echo "Expected dim: ${EXPECTED_DIM}"
else
  echo "Expected dim: infer from first message"
fi
if [[ -n "${DURATION}" ]]; then
  echo "Duration    : ${DURATION}s"
fi
if [[ -n "${MAX_FRAMES}" ]]; then
  echo "Max frames  : ${MAX_FRAMES}"
fi
echo ""
echo "Press Ctrl+C to stop."
echo "=========================================="

PY_ARGS=(--output "${OUTPUT_PATH}" --topic "${TOPIC}")
if [[ -n "${EXPECTED_DIM}" ]]; then
  PY_ARGS+=(--expected-dim "${EXPECTED_DIM}")
fi
if [[ -n "${DURATION}" ]]; then
  PY_ARGS+=(--duration "${DURATION}")
fi
if [[ -n "${MAX_FRAMES}" ]]; then
  PY_ARGS+=(--max-frames "${MAX_FRAMES}")
fi

exec python3 -u - "${PY_ARGS[@]}" <<'PY'
import argparse
import os
import signal
import struct
import sys
import threading
import time

import rospy
from std_msgs.msg import Float32MultiArray


def parse_args():
    parser = argparse.ArgumentParser(description="Record /vmp/input_data to raw float32 bin")
    parser.add_argument("--output", required=True)
    parser.add_argument("--topic", default="/vmp/input_data")
    parser.add_argument("--expected-dim", type=int, default=79)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--max-frames", type=int, default=None)
    return parser.parse_args()


class VmpInputBinRecorder:
    def __init__(self, output_path, topic, expected_dim=79, duration=None, max_frames=None):
        self.output_path = output_path
        self.topic = topic
        self.expected_dim = expected_dim
        self.duration = duration
        self.max_frames = max_frames
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.frame_dim = None
        self.frames = 0
        self.skipped = 0
        self.first_msg_wall = None
        self.last_msg_wall = None
        self.started_wall = time.time()
        self.next_status_wall = self.started_wall + 1.0
        self.out = open(output_path, "wb", buffering=1024 * 1024)

    def close(self):
        with self.lock:
            if not self.out.closed:
                self.out.flush()
                os.fsync(self.out.fileno())
                self.out.close()

    def request_stop(self, reason):
        if not self.done.is_set():
            print(f"\n[record_vmp_input_to_bin] stopping: {reason}")
            self.done.set()
            if not rospy.is_shutdown():
                rospy.signal_shutdown(reason)

    def callback(self, msg):
        data = list(msg.data)
        dim = len(data)
        now = time.time()

        if dim == 0:
            self.skipped += 1
            rospy.logwarn_throttle(2.0, "Received empty VMP frame; skipping")
            return

        if self.expected_dim is not None and dim != self.expected_dim:
            self.skipped += 1
            rospy.logwarn_throttle(
                2.0,
                "Frame dimension mismatch: got %d floats, expected %d; skipping",
                dim,
                self.expected_dim,
            )
            return

        with self.lock:
            if self.frame_dim is None:
                self.frame_dim = dim
                self.first_msg_wall = now
                print(f"\nFirst frame received: dim={dim} floats")
            elif dim != self.frame_dim:
                self.skipped += 1
                rospy.logwarn_throttle(
                    2.0,
                    "Frame dimension changed: got %d floats, first frame was %d; skipping",
                    dim,
                    self.frame_dim,
                )
                return

            self.out.write(struct.pack("<{}f".format(dim), *data))
            self.frames += 1
            self.last_msg_wall = now

            if self.max_frames is not None and self.frames >= self.max_frames:
                self.request_stop(f"max frames reached ({self.max_frames})")

    def print_status_if_due(self):
        now = time.time()
        if now < self.next_status_wall:
            return
        self.next_status_wall = now + 1.0

        with self.lock:
            frames = self.frames
            dim = self.frame_dim
            first = self.first_msg_wall
            skipped = self.skipped

        if frames == 0:
            print("\rWaiting for messages...", end="", flush=True)
            return

        elapsed = max(now - first, 1e-9)
        hz = frames / elapsed
        print(
            f"\rframes={frames} dim={dim} avg_hz={hz:.1f} skipped={skipped}",
            end="",
            flush=True,
        )

    def run(self):
        rospy.Subscriber(self.topic, Float32MultiArray, self.callback, queue_size=100)
        print(f"Subscribed to {self.topic}")

        def handle_signal(signum, _frame):
            self.request_stop(f"signal {signum}")

        signal.signal(signal.SIGINT, handle_signal)
        signal.signal(signal.SIGTERM, handle_signal)

        try:
            while not self.done.is_set() and not rospy.is_shutdown():
                if self.duration is not None and (time.time() - self.started_wall) >= self.duration:
                    self.request_stop(f"duration reached ({self.duration}s)")
                    break
                self.print_status_if_due()
                time.sleep(0.05)
        finally:
            self.close()
            print("")
            self.print_summary()

    def print_summary(self):
        file_size = os.path.getsize(self.output_path) if os.path.exists(self.output_path) else 0
        dim = self.frame_dim or self.expected_dim or 0
        expected_size = self.frames * dim * 4 if dim else 0

        print("==========================================")
        print("  VMP Input Data Recorder Finished")
        print("==========================================")
        print(f"Output bin : {self.output_path}")
        print(f"Frames     : {self.frames}")
        print(f"Frame dim  : {self.frame_dim if self.frame_dim is not None else 'unknown'}")
        print(f"Skipped    : {self.skipped}")
        print(f"File size  : {file_size} bytes")

        if expected_size and expected_size == file_size:
            print("Size check : OK")
        elif expected_size:
            print(f"Size check : expected {expected_size} bytes")

        if self.frames > 1 and self.first_msg_wall is not None and self.last_msg_wall is not None:
            duration = max(self.last_msg_wall - self.first_msg_wall, 1e-9)
            print(f"Duration   : {duration:.2f}s")
            print(f"Avg rate   : {(self.frames - 1) / duration:.1f} Hz")

        print("")
        print("Playback hint:")
        print("  Put this .bin under the VMP config's vmpRefDataDir, then set:")
        print("    vmpPlaybackMode     online_teleoperation")
        print("    onlineVRDataSource  bin_file")
        print(f"    onlineVRBinFile     {os.path.basename(self.output_path)}")
        print("  The playback robot config's vaeModel.in_c must match the recorded frame dim.")
        print("==========================================")


def main():
    args = parse_args()
    rospy.init_node("record_vmp_input_to_bin", anonymous=True, disable_signals=True)
    recorder = VmpInputBinRecorder(
        output_path=args.output,
        topic=args.topic,
        expected_dim=args.expected_dim,
        duration=args.duration,
        max_frames=args.max_frames,
    )
    recorder.run()
    return 0 if recorder.frames > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
PY
