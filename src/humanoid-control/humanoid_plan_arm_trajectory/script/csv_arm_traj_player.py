#!/usr/bin/env python3
"""CSV arm trajectory player for Kuavo.

This script is intentionally a thin external tool. It does not change the
controller implementation. It only:

1. Reads arm joint targets from a CSV file.
2. Optionally calls /arm_traj_change_mode to switch the arm mode.
3. Publishes sensor_msgs/JointState frames to /kuavo_arm_traj.
4. Optionally publishes waist yaw targets to /robot_waist_motion_data.
5. Optionally publishes dexterous hand targets to /control_robot_hand_position.
6. Optionally publishes head yaw/pitch targets to /robot_head_motion_data.
7. Optionally waits for a joystick button and toggles:
   - first press: switch arm mode to external_control (2) and play from start
   - next press: stop publishing and switch arm mode back to auto_swing (1)

Important conventions:
- /kuavo_arm_traj expects arm position/velocity in degrees.
- Most motion-export CSV files store joint values in radians, so use
  --position-unit rad and --velocity-unit rad for those files.
- The default full-body CSV layout is:
  columns 0:7   body/root state
  columns 7:34  27 joint positions
  columns 34:61 27 joint velocities
  From the 27 joints, this player uses index 12 as waist_yaw_joint and
  indices 13:27 as the 14 arm joints.
- Arm modes are arm-control modes only. They do not switch the body controller
  between MPC and AMP/RL.
- Waist control is independent from arm control. In AMP/RL, waist playback
  needs the controller config use_external_waist_controller=true and this
  player publishes /humanoid_controller/enable_waist_control when enabled.
- /control_robot_hand_position expects 12 remapped hand values in the range
  0-100: left hand 6 values followed by right hand 6 values.
- /robot_head_motion_data expects joint_data=[yaw, pitch] in degrees. If CSV
  head values are radians, use --head-unit rad.

Before running, source the workspace:
  cd /root/kuavo_ws
  source devel/setup.bash

Typical rosrun usage, 61-column deploy CSV with header and velocity:
  rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \
    --csv src/humanoid-control/humanoid_controllers/config/kuavo_v55/rl/IMG_9934_bvh_No1_50fps_deploy.csv \
    --has-header true \
    --position-columns 7:34 \
    --velocity-columns 34:61 \
    --fps 50 \
    --position-unit rad \
    --velocity-unit rad \
    --wait-for-trigger \
    --trigger-button-name BUTTON_RB

Typical rosrun usage, synchronized arm + waist from the same 27-joint block:
  rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \
    --csv src/humanoid-control/humanoid_controllers/config/kuavo_v55/rl/IMG_9934_bvh_No1_50fps_deploy.csv \
    --has-header true \
    --position-columns 7:34 \
    --velocity-columns 34:61 \
    --fps 50 \
    --position-unit rad \
    --velocity-unit rad \
    --publish-waist \
    --wait-for-trigger \
    --trigger-button-name BUTTON_RB

Typical rosrun usage, 34-column CSV without header or velocity:
  rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \
    --csv src/humanoid-control/humanoid_controllers/config/kuavo_v55/rl/IMG_9934_bvh_No1_50fps.csv \
    --has-header false \
    --position-columns 7:34 \
    --fps 50 \
    --position-unit rad \
    --wait-for-trigger \
    --trigger-button-name BUTTON_RB

Typical rosrun usage, synchronized arm + waist + dexterous hands:
  rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \
    --csv src/humanoid-control/humanoid_controllers/config/kuavo_v54/rl/IMG_0087_bvh_50fps_dexhand.csv \
    --has-header false \
    --position-columns 7:34 \
    --fps 50 \
    --position-unit rad \
    --publish-waist \
    --publish-hand \
    --hand-columns 54:66 \
    --wait-for-trigger \
    --trigger-button-name BUTTON_RB

Typical rosrun usage, synchronized arm + waist + head + dexterous hands:
  rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \
    --csv motion_with_head.csv \
    --has-header false \
    --position-columns 7:34 \
    --fps 50 \
    --position-unit rad \
    --publish-waist \
    --publish-hand \
    --publish-head \
    --head-unit rad \
    --wait-for-trigger \
    --trigger-button-name BUTTON_RB

Joystick button mapping:
- "右尖键" / right shoulder is BUTTON_RB.
- The default trigger is --trigger-button-name BUTTON_RB. The player resolves it
  from --channel-map-path, or from the ROS param channel_map_path normally set by
  joy_control_bt.launch.
- joystick_type:=bt2, sim, h12pro: BUTTON_RB -> button index 5.
- joystick_type:=bt2pro: BUTTON_RB -> button index 7.
- If no map file is available, BUTTON_RB is inferred from /joy.buttons length:
  bt2-like messages use index 5, bt2pro-like messages use index 7.
- For manual override, pass --trigger-button-index 5 or --trigger-button-index 7.
- If unsure, run `rostopic echo /joy`, press the target button once, and use the
  buttons[] index that changes from 0 to 1.

Real-robot note:
  Some JoyControl versions intentionally zero walking commands in AMP when the
  arm mode is not auto_swing (1). If the robot stops walking while this script
  plays in arm mode 2, that guard is likely active in the joystick node.
"""

import argparse
import csv
import json
import math
import os
import threading
import time

import rospy
from kuavo_msgs.msg import robotHandPosition, robotHeadMotionData, robotWaistControl
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool


# JointState names are not used for indexing by the current arm controller, but
# keeping real joint names makes rostopic echo/logs easier to inspect.
ARM_JOINT_NAMES = [
    "zarm_l1_joint",
    "zarm_l2_joint",
    "zarm_l3_joint",
    "zarm_l4_joint",
    "zarm_l5_joint",
    "zarm_l6_joint",
    "zarm_l7_joint",
    "zarm_r1_joint",
    "zarm_r2_joint",
    "zarm_r3_joint",
    "zarm_r4_joint",
    "zarm_r5_joint",
    "zarm_r6_joint",
    "zarm_r7_joint",
]


def parse_bool(value):
    """Parse command-line boolean strings."""
    if isinstance(value, bool):
        return value
    value = str(value).strip().lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError("expected true or false, got {}".format(value))


def parse_column_index(value, width, default):
    """Parse one CSV column index, allowing negative indices from the row end."""
    if value == "":
        return default
    index = int(value)
    if index < 0:
        index += width
    return index


def parse_columns(spec, width):
    """Parse column specs such as '7:34', '-12:', or '0,2,5:8' into zero-based indices."""
    indices = []
    for item in str(spec).split(","):
        item = item.strip()
        if not item:
            continue
        if ":" in item:
            start, end = item.split(":", 1)
            start = parse_column_index(start.strip(), width, 0)
            end = parse_column_index(end.strip(), width, width)
            if start < 0 or end < 0 or start > width or end > width:
                raise RuntimeError("column spec {} out of range for csv width {}".format(spec, width))
            indices.extend(range(start, end))
        else:
            index = parse_column_index(item, width, 0)
            if index < 0 or index >= width:
                raise RuntimeError("column spec {} out of range for csv width {}".format(spec, width))
            indices.append(index)
    if not indices:
        raise RuntimeError("empty column spec")
    if max(indices) >= width:
        raise RuntimeError("column spec {} out of range for csv width {}".format(spec, width))
    return indices


def infer_header(first_row, has_header):
    """Resolve header mode. In auto mode, a non-numeric first row is a header."""
    if has_header != "auto":
        return parse_bool(has_header)
    try:
        [float(cell) for cell in first_row if cell.strip()]
        return False
    except ValueError:
        return True


def read_csv_rows(path, delimiter, has_header):
    """Read non-empty CSV rows, skip comments, and remove the header if present."""
    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.reader(f, delimiter=delimiter)
        for row in reader:
            if not row or (len(row) == 1 and not row[0].strip()):
                continue
            if row[0].strip().startswith("#"):
                continue
            rows.append([cell.strip() for cell in row])
    if not rows:
        raise RuntimeError("csv is empty: {}".format(path))
    if infer_header(rows[0], has_header):
        rows = rows[1:]
    if not rows:
        raise RuntimeError("csv has no data rows: {}".format(path))
    return rows


def matrix_from_columns(rows, columns):
    """Extract a numeric matrix from selected CSV columns."""
    values = []
    for row_index, row in enumerate(rows):
        try:
            values.append([float(row[col]) for col in columns])
        except (IndexError, ValueError) as exc:
            raise RuntimeError("bad csv row {}: {}".format(row_index + 1, exc))
    return values


def resolve_arm_indices(dof_count, spec):
    """Map selected position columns to the 14 real arm joints.

    If the selected position block has 27 columns, it is assumed to be ordered as
    12 leg joints + 1 waist joint + 14 arm joints, so indices 13:27 are used.
    If it already has 14 columns, all columns are used.
    """
    if spec:
        indices = parse_columns(spec, dof_count)
    elif dof_count == 14:
        indices = list(range(14))
    elif dof_count >= 27:
        indices = list(range(13, 27))
    else:
        raise RuntimeError("cannot infer 14 arm indices from {} columns".format(dof_count))
    if len(indices) != 14:
        raise RuntimeError("arm indices must select 14 columns, got {}".format(len(indices)))
    return indices


def resolve_waist_indices(dof_count, spec):
    """Map selected position columns to waist joints.

    For the S55 27-joint position block, index 12 is waist_yaw_joint.
    """
    if spec:
        indices = parse_columns(spec, dof_count)
    elif dof_count >= 27:
        indices = [12]
    else:
        raise RuntimeError("cannot infer waist index from {} columns".format(dof_count))
    if len(indices) != 1:
        raise RuntimeError("waist indices must select 1 column, got {}".format(len(indices)))
    return indices


def to_degrees(values, unit):
    """Convert input values to degrees, matching the /kuavo_arm_traj convention."""
    if unit in ("deg", "degree", "degrees"):
        return values
    if unit in ("rad", "radian", "radians"):
        return [math.degrees(value) for value in values]
    raise RuntimeError("unsupported unit: {}".format(unit))


def clamp_uint8_percent(value):
    """Convert a CSV hand value to the uint8 0-100 hand command range."""
    return max(0, min(100, int(round(value))))


def call_mode_service(service_name, mode):
    """Call the arm-control mode service and fail loudly if the mode is rejected."""
    rospy.loginfo("waiting for %s", service_name)
    rospy.wait_for_service(service_name, timeout=5.0)
    proxy = rospy.ServiceProxy(service_name, changeArmCtrlMode)
    req = changeArmCtrlModeRequest()
    req.control_mode = int(mode)
    resp = proxy(req)
    if not resp.result:
        raise RuntimeError("{} rejected mode {}: {}".format(service_name, mode, resp.message))
    rospy.loginfo("arm control mode set to %d via %s", mode, service_name)


def load_button_map(path):
    """Load JoyButton mapping from humanoid_controllers/launch/joy/*.json."""
    if not path:
        return {}
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(path):
        rospy.logwarn("channel map path does not exist: %s", path)
        return {}
    with open(path, "r") as f:
        data = json.load(f)
    return data.get("JoyButton", {})


def ros_param_or_empty(*names):
    """Return the first existing ROS parameter from names, or an empty string."""
    for name in names:
        if rospy.has_param(name):
            return rospy.get_param(name)
    return ""


def infer_button_index_from_joy_msg(button_name, msg):
    """Fallback when no map file is available.

    bt2/sim/h12pro publish 8 or 11 buttons and use BUTTON_RB=5.
    bt2pro publishes 15 buttons and uses BUTTON_RB=7.
    """
    if button_name != "BUTTON_RB":
        return -1
    if len(msg.buttons) >= 15:
        return 7
    return 5


class CsvArmTrajectoryPlayer:
    """Owns playback state and optional joystick-triggered mode switching."""

    def __init__(
        self,
        args,
        positions,
        velocities,
        arm_indices,
        waist_indices=None,
        hand_positions=None,
        head_positions=None,
    ):
        self.args = args
        self.positions = positions
        self.velocities = velocities
        self.arm_indices = arm_indices
        self.waist_indices = waist_indices or []
        self.hand_positions = hand_positions
        self.head_positions = head_positions
        self.pub = rospy.Publisher(args.topic, JointState, queue_size=1, tcp_nodelay=True)
        self.waist_pub = None
        self.waist_enable_pub = None
        if self.args.publish_waist:
            self.waist_pub = rospy.Publisher(args.waist_topic, robotWaistControl, queue_size=1, tcp_nodelay=True)
            self.waist_enable_pub = rospy.Publisher(args.waist_enable_topic, Bool, queue_size=1, latch=True)
        self.hand_pub = None
        if self.args.publish_hand:
            self.hand_pub = rospy.Publisher(args.hand_topic, robotHandPosition, queue_size=1, tcp_nodelay=True)
        self.head_pub = None
        if self.args.publish_head:
            self.head_pub = rospy.Publisher(args.head_topic, robotHeadMotionData, queue_size=1, tcp_nodelay=True)

        self.lock = threading.Lock()
        self.playing = False
        self.external_active = False
        self.stop_requested = False
        self.prev_buttons = []
        self.last_trigger_time = rospy.Time(0)
        self.trigger_button_index = self._resolve_trigger_index_from_config()

    def wait_for_subscribers(self):
        """Give ROS subscribers a short chance to connect before publishing."""
        deadline = time.time() + 2.0
        while not rospy.is_shutdown() and time.time() < deadline:
            arm_ready = self.pub.get_num_connections() > 0
            waist_ready = not self.args.publish_waist or self.waist_pub.get_num_connections() > 0
            hand_ready = not self.args.publish_hand or self.hand_pub.get_num_connections() > 0
            head_ready = not self.args.publish_head or self.head_pub.get_num_connections() > 0
            if arm_ready and waist_ready and hand_ready and head_ready:
                break
            rospy.sleep(0.02)

    def play_blocking(self):
        """One-shot command-line mode: switch to external mode and play once."""
        if self.args.set_mode:
            call_mode_service(self.args.mode_service, self.args.external_mode)
        self._set_waist_external(True)
        self.wait_for_subscribers()
        rospy.loginfo("publishing %d frames at %.1f fps", len(self.positions), self.args.fps)
        self._publish_frames()

    def start_trigger_mode(self):
        """Joystick mode: stay alive and toggle playback on button rising edges."""
        self.wait_for_subscribers()
        rospy.Subscriber(self.args.joy_topic, Joy, self._joy_cb, queue_size=10)
        rospy.loginfo(
            "waiting for joystick trigger on %s button %s(index=%s): play external csv traj <-> RL mode %d",
            self.args.joy_topic,
            self.args.trigger_button_name,
            "auto" if self.trigger_button_index < 0 else str(self.trigger_button_index),
            self.args.rl_mode,
        )
        rospy.spin()

    def _joy_cb(self, msg):
        """Handle a single joystick message and detect the trigger rising edge."""
        idx = self.trigger_button_index
        if idx < 0:
            idx = infer_button_index_from_joy_msg(self.args.trigger_button_name, msg)
            if idx >= 0:
                self.trigger_button_index = idx
                rospy.loginfo(
                    "resolved joystick trigger %s to button index %d from /joy button count %d",
                    self.args.trigger_button_name,
                    idx,
                    len(msg.buttons),
                )
        if idx < 0 or idx >= len(msg.buttons):
            self.prev_buttons = list(msg.buttons)
            return

        old = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
        new = msg.buttons[idx]
        self.prev_buttons = list(msg.buttons)
        if old != 0 or new != 1:
            return

        now = rospy.Time.now()
        if (now - self.last_trigger_time).to_sec() < self.args.debounce_sec:
            return
        self.last_trigger_time = now
        threading.Thread(target=self.toggle, daemon=True).start()

    def toggle(self):
        """Toggle between external CSV playback and normal arm auto_swing mode."""
        with self.lock:
            should_return_to_rl = self.external_active or self.playing
            if should_return_to_rl:
                self.stop_requested = True
            elif self.playing:
                return
            else:
                self.playing = True
                self.stop_requested = False
                self.external_active = True

        if should_return_to_rl:
            try:
                call_mode_service(self.args.mode_service, self.args.rl_mode)
                self._set_waist_external(False)
                rospy.loginfo("switched arm back to RL mode %d", self.args.rl_mode)
            except Exception as exc:
                rospy.logerr("failed to switch arm back to RL mode %d: %s", self.args.rl_mode, exc)
            with self.lock:
                self.external_active = False
            return

        try:
            call_mode_service(self.args.mode_service, self.args.external_mode)
            self._set_waist_external(True)
            rospy.loginfo("play arm trajectory from beginning")
            self._publish_frames()
            with self.lock:
                stopped = self.stop_requested
            if not stopped:
                rospy.loginfo("arm trajectory finished; press trigger again to switch back to RL mode %d", self.args.rl_mode)
        except Exception as exc:
            rospy.logerr("failed to play arm trajectory: %s", exc)
            with self.lock:
                self.external_active = False
        finally:
            with self.lock:
                self.playing = False
                self.stop_requested = False

    def _publish_frames(self):
        """Publish CSV frames at the requested FPS until done or stopped."""
        rate = rospy.Rate(self.args.fps)
        while not rospy.is_shutdown():
            for frame_index in range(len(self.positions)):
                if self._should_stop():
                    return
                self.pub.publish(self._build_arm_msg(frame_index))
                if self.args.publish_waist:
                    self.waist_pub.publish(self._build_waist_msg(frame_index))
                if self.args.publish_hand:
                    self.hand_pub.publish(self._build_hand_msg(frame_index))
                if self.args.publish_head:
                    self.head_pub.publish(self._build_head_msg(frame_index))
                rate.sleep()
            if not self.args.loop:
                break

    def _build_arm_msg(self, frame_index):
        """Build one /kuavo_arm_traj JointState frame in degrees."""
        full_q = self.positions[frame_index]
        q_deg = to_degrees([full_q[idx] for idx in self.arm_indices], self.args.position_unit)
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = q_deg
        if self.velocities is not None:
            full_v = self.velocities[frame_index]
            msg.velocity = to_degrees([full_v[idx] for idx in self.arm_indices], self.args.velocity_unit)
        return msg

    def _build_waist_msg(self, frame_index):
        """Build one /robot_waist_motion_data frame in degrees."""
        full_q = self.positions[frame_index]
        waist_unit = self.args.waist_unit or self.args.position_unit
        q_deg = to_degrees([full_q[idx] for idx in self.waist_indices], waist_unit)
        msg = robotWaistControl()
        msg.header.stamp = rospy.Time.now()
        msg.data.data = q_deg
        return msg

    def _build_hand_msg(self, frame_index):
        """Build one /control_robot_hand_position frame with left6 + right6 values."""
        values = [clamp_uint8_percent(value) for value in self.hand_positions[frame_index]]
        msg = robotHandPosition()
        msg.header.stamp = rospy.Time.now()
        msg.left_hand_position = values[:6]
        msg.right_hand_position = values[6:]
        return msg

    def _build_head_msg(self, frame_index):
        """Build one /robot_head_motion_data frame with yaw + pitch in degrees."""
        head_unit = self.args.head_unit or self.args.position_unit
        q_deg = to_degrees(self.head_positions[frame_index], head_unit)
        msg = robotHeadMotionData()
        msg.joint_data = q_deg
        return msg

    def _set_waist_external(self, enabled):
        """Enable or disable RL waist external-control override."""
        if not self.args.publish_waist:
            return
        self.waist_enable_pub.publish(Bool(data=bool(enabled)))
        rospy.sleep(self.args.waist_enable_wait_sec)

    def _should_stop(self):
        """Thread-safe stop flag check used by the playback loop."""
        with self.lock:
            return self.stop_requested

    def _resolve_trigger_index_from_config(self):
        """Resolve trigger index from CLI override or active channel map file."""
        if self.args.trigger_button_index >= 0:
            return self.args.trigger_button_index

        channel_map_path = self.args.channel_map_path or ros_param_or_empty("channel_map_path", "/channel_map_path")
        button_map = load_button_map(channel_map_path)
        if self.args.trigger_button_name in button_map:
            idx = int(button_map[self.args.trigger_button_name])
            rospy.loginfo(
                "resolved joystick trigger %s to button index %d from %s",
                self.args.trigger_button_name,
                idx,
                channel_map_path,
            )
            return idx

        if channel_map_path:
            rospy.logwarn(
                "button %s not found in channel map %s; will infer from /joy message",
                self.args.trigger_button_name,
                channel_map_path,
            )
        return -1


def main():
    parser = argparse.ArgumentParser(
        description="Publish CSV arm trajectory to /kuavo_arm_traj, optionally with waist, hand, and head trajectory.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  # Compatible trigger: uses BUTTON_RB from channel_map_path, so bt2 and bt2pro
  # do not need different commands.
  rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \\
    --csv motion_deploy.csv --has-header true --position-columns 7:34 \\
    --velocity-columns 34:61 --position-unit rad --velocity-unit rad \\
    --wait-for-trigger --trigger-button-name BUTTON_RB

  # Synchronized arm + waist. For S55 27-joint blocks, waist is inferred as
  # selected position-block index 12.
  rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \\
    --csv motion_deploy.csv --has-header true --position-columns 7:34 \\
    --velocity-columns 34:61 --position-unit rad --velocity-unit rad \\
    --publish-waist --wait-for-trigger --trigger-button-name BUTTON_RB

  # Manual override if channel_map_path is not available: bt2 RB=5, bt2pro RB=7.
  rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \\
    --csv motion_deploy.csv --has-header true --position-columns 7:34 \\
    --velocity-columns 34:61 --position-unit rad --velocity-unit rad \\
    --wait-for-trigger --trigger-button-index 7

  # Play once immediately from a position-only CSV.
  rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \\
    --csv motion.csv --has-header false --position-columns 7:34 \\
    --position-unit rad

  # Play a dexhand CSV whose last 12 columns are remapped hand percent commands.
  rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \\
    --csv motion_dexhand.csv --has-header false --position-columns 7:34 \\
    --position-unit rad --publish-waist --publish-hand --hand-columns 54:66

  # Play synchronized arm + waist + head + hand targets. Defaults assume the
  # last 12 CSV columns are fingers, and the two columns before them are head
  # yaw,pitch.
  rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \\
    --csv motion_with_head.csv --has-header false --position-columns 7:34 \\
    --position-unit rad --publish-waist --publish-hand --publish-head \\
    --head-unit rad

Button mapping:
  BUTTON_RB is the right shoulder / "右尖键".
  bt2, sim, h12pro: BUTTON_RB = 5.
  bt2pro: BUTTON_RB = 7.
  Default resolve order:
    --trigger-button-index override
    --channel-map-path or ROS param channel_map_path
    fallback from /joy.buttons length for BUTTON_RB
  Check the live index with: rostopic echo /joy
""",
    )
    parser.add_argument("--csv", required=True, help="CSV path")
    parser.add_argument("--fps", type=float, default=50.0, help="CSV playback frequency")
    parser.add_argument("--delimiter", default=",", help="CSV delimiter")
    parser.add_argument("--has-header", default="auto", choices=("auto", "true", "false"), help="CSV header handling")
    parser.add_argument("--position-columns", default="7:34", help="CSV columns containing joint positions")
    parser.add_argument("--velocity-columns", default="", help="CSV columns containing joint velocities; leave empty if absent")
    parser.add_argument("--arm-indices", default="", help="Optional 14 indices inside the selected position block")
    parser.add_argument("--publish-waist", action="store_true", help="Also publish waist_yaw_joint to /robot_waist_motion_data")
    parser.add_argument("--waist-indices", default="", help="Optional waist index inside the selected position block; S55 default is 12")
    parser.add_argument("--waist-unit", default="", choices=("", "rad", "deg"), help="Unit of selected waist values; defaults to --position-unit")
    parser.add_argument("--waist-topic", default="/robot_waist_motion_data", help="Waist command topic")
    parser.add_argument("--waist-enable-topic", default="/humanoid_controller/enable_waist_control", help="Waist external-control enable topic")
    parser.add_argument("--waist-enable-wait-sec", type=float, default=0.05, help="Delay after toggling waist external control")
    parser.add_argument("--publish-hand", action="store_true", help="Also publish dexterous hand commands to /control_robot_hand_position")
    parser.add_argument("--hand-columns", default="-12:", help="CSV columns containing 12 remapped hand values: left6 then right6")
    parser.add_argument("--hand-topic", default="/control_robot_hand_position", help="Dexterous hand command topic")
    parser.add_argument("--publish-head", action="store_true", help="Also publish head yaw/pitch commands to /robot_head_motion_data")
    parser.add_argument("--head-columns", default="-14:-12", help="CSV columns containing head yaw,pitch targets")
    parser.add_argument("--head-unit", default="", choices=("", "rad", "deg"), help="Unit of selected head values; defaults to --position-unit")
    parser.add_argument("--head-topic", default="/robot_head_motion_data", help="Head command topic")
    parser.add_argument("--position-unit", default="rad", choices=("rad", "deg"), help="Unit of CSV position values")
    parser.add_argument("--velocity-unit", default="rad", choices=("rad", "deg"), help="Unit of CSV velocity values")
    parser.add_argument("--topic", default="/kuavo_arm_traj", help="JointState topic consumed by the arm controller")
    parser.add_argument("--set-mode", type=parse_bool, default=True, help="Whether to call the arm mode service before playback")
    parser.add_argument("--mode-service", default="/arm_traj_change_mode", help="Arm mode service name")
    parser.add_argument("--external-mode", type=int, default=2, help="Arm mode used for external /kuavo_arm_traj control")
    parser.add_argument("--rl-mode", type=int, default=1, help="Arm mode restored on the second trigger press")
    parser.add_argument("--loop", action="store_true", help="Loop CSV playback until stopped")
    parser.add_argument("--wait-for-trigger", action="store_true", help="Wait for joystick trigger instead of playing immediately")
    parser.add_argument("--joy-topic", default="/joy", help="Joystick topic")
    parser.add_argument("--trigger-button-name", default="BUTTON_RB", help="JoyButton name used for trigger mode")
    parser.add_argument("--trigger-button-index", type=int, default=-1, help="Manual button index override; bt2/sim/h12pro RB=5, bt2pro RB=7")
    parser.add_argument("--channel-map-path", default="", help="Optional joystick mapping JSON; defaults to ROS param channel_map_path")
    parser.add_argument("--debounce-sec", type=float, default=0.5, help="Minimum seconds between accepted trigger presses")
    args = parser.parse_args()

    rospy.init_node("csv_arm_traj_player", anonymous=True)

    path = os.path.abspath(os.path.expanduser(args.csv))
    rows = read_csv_rows(path, args.delimiter, args.has_header)
    width = max(len(row) for row in rows)
    position_columns = parse_columns(args.position_columns, width)
    positions = matrix_from_columns(rows, position_columns)
    arm_indices = resolve_arm_indices(len(position_columns), args.arm_indices)
    waist_indices = resolve_waist_indices(len(position_columns), args.waist_indices) if args.publish_waist else []
    hand_positions = None
    if args.publish_hand:
        hand_columns = parse_columns(args.hand_columns, width)
        if len(hand_columns) != 12:
            raise RuntimeError("hand columns must select 12 columns, got {}".format(len(hand_columns)))
        hand_positions = matrix_from_columns(rows, hand_columns)
    head_positions = None
    if args.publish_head:
        head_columns = parse_columns(args.head_columns, width)
        if len(head_columns) != 2:
            raise RuntimeError("head columns must select 2 columns, got {}".format(len(head_columns)))
        head_positions = matrix_from_columns(rows, head_columns)

    # Velocity is optional. If absent, the downstream controller estimates or
    # filters velocity from position changes depending on its current mode.
    velocities = None
    if args.velocity_columns:
        velocity_columns = parse_columns(args.velocity_columns, width)
        raw_velocities = matrix_from_columns(rows, velocity_columns)
        if len(velocity_columns) != len(position_columns):
            raise RuntimeError("velocity column count must match position column count")
        velocities = raw_velocities

    player = CsvArmTrajectoryPlayer(args, positions, velocities, arm_indices, waist_indices, hand_positions, head_positions)
    rospy.loginfo("loaded %d frames from %s", len(positions), path)
    if args.wait_for_trigger:
        player.start_trigger_mode()
    else:
        player.play_blocking()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        raise
