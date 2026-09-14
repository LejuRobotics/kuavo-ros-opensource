#!/usr/bin/env python3

import argparse
import bz2
import struct
import sys
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, Tuple

import rospy
from geometry_msgs.msg import Point, Quaternion
from kuavo_msgs.msg import RigidBodyDescription, xsensePoseInfo, xsensePoseInfoList


OP_MSG_DATA = 0x02
OP_CHUNK = 0x05
OP_CONNECTION = 0x07

DEFAULT_BAG_PATH = (
    Path(__file__).resolve().parents[1]
    / "data"
    / "xsens_bvh_udp_20260623_201247.bag"
)

POSE_INPUT_TOPIC = "/xsens_udp/pose_quaternion"
SKELETON_INPUT_TOPIC = "/xsens_bvh/skeleton"
POSE_OUTPUT_TOPIC = "/xsense/world_bone_poses"
SKELETON_OUTPUT_TOPIC = "/xsense/bvh_seg_desc"

XSENSE_BODY_NAMES = [
    "Hips",
    "Chest",
    "Neck",
    "Head",
    "LeftCollar",
    "LeftShoulder",
    "LeftElbow",
    "LeftWrist",
    "RightCollar",
    "RightShoulder",
    "RightElbow",
    "RightWrist",
    "LeftHip",
    "LeftKnee",
    "LeftAnkle",
    "LeftToe",
    "RightHip",
    "RightKnee",
    "RightAnkle",
    "RightToe",
]

XSENS_SEGMENT_BY_BODY_NAME = {
    "Hips": "Pelvis",
    "Chest": "L5",
    "Neck": "Neck",
    "Head": "Head",
    "LeftCollar": "Left Shoulder",
    "LeftShoulder": "Left Upper Arm",
    "LeftElbow": "Left Forearm",
    "LeftWrist": "Left Hand",
    "RightCollar": "Right Shoulder",
    "RightShoulder": "Right Upper Arm",
    "RightElbow": "Right Forearm",
    "RightWrist": "Right Hand",
    "LeftHip": "Left Upper Leg",
    "LeftKnee": "Left Lower Leg",
    "LeftAnkle": "Left Foot",
    "LeftToe": "Left Toe",
    "RightHip": "Right Upper Leg",
    "RightKnee": "Right Lower Leg",
    "RightAnkle": "Right Foot",
    "RightToe": "Right Toe",
}


def _read_u32_file(f):
    raw = f.read(4)
    if len(raw) < 4:
        return None
    return struct.unpack("<I", raw)[0]


def _parse_record_header(raw: bytes) -> Dict[str, bytes]:
    fields = {}
    offset = 0
    while offset + 4 <= len(raw):
        field_len = struct.unpack_from("<I", raw, offset)[0]
        offset += 4
        field = raw[offset : offset + field_len]
        offset += field_len
        if b"=" not in field:
            continue
        key, value = field.split(b"=", 1)
        fields[key.decode("ascii", "replace")] = value
    return fields


def _read_field_u32(header: Dict[str, bytes], key: str) -> int:
    return struct.unpack("<I", header[key])[0]


def _read_field_time(header: Dict[str, bytes], key: str) -> float:
    sec, nsec = struct.unpack("<II", header[key])
    return float(sec) + float(nsec) * 1e-9


def _records_from_bytes(raw: bytes) -> Iterator[Tuple[Dict[str, bytes], bytes]]:
    offset = 0
    while offset + 8 <= len(raw):
        header_len = struct.unpack_from("<I", raw, offset)[0]
        offset += 4
        header = _parse_record_header(raw[offset : offset + header_len])
        offset += header_len
        if offset + 4 > len(raw):
            break
        data_len = struct.unpack_from("<I", raw, offset)[0]
        offset += 4
        data = raw[offset : offset + data_len]
        offset += data_len
        yield header, data


class RawRosbagReader:
    def __init__(self, bag_path: Path):
        self.bag_path = bag_path

    def iter_messages(
        self,
        topics: Optional[Iterable[str]] = None,
    ) -> Iterator[Tuple[str, str, float, bytes]]:
        topic_filter = set(topics) if topics is not None else None
        connections = {}

        with self.bag_path.open("rb") as f:
            magic = f.readline().rstrip(b"\n")
            if magic != b"#ROSBAG V2.0":
                raise ValueError(f"{self.bag_path} is not a ROS1 bag v2 file")

            while not rospy.is_shutdown():
                header_len = _read_u32_file(f)
                if header_len is None:
                    break
                header = _parse_record_header(f.read(header_len))
                data_len = _read_u32_file(f)
                if data_len is None:
                    break
                data = f.read(data_len)
                op = header.get("op", b"\x00")[0]

                if op == OP_CONNECTION:
                    self._remember_connection(connections, header, data)
                elif op == OP_CHUNK:
                    compression = header.get("compression", b"none").decode("ascii", "replace")
                    if compression == "none":
                        chunk = data
                    elif compression == "bz2":
                        chunk = bz2.decompress(data)
                    else:
                        raise ValueError(f"unsupported bag chunk compression: {compression}")

                    for inner_header, inner_data in _records_from_bytes(chunk):
                        inner_op = inner_header.get("op", b"\x00")[0]
                        if inner_op == OP_CONNECTION:
                            self._remember_connection(connections, inner_header, inner_data)
                        elif inner_op == OP_MSG_DATA:
                            conn_id = _read_field_u32(inner_header, "conn")
                            conn = connections.get(conn_id)
                            if conn is None:
                                continue
                            topic = conn["topic"]
                            if topic_filter is not None and topic not in topic_filter:
                                continue
                            yield (
                                topic,
                                conn["type"],
                                _read_field_time(inner_header, "time"),
                                inner_data,
                            )

    @staticmethod
    def _remember_connection(connections, header, data):
        conn_id = _read_field_u32(header, "conn")
        conn_header = _parse_record_header(data)
        connections[conn_id] = {
            "topic": header.get("topic", b"").decode("utf-8", "replace"),
            "type": conn_header.get("type", b"").decode("utf-8", "replace"),
        }


def _read_u32(buf: bytes, offset: int) -> Tuple[int, int]:
    return struct.unpack_from("<I", buf, offset)[0], offset + 4


def _read_u16(buf: bytes, offset: int) -> Tuple[int, int]:
    return struct.unpack_from("<H", buf, offset)[0], offset + 2


def _read_u8(buf: bytes, offset: int) -> Tuple[int, int]:
    return struct.unpack_from("<B", buf, offset)[0], offset + 1


def _read_i32(buf: bytes, offset: int) -> Tuple[int, int]:
    return struct.unpack_from("<i", buf, offset)[0], offset + 4


def _read_bool(buf: bytes, offset: int) -> Tuple[bool, int]:
    return struct.unpack_from("<?", buf, offset)[0], offset + 1


def _read_f64(buf: bytes, offset: int) -> Tuple[float, int]:
    return struct.unpack_from("<d", buf, offset)[0], offset + 8


def _read_string(buf: bytes, offset: int) -> Tuple[str, int]:
    length, offset = _read_u32(buf, offset)
    text = buf[offset : offset + length].decode("utf-8", "replace")
    return text, offset + length


def _read_header_msg(buf: bytes, offset: int):
    seq, offset = _read_u32(buf, offset)
    sec, offset = _read_u32(buf, offset)
    nsec, offset = _read_u32(buf, offset)
    frame_id, offset = _read_string(buf, offset)
    return {"seq": seq, "stamp": sec + nsec * 1e-9, "frame_id": frame_id}, offset


def _read_point(buf: bytes, offset: int):
    x, offset = _read_f64(buf, offset)
    y, offset = _read_f64(buf, offset)
    z, offset = _read_f64(buf, offset)
    return (x, y, z), offset


def _read_quaternion(buf: bytes, offset: int):
    x, offset = _read_f64(buf, offset)
    y, offset = _read_f64(buf, offset)
    z, offset = _read_f64(buf, offset)
    w, offset = _read_f64(buf, offset)
    return (x, y, z, w), offset


def _read_bvh_joint(buf: bytes, offset: int):
    name, offset = _read_string(buf, offset)
    parent_index, offset = _read_i32(buf, offset)
    parent_name, offset = _read_string(buf, offset)
    is_end_site, offset = _read_bool(buf, offset)
    joint_offset, offset = _read_point(buf, offset)
    channel_count, offset = _read_u32(buf, offset)
    channels = []
    for _ in range(channel_count):
        channel, offset = _read_string(buf, offset)
        channels.append(channel)
    return (
        {
            "name": name,
            "parent_index": parent_index,
            "parent_name": parent_name,
            "is_end_site": is_end_site,
            "offset": joint_offset,
            "channels": channels,
        },
        offset,
    )


def read_bvh_skeleton(buf: bytes):
    offset = 0
    header, offset = _read_header_msg(buf, offset)
    source_file, offset = _read_string(buf, offset)
    source_host, offset = _read_string(buf, offset)
    frame_count, offset = _read_u32(buf, offset)
    frame_time, offset = _read_f64(buf, offset)
    unit_hint, offset = _read_string(buf, offset)
    joint_count, offset = _read_u32(buf, offset)
    joints = []
    for _ in range(joint_count):
        joint, offset = _read_bvh_joint(buf, offset)
        joints.append(joint)
    return {
        "header": header,
        "source_file": source_file,
        "source_host": source_host,
        "frame_count": frame_count,
        "frame_time": frame_time,
        "unit_hint": unit_hint,
        "joints": joints,
    }


def _read_xsens_header(buf: bytes, offset: int):
    header = {}
    header["id_string"], offset = _read_string(buf, offset)
    header["message_type"], offset = _read_string(buf, offset)
    header["sample_counter"], offset = _read_u32(buf, offset)
    header["datagram_counter"], offset = _read_u8(buf, offset)
    header["fragment_index"], offset = _read_u8(buf, offset)
    header["is_last_fragment"], offset = _read_bool(buf, offset)
    header["number_of_items"], offset = _read_u8(buf, offset)
    header["time_code"], offset = _read_u32(buf, offset)
    header["character_id"], offset = _read_u8(buf, offset)
    header["body_segments"], offset = _read_u8(buf, offset)
    header["props"], offset = _read_u8(buf, offset)
    header["finger_tracking_segments"], offset = _read_u8(buf, offset)
    header["reserved"], offset = _read_u16(buf, offset)
    header["payload_size"], offset = _read_u16(buf, offset)
    header["datagram_size"], offset = _read_u32(buf, offset)
    header["reconstructed"], offset = _read_bool(buf, offset)
    header["coordinate_system"], offset = _read_string(buf, offset)
    return header, offset


def read_pose_quaternion(buf: bytes):
    offset = 0
    xsens_header, offset = _read_xsens_header(buf, offset)
    segment_count, offset = _read_u32(buf, offset)
    segments = []
    for _ in range(segment_count):
        segment_id, offset = _read_u32(buf, offset)
        segment_name, offset = _read_string(buf, offset)
        position, offset = _read_point(buf, offset)
        orientation, offset = _read_quaternion(buf, offset)
        segments.append(
            {
                "segment_id": segment_id,
                "name": segment_name,
                "position": position,
                "orientation": orientation,
            }
        )
    return {"xsens_header": xsens_header, "segments": segments}


def _offset_scale_for_skeleton(skeleton, override: Optional[float]) -> float:
    if override is not None:
        return override
    unit_hint = str(skeleton.get("unit_hint", "")).strip().lower()
    if unit_hint in ("cm", "centimeter", "centimeters"):
        return 0.01
    return 1.0


def build_rigid_body_description(
    skeleton,
    stamp: rospy.Time,
    offset_scale_override: Optional[float],
) -> RigidBodyDescription:
    offset_scale = _offset_scale_for_skeleton(skeleton, offset_scale_override)
    msg = RigidBodyDescription()
    msg.header.stamp = stamp
    msg.header.frame_id = skeleton.get("header", {}).get("frame_id", "xsens_bvh")

    for index, joint in enumerate(skeleton["joints"]):
        msg.ids.append(index + 1)
        msg.names.append(joint["name"])
        parent_index = int(joint["parent_index"])
        msg.parent_ids.append(0 if parent_index < 0 else parent_index + 1)
        x, y, z = joint["offset"]
        msg.offsets.append(
            Point(
                x=float(x) * offset_scale,
                y=float(y) * offset_scale,
                z=float(z) * offset_scale,
            )
        )

    return msg


def build_pose_info_list(
    frame,
    stamp: rospy.Time,
    position_scale: float,
) -> xsensePoseInfoList:
    segments_by_name = {segment["name"]: segment for segment in frame["segments"]}
    msg = xsensePoseInfoList()
    msg.header.stamp = stamp
    msg.header.frame_id = "xsens_bvh"

    for output_name in XSENSE_BODY_NAMES:
        source_name = XSENS_SEGMENT_BY_BODY_NAME[output_name]
        if source_name not in segments_by_name:
            raise KeyError(f"missing segment {source_name!r} for output joint {output_name!r}")
        segment = segments_by_name[source_name]
        px, py, pz = segment["position"]
        qx, qy, qz, qw = segment["orientation"]

        pose = xsensePoseInfo()
        pose.name = output_name
        pose.segment_id = int(segment["segment_id"])
        pose.position = Point(
            x=float(px) * position_scale,
            y=float(py) * position_scale,
            z=float(pz) * position_scale,
        )
        pose.orientation = Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))
        msg.poses.append(pose)

    return msg


def find_first_skeleton(reader: RawRosbagReader, topic: str):
    for in_topic, msg_type, bag_time, data in reader.iter_messages([topic]):
        if msg_type != "xsens_udp_bridge/BvhSkeleton":
            rospy.logwarn("Unexpected skeleton topic type %s on %s", msg_type, in_topic)
        return bag_time, read_bvh_skeleton(data)
    return None, None


def publish_skeleton(reader, publisher, args) -> bool:
    bag_time, skeleton = find_first_skeleton(reader, args.skeleton_input_topic)
    if skeleton is None:
        rospy.logerr("No skeleton message found on %s", args.skeleton_input_topic)
        return False

    stamp = rospy.Time.from_sec(bag_time) if args.preserve_bag_stamp else rospy.Time.now()
    msg = build_rigid_body_description(skeleton, stamp, args.offset_scale)
    repeats = max(1, int(args.skeleton_repeats))
    for _ in range(repeats):
        if rospy.is_shutdown():
            return False
        publisher.publish(msg)
        rospy.sleep(0.05)

    rospy.loginfo(
        "Published %d-segment skeleton to %s (unit_hint=%s)",
        len(msg.ids),
        args.skeleton_output_topic,
        skeleton.get("unit_hint", ""),
    )
    return True


def play_pose_stream(reader, publisher, args) -> int:
    published = 0
    last_bag_time = None

    for topic, msg_type, bag_time, data in reader.iter_messages([args.pose_input_topic]):
        if rospy.is_shutdown():
            break
        if msg_type != "xsens_udp_bridge/PoseQuaternion":
            rospy.logwarn_throttle(2.0, "Unexpected pose topic type %s on %s", msg_type, topic)

        if last_bag_time is not None and args.rate_scale > 0.0:
            delay = (bag_time - last_bag_time) / args.rate_scale
            if delay > 0.0:
                rospy.sleep(delay)
        last_bag_time = bag_time

        frame = read_pose_quaternion(data)
        stamp = rospy.Time.from_sec(bag_time) if args.preserve_bag_stamp else rospy.Time.now()
        try:
            msg = build_pose_info_list(frame, stamp, args.position_scale)
        except KeyError as exc:
            rospy.logwarn_throttle(2.0, "Skipping pose frame: %s", exc)
            continue

        publisher.publish(msg)
        published += 1
        if args.max_frames > 0 and published >= args.max_frames:
            break

    return published


def parse_args(argv: List[str]):
    parser = argparse.ArgumentParser(
        description="Republish the Xsens UDP ROS bag as /xsense retargeting inputs."
    )
    parser.add_argument("--bag", default=str(DEFAULT_BAG_PATH))
    parser.add_argument("--pose-input-topic", default=POSE_INPUT_TOPIC)
    parser.add_argument("--skeleton-input-topic", default=SKELETON_INPUT_TOPIC)
    parser.add_argument("--pose-output-topic", default=POSE_OUTPUT_TOPIC)
    parser.add_argument("--skeleton-output-topic", default=SKELETON_OUTPUT_TOPIC)
    parser.add_argument("--rate-scale", type=float, default=1.0)
    parser.add_argument("--position-scale", type=float, default=1.0)
    parser.add_argument("--offset-scale", type=float, default=None)
    parser.add_argument("--skeleton-repeats", type=int, default=3)
    parser.add_argument("--startup-delay", type=float, default=0.5)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--preserve-bag-stamp", action="store_true")
    return parser.parse_args(argv)


def apply_ros_param_overrides(args):
    args.bag = rospy.get_param("~bag", args.bag)
    args.pose_input_topic = rospy.get_param("~pose_input_topic", args.pose_input_topic)
    args.skeleton_input_topic = rospy.get_param("~skeleton_input_topic", args.skeleton_input_topic)
    args.pose_output_topic = rospy.get_param("~pose_output_topic", args.pose_output_topic)
    args.skeleton_output_topic = rospy.get_param("~skeleton_output_topic", args.skeleton_output_topic)
    args.rate_scale = float(rospy.get_param("~rate_scale", args.rate_scale))
    args.position_scale = float(rospy.get_param("~position_scale", args.position_scale))
    offset_scale = rospy.get_param("~offset_scale", args.offset_scale)
    args.offset_scale = None if offset_scale in (None, "") else float(offset_scale)
    args.skeleton_repeats = int(rospy.get_param("~skeleton_repeats", args.skeleton_repeats))
    args.startup_delay = float(rospy.get_param("~startup_delay", args.startup_delay))
    args.max_frames = int(rospy.get_param("~max_frames", args.max_frames))
    args.loop = bool(rospy.get_param("~loop", args.loop))
    args.preserve_bag_stamp = bool(
        rospy.get_param("~preserve_bag_stamp", args.preserve_bag_stamp)
    )
    args.bag = str(Path(args.bag).expanduser())
    return args


def main():
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    rospy.init_node("xsens_bag_to_xsense_topics", anonymous=False)
    args = apply_ros_param_overrides(args)

    bag_path = Path(args.bag)
    if not bag_path.exists():
        rospy.logerr("Bag file does not exist: %s", bag_path)
        return 1

    pose_pub = rospy.Publisher(args.pose_output_topic, xsensePoseInfoList, queue_size=10)
    skeleton_pub = rospy.Publisher(
        args.skeleton_output_topic,
        RigidBodyDescription,
        queue_size=1,
        latch=True,
    )

    rospy.sleep(max(0.0, args.startup_delay))
    rospy.loginfo("Reading Xsens bag: %s", bag_path)
    rospy.loginfo("Publishing poses: %s -> %s", args.pose_input_topic, args.pose_output_topic)
    rospy.loginfo(
        "Publishing skeleton: %s -> %s",
        args.skeleton_input_topic,
        args.skeleton_output_topic,
    )

    total = 0
    while not rospy.is_shutdown():
        reader = RawRosbagReader(bag_path)
        if not publish_skeleton(reader, skeleton_pub, args):
            return 1

        reader = RawRosbagReader(bag_path)
        count = play_pose_stream(reader, pose_pub, args)
        total += count
        rospy.loginfo("Published %d pose frames this pass (%d total)", count, total)

        if not args.loop or args.max_frames > 0:
            break

    return 0


if __name__ == "__main__":
    sys.exit(main())
