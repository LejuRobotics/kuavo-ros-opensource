#!/usr/bin/env python3

import json
import socket
import struct
import sys

import rospy
from geometry_msgs.msg import Point
from kuavo_msgs.msg import RigidBodyDescription


PROTOCOL = "xsens_bvh_desc_v1"
TOPIC_NAME = "/xsense/bvh_seg_desc"
DEFAULT_FRAME_ID = "xsense_bvh"
LISTEN_IP = "0.0.0.0"
BVH_READY_TEXT = "bvh文件已解析"


class BvhPayloadError(RuntimeError):
    pass


def print_flush(text):
    sys.stdout.write(text + "\n")
    sys.stdout.flush()


def recv_exact(sock, length):
    chunks = []
    remaining = length
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise BvhPayloadError("连接提前关闭，未收到完整数据")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def send_json(sock, payload):
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    sock.sendall(struct.pack(">I", len(data)) + data)


def read_length_prefixed_json(sock, max_payload_bytes):
    header = recv_exact(sock, 4)
    payload_size = struct.unpack(">I", header)[0]
    if payload_size <= 0:
        raise BvhPayloadError("payload长度为0")
    if payload_size > max_payload_bytes:
        raise BvhPayloadError(
            "payload过大：{} bytes，最大允许 {} bytes".format(
                payload_size, max_payload_bytes
            )
        )

    raw_payload = recv_exact(sock, payload_size)
    try:
        return json.loads(raw_payload.decode("utf-8"))
    except UnicodeDecodeError as exc:
        raise BvhPayloadError("payload不是合法UTF-8") from exc
    except json.JSONDecodeError as exc:
        raise BvhPayloadError("payload不是合法JSON：{}".format(exc)) from exc


def _validate_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BvhPayloadError("{} 必须是数字".format(label))
    return float(value)


def validate_bvh_payload(payload):
    if not isinstance(payload, dict):
        raise BvhPayloadError("payload必须是JSON对象")
    if payload.get("protocol") != PROTOCOL:
        raise BvhPayloadError("protocol错误，应为 {}".format(PROTOCOL))
    if payload.get("unit") != "m":
        raise BvhPayloadError("unit错误，应为m，Windows端需要先把BVH offset转换成米")

    joints = payload.get("joints")
    if not isinstance(joints, list) or not joints:
        raise BvhPayloadError("joints必须是非空数组")

    expected_id = 1
    validated_joints = []
    names = set()
    for index, joint in enumerate(joints):
        if not isinstance(joint, dict):
            raise BvhPayloadError("joints[{}]必须是对象".format(index))

        joint_id = joint.get("id")
        if not isinstance(joint_id, int) or joint_id != expected_id:
            raise BvhPayloadError(
                "joint id必须从1开始连续递增，当前期望{}，实际{}".format(
                    expected_id, joint_id
                )
            )

        name = joint.get("name")
        if not isinstance(name, str) or not name.strip():
            raise BvhPayloadError("joint {} 的name为空".format(joint_id))
        name = name.strip()
        if name in names:
            raise BvhPayloadError("joint name重复：{}".format(name))
        names.add(name)

        parent_id = joint.get("parent_id")
        if not isinstance(parent_id, int):
            raise BvhPayloadError("joint {} 的parent_id必须是整数".format(joint_id))
        if parent_id < 0 or parent_id >= joint_id:
            raise BvhPayloadError(
                "joint {} 的parent_id非法，应为0或小于当前id".format(joint_id)
            )

        offset = joint.get("offset")
        if not isinstance(offset, list) or len(offset) != 3:
            raise BvhPayloadError("joint {} 的offset必须是3个数字".format(joint_id))
        offset_xyz = [
            _validate_number(offset[0], "joint {} offset[0]".format(joint_id)),
            _validate_number(offset[1], "joint {} offset[1]".format(joint_id)),
            _validate_number(offset[2], "joint {} offset[2]".format(joint_id)),
        ]

        validated_joints.append(
            {
                "id": joint_id,
                "name": name,
                "parent_id": parent_id,
                "offset": offset_xyz,
            }
        )
        expected_id += 1

    frame_id = payload.get("frame_id", DEFAULT_FRAME_ID)
    if frame_id is None:
        frame_id = DEFAULT_FRAME_ID
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise BvhPayloadError("frame_id必须是非空字符串")

    return frame_id.strip(), validated_joints


def build_rigid_body_description(frame_id, joints):
    msg = RigidBodyDescription()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    for joint in joints:
        msg.ids.append(joint["id"])
        msg.names.append(joint["name"])
        msg.parent_ids.append(joint["parent_id"])
        msg.offsets.append(
            Point(
                x=joint["offset"][0],
                y=joint["offset"][1],
                z=joint["offset"][2],
            )
        )

    return msg


def handle_client(sock, address, publisher, max_payload_bytes):
    rospy.loginfo("收到BVH TCP连接：%s:%s", address[0], address[1])
    try:
        payload = read_length_prefixed_json(sock, max_payload_bytes)
        frame_id, joints = validate_bvh_payload(payload)
        msg = build_rigid_body_description(frame_id, joints)
        publisher.publish(msg)
        send_json(
            sock,
            {
                "ok": True,
                "message": "BVH skeleton received",
                "joint_count": len(joints),
            },
        )
        print_flush(BVH_READY_TEXT)
        print_flush("bvh 网络数据已解析")
        rospy.loginfo(
            "Published BVH segment description to %s: joints=%d",
            TOPIC_NAME,
            len(joints),
        )
        return True
    except Exception as exc:
        message = str(exc)
        rospy.logwarn("BVH TCP数据无效：%s", message)
        try:
            send_json(sock, {"ok": False, "message": message})
        except OSError:
            pass
        return False


def serve_until_first_valid_bvh(publisher, tcp_port, max_payload_bytes, accept_timeout_sec):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(accept_timeout_sec)
    try:
        server.bind((LISTEN_IP, tcp_port))
        server.listen(1)
    except OSError as exc:
        server.close()
        rospy.logerr("BVH TCP端口监听失败：%s:%d，%s", LISTEN_IP, tcp_port, exc)
        return False

    print_flush(
        "等待 Windows BVH 程序发送骨架数据，监听 {}:{}".format(
            LISTEN_IP, tcp_port
        )
    )
    rospy.loginfo("Waiting for BVH TCP payload on %s:%d", LISTEN_IP, tcp_port)

    try:
        while not rospy.is_shutdown():
            try:
                client, address = server.accept()
            except socket.timeout:
                continue
            except OSError as exc:
                rospy.logerr("BVH TCP accept失败：%s", exc)
                return False

            with client:
                client.settimeout(10.0)
                if handle_client(client, address, publisher, max_payload_bytes):
                    return True
    finally:
        server.close()

    return False


def load_params():
    return {
        "tcp_port": int(rospy.get_param("~tcp_port", 9764)),
        "max_payload_bytes": int(rospy.get_param("~max_payload_bytes", 8 * 1024 * 1024)),
        "accept_timeout_sec": float(rospy.get_param("~accept_timeout_sec", 0.5)),
    }


def main():
    rospy.init_node("xsense_bvh_desc_publisher")
    params = load_params()

    publisher = rospy.Publisher(
        TOPIC_NAME,
        RigidBodyDescription,
        queue_size=1,
        latch=True,
    )
    rospy.sleep(0.2)

    ok = serve_until_first_valid_bvh(
        publisher=publisher,
        tcp_port=params["tcp_port"],
        max_payload_bytes=params["max_payload_bytes"],
        accept_timeout_sec=params["accept_timeout_sec"],
    )
    if not ok:
        return 1

    rospy.loginfo("BVH TCP接收已完成，保持latched publisher存活")
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
