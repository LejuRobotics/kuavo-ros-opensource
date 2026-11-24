"""SG100 串口实时读取回归测试（无需硬件）。"""

import contextlib
import io
import math
import struct
import unittest
from unittest import mock

from sg100_native import connection
from sg100_native import live
from sg100_native import uwb_decode


def make_packet(board_id, seq):
    packet = bytearray(uwb_decode.PACKET_LEN)
    struct.pack_into("<II", packet, 0, uwb_decode.HEADER, seq)
    struct.pack_into("<IfI", packet, 200, board_id, 25.0, 100)
    struct.pack_into("<H", packet, 230, uwb_decode.crc16_kermit(packet[:230]))
    return bytes(packet)


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        self.value += 0.01
        return self.value


class AlwaysDataStream:
    """模拟内核缓冲始终有数据，不能被无限读空拖住求解。"""

    def __init__(self):
        self.buffer = bytearray()
        self.closed = False
        self.read_sizes = []

    @property
    def in_waiting(self):
        self.buffer.extend(make_packet(2, len(self.read_sizes)))
        return len(self.buffer)

    def read(self, size):
        self.read_sizes.append(size)
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def close(self):
        self.closed = True


class BufferedStream:
    def __init__(self, chunks):
        self.buffer = bytearray(b"".join(chunks))
        self.closed = False
        self.read_sizes = []

    @property
    def in_waiting(self):
        return len(self.buffer)

    def read(self, size):
        self.read_sizes.append(size)
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def close(self):
        self.closed = True


class SerialEventsTest(unittest.TestCase):
    def setUp(self):
        self.port = {"device": "/dev/test", "serial_number": "test"}
        self.patches = [
            mock.patch.object(connection, "select_port", return_value=self.port),
            mock.patch.object(connection, "unique_usb_serial", return_value="test"),
        ]
        for patch in self.patches:
            patch.start()

    def tearDown(self):
        for patch in self.patches:
            patch.stop()

    def events(self, stream, **kwargs):
        return connection.serial_events(
            requested="/dev/test", reconnect=False, **kwargs
        )

    def test_passive_scan_counts_two_hand_ids_on_one_receiver_and_ignores_bad_crc(self):
        damaged = bytearray(make_packet(99, 1))
        damaged[-1] ^= 0x01
        stream = BufferedStream([b"noise", make_packet(12, 1), damaged,
                                 make_packet(2, 1), make_packet(12, 2)])
        with mock.patch.object(connection, "_open_serial", return_value=stream) as open_port, \
                mock.patch.object(connection.time, "monotonic", side_effect=Clock()):
            result = connection.scan_boards("/dev/test", 921600, 0.05)
        open_port.assert_called_once_with("/dev/test", 921600)
        self.assertEqual(result["device"], "/dev/test")
        self.assertEqual({row["board_id"]: row["frames"] for row in result["boards"]},
                         {12: 2, 2: 1})
        self.assertTrue(stream.closed)

    def test_continuous_data_returns_to_solver(self):
        stream = AlwaysDataStream()
        with mock.patch.object(connection, "_open_serial", return_value=stream):
            events = self.events(stream)
            self.assertEqual(next(events)[0], "opened")
            kind, frame = next(events)
        self.assertEqual(kind, "frame")
        self.assertEqual(frame["board_id"], 2)
        self.assertLessEqual(len(stream.read_sizes), 3)
        self.assertLessEqual(sum(stream.read_sizes), 12288)
        self.assertTrue(all(size <= 4096 for size in stream.read_sizes))
        events.close()
        self.assertTrue(stream.closed)

    def test_latest_frame_per_board(self):
        stream = BufferedStream([
            make_packet(2, 1), make_packet(12, 2), make_packet(2, 3),
        ])
        with mock.patch.object(connection, "_open_serial", return_value=stream):
            events = self.events(stream)
            next(events)
            frames = [next(events)[1] for _ in range(2)]
        events.close()
        self.assertEqual({frame["board_id"]: frame["seq"] for frame in frames}, {2: 3, 12: 2})
        self.assertTrue(stream.closed)

    def test_old_frames_do_not_accumulate_across_solver_yields(self):
        stream = BufferedStream([
            make_packet(2, 1), make_packet(2, 2),
            make_packet(2, 3), make_packet(2, 4),
        ])
        with mock.patch.object(connection, "_open_serial", return_value=stream), \
                mock.patch.object(connection.time, "monotonic", Clock()):
            events = self.events(stream, silence_timeout=0.3)
            next(events)
            kind, frame = next(events)
            self.assertEqual((kind, frame["seq"]), ("frame", 4))
            with self.assertRaises(ConnectionError):
                while True:
                    self.assertNotEqual(next(events)[0], "frame")
        self.assertTrue(stream.closed)

    def test_valid_frames_refresh_silence_timeout(self):
        first = make_packet(2, 1)
        second = make_packet(2, 2)
        stream = BufferedStream([first, second, b""])
        with mock.patch.object(connection, "_open_serial", return_value=stream), \
                mock.patch.object(connection.time, "monotonic", Clock()):
            events = self.events(stream, silence_timeout=0.15)
            next(events)
            self.assertEqual(next(events)[1]["seq"], 2)
            self.assertEqual(next(events)[0], "idle")
        events.close()
        self.assertTrue(stream.closed)

    def test_silence_disconnects_and_closes(self):
        stream = BufferedStream([b""])
        with mock.patch.object(connection, "_open_serial", return_value=stream), \
                mock.patch.object(connection.time, "monotonic", Clock()):
            events = self.events(stream, silence_timeout=0.1)
            next(events)
            with self.assertRaisesRegex(ConnectionError, "No valid SG100 frames"):
                while True:
                    next(events)
        self.assertTrue(stream.closed)


class SolverProtocolTest(unittest.TestCase):
    def test_real_solver_preserves_joint_and_pose_protocol(self):
        packet = bytearray(make_packet(2, 7))
        for finger, index in enumerate(uwb_decode.FINGER_PICK):
            offset = 8 + 32 * index
            struct.pack_into("<3f", packet, offset, 0.08, 0.02 * (finger - 2), -0.08)
            struct.pack_into("<4f", packet, offset + 12, 1.0, 0.0, 0.0, 0.0)
        result = live.HandRouter({"right": 2}).process(
            uwb_decode.parse_frame(packet), 1.0
        )
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            live._output(result, True, False)
            live._output_gesture_poses(result)
        joint, pose = [line.split() for line in stdout.getvalue().splitlines()]
        self.assertEqual(joint[:4], ["J", "right", "2", "7"])
        self.assertEqual(pose[:4], ["P", "right", "2", "7"])
        self.assertEqual(len(joint), 15)
        self.assertEqual(len(pose), 39)
        self.assertTrue(all(math.isfinite(float(value)) for value in joint[4:] + pose[4:]))


if __name__ == "__main__":
    unittest.main()
