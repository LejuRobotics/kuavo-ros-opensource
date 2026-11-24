"""Decode 232-byte UWB glove packets and produce solver input arrays.

The serial link uses 921600 baud, 8N1. Each packet contains six modules,
position and wxyz orientation, metadata, and a CRC16/KERMIT checksum.
"""

import struct

HEADER = 0xFFAD1234
PACKET_LEN = 232
HEADER_LE = struct.pack("<I", HEADER)  # 34 12 ad ff
# SDK五指顺序 thumb,index,middle,ring,pinky <- 线序索引
FINGER_PICK = [4, 5, 2, 0, 1]
FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]


def crc16_kermit(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def parse_module(buf, off):
    x, y, z, qw, qx, qy, qz, r = struct.unpack_from("<8f", buf, off)
    return dict(pos=(x, y, z), quat=(qw, qx, qy, qz), radius=r)


def parse_frame(buf):
    assert len(buf) == PACKET_LEN
    hdr, seq = struct.unpack_from("<II", buf, 0)
    mods = [parse_module(buf, 8 + i * 32) for i in range(6)]
    board_id, temp, solve_us = struct.unpack_from("<IfI", buf, 200)
    crc_stored = struct.unpack_from("<H", buf, 230)[0]
    crc_calc = crc16_kermit(buf[0:230])
    return dict(
        hdr=hdr,
        seq=seq,
        mods=mods,
        board_id=board_id,
        temp=temp,
        solve_us=solve_us,
        crc_ok=(crc_stored == crc_calc),
        crc_stored=crc_stored,
        crc_calc=crc_calc,
    )


def iter_frames(stream: bytes):
    i, n = 0, len(stream)
    while i + PACKET_LEN <= n:
        if stream[i : i + 4] == HEADER_LE:
            yield i, parse_frame(stream[i : i + PACKET_LEN])
            i += PACKET_LEN
        else:
            i += 1


def to_sdk_arrays(frame):
    """→ position[15], orientation[20] (SDK输入, thumb,index,middle,ring,pinky; wxyz)"""
    pos, orient = [], []
    for idx in FINGER_PICK:
        m = frame["mods"][idx]
        pos += list(m["pos"])
        orient += list(m["quat"])
    return pos, orient
