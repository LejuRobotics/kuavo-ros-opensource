"""Receiver discovery, persisted connection settings, and serial frame events."""

import json
import math
import os
import tempfile
import time
from pathlib import Path

import serial
from serial.tools import list_ports

from . import uwb_decode as protocol


def default_settings_path():
    """Return the per-user SG100 connection settings path."""
    root = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
    return root / "sg100" / "connection.json"


def _validate_settings(settings):
    if not isinstance(settings, dict):
        raise ValueError("Connection settings must be a JSON object")
    allowed = {"serial", "usb_serial", "hand", "left_id", "right_id", "baud"}
    unknown = set(settings) - allowed
    if unknown:
        raise ValueError("Unknown connection setting: " + ", ".join(sorted(map(str, unknown))))
    if "serial" in settings and not isinstance(settings["serial"], str):
        raise ValueError("serial must be a string")
    if settings.get("usb_serial") is not None and not isinstance(settings["usb_serial"], str):
        raise ValueError("usb_serial must be a string or null")
    if "hand" in settings and settings["hand"] not in ("left", "right", "both"):
        raise ValueError("hand must be left, right, or both")
    for key in ("left_id", "right_id"):
        value = settings.get(key)
        if value is not None and (type(value) is not int or not 0 <= value <= 0xFFFFFFFF):
            raise ValueError(f"{key} must be an unsigned 32-bit integer or null")
    if "baud" in settings and (type(settings["baud"]) is not int or settings["baud"] <= 0):
        raise ValueError("baud must be a positive integer")
    if settings.get("left_id") is not None and settings.get("left_id") == settings.get("right_id"):
        raise ValueError("left_id and right_id must be different")
    return dict(settings)


def load_settings(path):
    """Load and validate a connection settings file; absent files are empty."""
    try:
        with Path(path).open(encoding="utf-8") as stream:
            settings = json.load(stream)
    except FileNotFoundError:
        return {}
    return _validate_settings(settings)


def save_settings(path, settings):
    """Atomically replace validated settings, preserving an existing file on failure."""
    settings = _validate_settings(settings)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            prefix=".connection-",
            suffix=".json",
            dir=path.parent,
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            json.dump(settings, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _port_info(port):
    return {
        key: getattr(port, key, None)
        for key in ("device", "description", "serial_number", "vid", "pid", "location")
    }


def list_receivers():
    """List USB serial candidates in stable device order."""
    receivers = []
    for port in list_ports.comports():
        info = _port_info(port)
        device = info["device"] or ""
        usb_metadata = any(
            info[key] is not None for key in ("vid", "pid", "serial_number", "location")
        )
        if usb_metadata or device.startswith(("/dev/ttyUSB", "/dev/ttyACM", "/dev/serial/by-id/")):
            receivers.append(info)
    return sorted(receivers, key=lambda info: info["device"])


def unique_usb_serial(selected, receivers=None) -> str | None:
    """Return a receiver's USB serial only when it uniquely identifies a candidate."""
    identity = selected.get("serial_number")
    if not isinstance(identity, str) or not identity:
        return None
    if receivers is None:
        receivers = list_receivers()
    if sum(port.get("serial_number") == identity for port in receivers) == 1:
        return identity
    return None


def select_port(requested="auto", usb_serial=None):
    """Resolve a receiver by USB identity, explicit path, or a unique candidate."""
    receivers = list_receivers()
    if usb_serial is not None:
        matches = [port for port in receivers if port["serial_number"] == usb_serial]
        if len(matches) != 1:
            raise ValueError(
                f"USB receiver {usb_serial!r} has {len(matches)} matches; "
                "use --list-devices and --serial to select a receiver"
            )
        return matches[0]
    if requested != "auto":
        for port in receivers:
            if port["device"] == requested:
                return port
        return dict(
            device=requested,
            description=None,
            serial_number=None,
            vid=None,
            pid=None,
            location=None,
        )
    if len(receivers) != 1:
        raise ValueError(
            f"Found {len(receivers)} USB serial receivers; "
            "use --list-devices and --serial to select a receiver"
        )
    return receivers[0]


class FrameDecoder:
    """Incrementally decode CRC-valid packets and recover after damaged bytes."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while True:
            index = self.buffer.find(protocol.HEADER_LE)
            if index < 0:
                if len(self.buffer) > 3:
                    del self.buffer[:-3]
                break
            if index:
                del self.buffer[:index]
            if len(self.buffer) < protocol.PACKET_LEN:
                break
            frame = protocol.parse_frame(bytes(self.buffer[: protocol.PACKET_LEN]))
            if frame["crc_ok"]:
                del self.buffer[: protocol.PACKET_LEN]
                frames.append(frame)
            else:
                del self.buffer[0]
        return frames


def _open_serial(device, baud):
    # Set line states before opening so passive discovery does not pulse them.
    stream = serial.Serial(
        port=None,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
        exclusive=True,
    )
    try:
        stream.dtr = False
        stream.rts = False
        stream.port = device
        stream.open()
    except BaseException:
        stream.close()
        raise
    return stream


def scan_boards(device, baud=921600, duration=2.0):
    """Passively count valid packets per board during a bounded serial scan."""
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be finite and positive")
    stream = _open_serial(device, baud)
    decoder = FrameDecoder()
    counts = {}
    started = time.monotonic()
    try:
        while time.monotonic() - started < duration:
            size = max(1, min(4096, getattr(stream, "in_waiting", 0)))
            for frame in decoder.feed(stream.read(size)):
                board_id = frame["board_id"]
                counts[board_id] = counts.get(board_id, 0) + 1
    finally:
        stream.close()
    elapsed = time.monotonic() - started
    return {
        "device": device,
        "boards": [
            {"board_id": board_id, "frames": count, "hz": count / elapsed if elapsed else 0.0}
            for board_id, count in sorted(counts.items())
        ],
    }


# 每批最多读取 12 KiB，限制积压解码耗时。持续有数据时也必须返回求解，不能无限等待串口读空。
_SERIAL_READ_CHUNK = 4096
_MAX_SERIAL_READS_PER_BATCH = 3


def serial_events(
    requested="auto",
    usb_serial=None,
    baud=921600,
    reconnect=True,
    silence_timeout=2.0,
    retry_window=10.0,
    retry_interval=1.0,
):
    """Yield receiver lifecycle and valid frame events, closing at every exit."""
    for name, value in (("silence_timeout", silence_timeout), ("retry_interval", retry_interval)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not math.isfinite(retry_window) or retry_window < 0:
        raise ValueError("retry_window must be finite and nonnegative")
    retry_started = None
    identity = usb_serial
    while True:
        try:
            selected = select_port(requested, usb_serial=identity)
            stream = _open_serial(selected["device"], baud)
            try:
                decoder = FrameDecoder()
                last_valid = time.monotonic()
                if identity is None:
                    identity = unique_usb_serial(selected)
                yield "opened", selected
                while True:
                    latest = {}
                    for _ in range(_MAX_SERIAL_READS_PER_BATCH):
                        waiting = getattr(stream, "in_waiting", 0)
                        size = max(1, min(_SERIAL_READ_CHUNK, waiting))
                        chunk = stream.read(size)
                        # 解码已读积压，只保留本批每块板的最新帧；未到求解前不刷新断线计时。
                        for frame in decoder.feed(chunk):
                            latest[frame["board_id"]] = frame
                        if not waiting:
                            break
                    if latest:
                        for frame in latest.values():
                            yield "frame", frame
                        last_valid = time.monotonic()
                        retry_started = None
                    elif (
                        retry_started is not None
                        and time.monotonic() - retry_started >= retry_window
                    ):
                        raise ConnectionError("No valid frames within the reconnect window")
                    elif time.monotonic() - last_valid >= silence_timeout:
                        raise ConnectionError(
                            f"No valid SG100 frames received for {silence_timeout:g} seconds"
                        )
                    elif not chunk:
                        yield "idle", None
            finally:
                stream.close()
        except (OSError, serial.SerialException, ValueError) as error:
            if retry_started is None:
                retry_started = time.monotonic()
            yield "disconnected", {"error": str(error)}
            if not reconnect:
                raise
            remaining = retry_window - (time.monotonic() - retry_started)
            if remaining <= 0:
                raise ConnectionError(f"Unable to reconnect receiver: {error}") from error
            time.sleep(min(retry_interval, remaining))
            if time.monotonic() - retry_started >= retry_window:
                raise ConnectionError(f"Unable to reconnect receiver: {error}") from error
