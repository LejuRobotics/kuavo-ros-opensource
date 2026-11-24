"""SG100 receiver discovery, board routing, replay and optional hand display."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

from .config import load_config
from .connection import (
    FrameDecoder,
    default_settings_path,
    list_receivers,
    load_settings,
    save_settings,
    scan_boards,
    select_port,
    serial_events,
    unique_usb_serial,
)
from .display import HandVisualizer
from .display import display_models as display_models
from .pose import InvalidPoseError
from .session import HandRouter


def frames(stream):
    decoder = FrameDecoder()
    while True:
        chunk = stream.read(4096)
        if not chunk:
            if getattr(stream, "is_open", False):
                continue
            break
        yield from decoder.feed(chunk)


def board_id(value):
    try:
        number = int(value, 16 if value.lower().startswith("0x") else 10)
    except ValueError as error:
        raise argparse.ArgumentTypeError("ID must be a decimal or hexadecimal integer") from error
    if not 0 <= number <= 0xFFFFFFFF:
        raise argparse.ArgumentTypeError("ID must be an unsigned 32-bit integer")
    return number


def _show_ports(ports):
    if not ports:
        print("No USB serial receivers found. Connect the receiver and try again.")
    for port in ports:
        print(
            f"{port['device']}  USB serial={port['serial_number'] or '-'}  "
            f"{port['description'] or ''}"
        )


def _configure(settings, path):
    ports = list_receivers()
    _show_ports(ports)
    default_port = settings.get("serial", ports[0]["device"] if len(ports) == 1 else "auto")
    entered_port = input(f"Serial port [{default_port}]: ").strip()
    selected = select_port(
        entered_port or default_port,
        None if entered_port else settings.get("usb_serial"),
    )
    default_hand = settings.get("hand", "both")
    hand = input(f"Hand mode left/right/both [{default_hand}]: ").strip() or default_hand
    if hand not in ("left", "right", "both"):
        raise ValueError("hand must be left, right, or both")
    print("Reading board IDs for 2 seconds; no commands are sent to the receiver.")
    found = scan_boards(selected["device"], settings.get("baud", 921600))
    print("Observed IDs:", ", ".join(str(row["board_id"]) for row in found["boards"]) or "none")
    result = dict(
        settings,
        serial=selected["device"],
        usb_serial=unique_usb_serial(selected, ports),
        hand=hand,
    )
    for side in ("left", "right") if hand == "both" else (hand,):
        default = settings.get(side + "_id")
        value = input(
            f"{side.capitalize()} board ID [{default if default is not None else ''}]: "
        ).strip()
        if not value and default is None:
            raise ValueError(f"{side} board ID is required")
        result[side + "_id"] = board_id(value) if value else default
    save_settings(path, result)
    print(
        f"Connection saved to {path}. Start the program without connection flags.", file=sys.stderr
    )


def _replay_events(path, rate):
    deadline = time.monotonic()
    with path.open("rb") as stream:
        for frame in frames(stream):
            if rate > 0:
                deadline += 1.0 / rate
                delay = deadline - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            yield "frame", frame


def _mapping(settings, hand):
    sides = ("left", "right") if hand == "both" else (hand,)
    result = {side: settings.get(side + "_id") for side in sides}
    if any(value is None for value in result.values()):
        return None
    if len(set(result.values())) != len(result):
        raise ValueError("left and right board IDs must be different")
    return result


def _output(result, both, as_json):
    if as_json:
        print(
            json.dumps(
                {key: result[key] for key in ("hand", "board_id", "seq")}
                | {"joint_angles": result["joint_angles"].tolist()}
            ),
            flush=True,
        )
    else:
        prefix = (
            ["J", result["hand"], result["board_id"], result["seq"]]
            if both
            else ["J", result["seq"]]
        )
        print(
            *prefix, *(format(float(value), ".17g") for value in result["joint_angles"]), flush=True
        )


def _output_gesture_poses(result):
    """Optional "P" line: pose-tracker five-finger poses for the same solved frame.

    P hand board_id seq p0x p0y p0z q0w q0x q0y q0z ... p4z q4z   (35 values)
    Positions are meters, quaternions WXYZ, finger order thumb/index/middle/ring/pinky.
    Emitted after the "J" line of the same frame; only with --emit-gesture-poses.
    Data comes from HandRouter.process() (already disambiguated); PoseTracker and
    SG100SDK are never re-run here.
    """
    positions = np.asarray(result["tracked_positions"], dtype=float).reshape(5, 3)
    quaternions = np.asarray(result["tracked_orientations"], dtype=float).reshape(5, 4)
    values = []
    for finger in range(5):
        values.extend(positions[finger])
        values.extend(quaternions[finger])
    if not all(math.isfinite(float(value)) for value in values):
        print(
            "Skipped non-finite gesture pose line",
            file=sys.stderr,
            flush=True,
        )
        return
    print(
        "P", result["hand"], result["board_id"], result["seq"],
        *(format(float(value), ".17g") for value in values),
        flush=True,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="Receiver port; auto selects a unique USB receiver")
    parser.add_argument("--usb-serial", help="Select a receiver by its USB serial number")
    parser.add_argument("--baud", type=int)
    parser.add_argument("--hand", "--mode", choices=("right", "left", "both"))
    parser.add_argument("--left-id", type=board_id)
    parser.add_argument("--right-id", type=board_id)
    parser.add_argument("--settings", type=Path, default=default_settings_path())
    parser.add_argument("--list-devices", action="store_true", help="List USB receivers and exit")
    parser.add_argument(
        "--scan", action="store_true", help="Passively scan receiver board IDs and exit"
    )
    parser.add_argument("--scan-seconds", type=float, default=2.0)
    parser.add_argument(
        "--configure", action="store_true", help="Interactively select and save connection"
    )
    parser.add_argument(
        "--save-connection", action="store_true", help="Save supplied connection flags and exit"
    )
    parser.add_argument("--no-reconnect", action="store_true")
    parser.add_argument("--ini", help="Single-hand compute INI; its side selects the default hand")
    parser.add_argument("--left-ini")
    parser.add_argument("--right-ini")
    parser.add_argument("--replay", type=Path)
    parser.add_argument(
        "--rate", type=float, default=100.0, help="Replay frames/sec; zero means unpaced"
    )
    parser.add_argument("--no-viz", action="store_true")
    parser.add_argument("--display-dir", type=Path)
    parser.add_argument("--max-frames", type=int, help="Stop after this many matching frames")
    parser.add_argument("--json", action="store_true", help="Emit JSON with hand and board ID")
    parser.add_argument(
        "--emit-gesture-poses",
        action="store_true",
        help="Also emit 'P' lines with pose-tracked five-finger poses per solved frame",
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.rate) or args.rate < 0:
        parser.error("--rate must be finite and nonnegative")
    if args.max_frames is not None and args.max_frames <= 0:
        parser.error("--max-frames must be greater than zero")
    if not math.isfinite(args.scan_seconds) or args.scan_seconds <= 0:
        parser.error("--scan-seconds must be finite and greater than zero")
    if args.baud is not None and args.baud <= 0:
        parser.error("--baud must be greater than zero")
    if args.list_devices:
        _show_ports(list_receivers())
        return
    try:
        settings = load_settings(args.settings)
        for key in ("serial", "hand", "left_id", "right_id", "baud"):
            value = getattr(args, key)
            if value is not None:
                settings[key] = value
        if args.serial is not None:
            settings.pop("usb_serial", None)
        if args.usb_serial is not None:
            settings["usb_serial"] = args.usb_serial
        baud = settings.get("baud", 921600)
        requested = settings.get("serial", "auto")
        usb_serial = settings.get("usb_serial")
        if args.configure:
            _configure(settings, args.settings)
            return
        if args.scan:
            if args.serial is not None or usb_serial:
                ports = [select_port(requested, usb_serial)]
            else:
                ports = list_receivers()
            if not ports:
                parser.exit(1, "No USB receivers found; use --serial for an explicit port.\n")
            for port in ports:
                result = scan_boards(port["device"], baud, args.scan_seconds)
                if args.json:
                    print(json.dumps(result), flush=True)
                else:
                    print(port["device"], "USB serial=" + str(port["serial_number"] or "-"))
                    for row in result["boards"]:
                        print(
                            f"  board ID {row['board_id']}  {row['hz']:.1f} Hz  {row['frames']} frames"
                        )
                    if not result["boards"]:
                        print(
                            "  No valid frames. Check receiver power, glove power and the serial port."
                        )
            return
        hand = settings.get("hand", "right")
        if args.ini and args.hand is None:
            hand = load_config(args.ini).get("sdk", {}).get("side", hand)
        if hand not in ("right", "left", "both"):
            raise ValueError("INI side must be left or right")
        settings["hand"] = hand
        mapping = _mapping(settings, hand)
        if mapping is None and (not args.replay or hand == "both"):
            parser.error(
                "Set the enabled hand's board ID with --left-id/--right-id, or use --configure. "
                "Discover IDs with --scan; INI names do not identify connected hardware."
            )
        if args.save_connection:
            selected = select_port(requested, usb_serial)
            settings.update(
                serial=selected["device"], usb_serial=unique_usb_serial(selected), baud=baud
            )
            save_settings(args.settings, settings)
            print(f"Connection saved to {args.settings}", file=sys.stderr)
            return
        if args.ini and hand == "both":
            parser.error("--ini is for a single hand; use --left-ini and --right-ini for both")
        paths = {"left": args.left_ini, "right": args.right_ini}
        if args.ini:
            paths[hand] = args.ini
        router = HandRouter(mapping, ini_paths=paths) if mapping else None
        viewer = None
        if not args.no_viz:
            viewer = HandVisualizer(
                ("left", "right") if hand == "both" else (hand,), args.display_dir
            )
            print("Meshcat " + viewer.viewer.url(), file=sys.stderr, flush=True)
        events = (
            _replay_events(args.replay, args.rate)
            if args.replay
            else serial_events(
                requested=requested,
                usb_serial=usb_serial,
                baud=baud,
                reconnect=not args.no_reconnect,
            )
        )
        count, seen_ids = 0, set()
        try:
            print(
                "Keep your palm open and fingers straight when starting or reconnecting.",
                file=sys.stderr,
                flush=True,
            )
            if mapping:
                print(
                    "Waiting for "
                    + ", ".join(f"{side} ID {value}" for side, value in mapping.items()),
                    file=sys.stderr,
                    flush=True,
                )
            for kind, payload in events:
                now = time.monotonic()
                if kind == "opened":
                    print(
                        f"Serial opened {payload['device']} at {baud}; waiting for target board data.",
                        file=sys.stderr,
                        flush=True,
                    )
                    if router:
                        router.reset()
                elif kind == "disconnected":
                    print("Receiver disconnected: " + payload["error"], file=sys.stderr, flush=True)
                    if router:
                        for side, value in router.board_ids.items():
                            print(
                                f"{side} ID {value} OFFLINE; receiver disconnected.",
                                file=sys.stderr,
                                flush=True,
                            )
                        router.reset()
                    if viewer:
                        for side in viewer.models:
                            viewer.hide(side)
                elif kind == "frame":
                    frame = payload
                    if frame["board_id"] not in seen_ids:
                        seen_ids.add(frame["board_id"])
                        print(f"Observed board ID {frame['board_id']}", file=sys.stderr, flush=True)
                    if router is None:
                        mapping = {hand: frame["board_id"]}
                        print(
                            f"Single-hand replay: using board ID {frame['board_id']} as {hand}.",
                            file=sys.stderr,
                            flush=True,
                        )
                        router = HandRouter(mapping, ini_paths=paths)
                    try:
                        result = router.process(frame, now)
                    except InvalidPoseError as error:
                        print(
                            f"Skipped invalid pose from board ID {frame['board_id']} "
                            f"seq {frame['seq']}: {error}",
                            file=sys.stderr,
                            flush=True,
                        )
                        result = None
                    if result is not None:
                        if result["became_online"]:
                            print(
                                f"{result['hand']} ID {result['board_id']} ONLINE",
                                file=sys.stderr,
                                flush=True,
                            )
                        if viewer:
                            viewer.update(result["hand"], result["joint_angles"])
                        _output(result, hand == "both", args.json)
                        if args.emit_gesture_poses:
                            _output_gesture_poses(result)
                        count += 1
                        if args.max_frames is not None and count >= args.max_frames:
                            break
                if router:
                    for side in router.expire(now):
                        print(
                            f"{side} ID {router.board_ids[side]} OFFLINE; waiting for data.",
                            file=sys.stderr,
                            flush=True,
                        )
                        if viewer:
                            viewer.hide(side)
        except KeyboardInterrupt:
            return
        finally:
            events.close()
            if viewer:
                viewer.close()
        if args.replay and count == 0:
            parser.exit(
                1,
                f"No valid joint frames for configured IDs {mapping}; observed IDs {sorted(seen_ids)}.\n",
            )
    except (OSError, ValueError, argparse.ArgumentTypeError) as error:
        parser.exit(1, f"Connection error: {error}\n")
    except EOFError:
        parser.exit(
            1,
            "Configuration needs an interactive terminal, or use connection flags with --save-connection.\n",
        )
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
