#!/usr/bin/env python3
"""Test controller switching through the simulated joystick /joy topic.

The mapping follows humanoid_controllers/launch/joy/sim.json:
  BUTTON_RL = buttons[2]
  AXIS_RIGHT_RT = axes[5]

Example:
  rosrun humanoid_controllers test_sim_joystick_controller_switch_cycle.py \
      --count 200 --interval 0.5 --output /tmp/sim_controller_switch.csv
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime, timezone

import rospy
from kuavo_msgs.srv import getControllerList, switchController
from sensor_msgs.msg import Joy


RL_BUTTON_INDEX = 2
RIGHT_RT_AXIS_INDEX = 5
AXES_COUNT = 8
# HumanoidAutoGaitJoyCommandNodeWithVel validates simulated/Xbox Joy messages
# as 11-button messages, even though sim.json only assigns the first 8 names.
BUTTONS_COUNT = 11
AMP_WILD_CONTROLLER = "amp_wild_controller"


def wall_timestamp():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def action_sequence(count):
    for index in range(count):
        yield "next" if index % 2 == 0 else "previous"


def build_joy_frame(action):
    """Build one press/release frame using the simulated joystick mapping."""
    if action not in {"next", "previous", "release"}:
        raise ValueError("unsupported joystick action: {}".format(action))

    message = Joy()
    message.axes = [0.0] * AXES_COUNT
    message.buttons = [0] * BUTTONS_COUNT
    if action in {"next", "previous"}:
        message.buttons[RL_BUTTON_INDEX] = 1
    if action == "previous":
        message.axes[RIGHT_RT_AXIS_INDEX] = -1.0
    return message


def get_controller_state(get_list):
    response = get_list()
    if not response.success:
        raise RuntimeError(response.message or "get_controller_list returned success=false")
    if response.current_index < 0 or response.current_index >= len(response.controller_names):
        raise RuntimeError(
            "invalid current controller index {} for {} controllers".format(
                response.current_index, len(response.controller_names)
            )
        )
    return response.current_controller, response.current_index, list(response.controller_names)


def expected_controller(state, action):
    current, index, names = state
    if not names:
        raise RuntimeError("controller list is empty")
    offset = 1 if action == "next" else -1
    return current, names[(index + offset) % len(names)]


def wait_for_controller(get_list, expected, timeout):
    deadline = time.monotonic() + timeout
    last_state = "unknown"
    while not rospy.is_shutdown() and time.monotonic() <= deadline:
        try:
            current, _, _ = get_controller_state(get_list)
            last_state = current
            if current == expected:
                return current
        except Exception as exc:
            last_state = "query error: {}".format(exc)
        rospy.sleep(0.02)
    return last_state


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=200, help="number of next/previous actions")
    parser.add_argument("--interval", type=float, default=3.0, help="seconds between actions")
    parser.add_argument("--switch-window-wait", type=float, default=3.0, help="seconds to wait after initial controller setup")
    parser.add_argument("--press-duration", type=float, default=0.08, help="seconds to hold the simulated button")
    parser.add_argument("--wait-timeout", type=float, default=3.0, help="seconds to wait for the target controller")
    parser.add_argument("--output", default="", help="CSV path")
    parser.add_argument("--joy-topic", default="/joy", help="simulated joystick topic")
    parser.add_argument("--switch-service", default="/humanoid_controller/switch_controller")
    parser.add_argument("--list-service", default="/humanoid_controller/get_controller_list")
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("--count must be positive")
    if args.interval < 0 or args.press_duration < 0 or args.wait_timeout <= 0 or args.switch_window_wait < 0:
        parser.error("interval, press-duration, and switch-window-wait must be non-negative; wait-timeout must be positive")
    return args


def main():
    args = parse_args()
    rospy.init_node("sim_joystick_controller_switch_cycle", anonymous=True)
    output_path = args.output or (
        "sim_controller_switch_cycle_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    )
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    rospy.wait_for_service(args.list_service)
    rospy.wait_for_service(args.switch_service)
    get_list = rospy.ServiceProxy(args.list_service, getControllerList)
    switch = rospy.ServiceProxy(args.switch_service, switchController)
    joy_pub = rospy.Publisher(args.joy_topic, Joy, queue_size=1)
    rospy.sleep(0.2)

    fields = [
        "sequence", "phase", "request_wall_time", "response_wall_time",
        "request_ros_time", "response_ros_time", "action", "source_controller",
        "target_controller", "actual_controller_after", "command_published",
        "verified", "elapsed_ms", "response_message", "error",
    ]
    rows = []
    success_count = 0
    failure_count = 0
    stopped_early = False

    with open(output_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()

        # Establish the requested starting point. If it is already active and the
        # service rejects a same-controller request, the verified state is enough.
        init_request_wall = wall_timestamp()
        init_request_ros = rospy.Time.now().to_sec()
        init_start = time.monotonic()
        init_success = False
        init_message = ""
        init_error = ""
        try:
            init_response = switch(controller_name=AMP_WILD_CONTROLLER)
            init_success = bool(init_response.success)
            init_message = init_response.message
        except rospy.ServiceException as exc:
            init_error = str(exc)
        init_actual = "unknown"
        try:
            init_actual, _, _ = get_controller_state(get_list)
        except Exception as exc:
            init_error = init_error or "initial state query failed: {}".format(exc)
        init_verified = init_actual == AMP_WILD_CONTROLLER
        if not init_verified:
            stopped_early = True
            init_error = init_error or "failed to establish amp_wild_controller"
        elif not init_success:
            rospy.logwarn("Initial switch service reported failure, but amp_wild_controller is active: %s", init_message)
        init_elapsed = round((time.monotonic() - init_start) * 1000.0, 3)
        init_row = {
            "sequence": 0, "phase": "initial", "request_wall_time": init_request_wall,
            "response_wall_time": wall_timestamp(), "request_ros_time": init_request_ros,
            "response_ros_time": rospy.Time.now().to_sec(), "action": "switch_service",
            "source_controller": "", "target_controller": AMP_WILD_CONTROLLER,
            "actual_controller_after": init_actual, "command_published": False,
            "verified": init_verified, "elapsed_ms": init_elapsed,
            "response_message": init_message, "error": init_error,
        }
        writer.writerow(init_row)
        csv_file.flush()
        rows.append(init_row)

        # Clear a possibly held RL button in the real simulator/node state and
        # let the controller's post-switch joystick lock expire before the first
        # rising edge. Otherwise the first test action can be consumed by the
        # controller-switch window or fail the rising-edge check.
        joy_pub.publish(build_joy_frame("release"))
        rospy.sleep(0.1)
        joy_pub.publish(build_joy_frame("release"))
        if args.switch_window_wait:
            rospy.sleep(args.switch_window_wait)

        for sequence, action in enumerate(action_sequence(args.count), start=1):
            if stopped_early:
                break
            request_wall = wall_timestamp()
            request_ros = rospy.Time.now().to_sec()
            source = ""
            target = ""
            actual_after = "unknown"
            command_published = False
            verified = False
            response_message = ""
            error = ""
            start = time.monotonic()
            try:
                state = get_controller_state(get_list)
                source, target = expected_controller(state, action)
                joy_pub.publish(build_joy_frame(action))
                command_published = True
                rospy.sleep(args.press_duration)
                joy_pub.publish(build_joy_frame("release"))
                actual_after = wait_for_controller(get_list, target, args.wait_timeout)
                verified = actual_after == target
                if not verified:
                    error = "expected '{}', actual '{}'".format(target, actual_after)
            except (rospy.ServiceException, RuntimeError, ValueError, OSError) as exc:
                error = str(exc)
                try:
                    actual_after, _, _ = get_controller_state(get_list)
                except Exception:
                    actual_after = "unknown"

            if verified:
                success_count += 1
            else:
                failure_count += 1
            if actual_after == "unknown":
                stopped_early = True

            row = {
                "sequence": sequence, "phase": "joystick", "request_wall_time": request_wall,
                "response_wall_time": wall_timestamp(), "request_ros_time": request_ros,
                "response_ros_time": rospy.Time.now().to_sec(), "action": action,
                "source_controller": source, "target_controller": target,
                "actual_controller_after": actual_after, "command_published": command_published,
                "verified": verified, "elapsed_ms": round((time.monotonic() - start) * 1000.0, 3),
                "response_message": response_message, "error": error,
            }
            writer.writerow(row)
            csv_file.flush()
            rows.append(row)
            rospy.loginfo(
                "[%d/%d] %s %s -> %s | actual=%s verified=%s elapsed=%sms | %s",
                sequence, args.count, action, source or "unknown", target or "unknown",
                actual_after, verified, row["elapsed_ms"], error or "ok",
            )
            if args.interval:
                rospy.sleep(args.interval)

    action_rows = [row for row in rows if row["phase"] == "joystick"]
    print("\nSim joystick controller switch test finished")
    print("  Requested actions: {}".format(args.count))
    print("  Recorded actions: {}".format(len(action_rows)))
    print("  Verified success: {}".format(success_count))
    print("  Failure: {}".format(failure_count))
    print("  CSV: {}".format(os.path.abspath(output_path)))
    return 0 if init_verified and len(action_rows) == args.count and failure_count == 0 else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(2)
