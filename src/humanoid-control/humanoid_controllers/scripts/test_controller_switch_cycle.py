#!/usr/bin/env python3
"""Measure repeated switching between AMP and depth locomotion controllers.

Example:
    rosrun humanoid_controllers test_controller_switch_cycle.py --count 200
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime, timezone

import rospy
from kuavo_msgs.srv import getControllerList, switchController


AMP_CONTROLLER = "amp_controller"
DEPTH_CONTROLLER = "depth_loco_controller"
VALID_CONTROLLERS = {AMP_CONTROLLER, DEPTH_CONTROLLER}


def wall_timestamp():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def get_current_controller(get_list):
    response = get_list()
    if not response.success:
        raise RuntimeError(response.message or "get_controller_list returned success=false")
    return response.current_controller


def write_row(writer, row):
    writer.writerow(row)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--count",
        type=int,
        default=200,
        help="number of one-way transitions (default: 200)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="seconds to wait after each transition (default: 0.5)",
    )
    parser.add_argument(
        "--output",
        default="",
        help="CSV path; defaults to controller_switch_cycle_<timestamp>.csv",
    )
    parser.add_argument(
        "--switch-service",
        default="/humanoid_controller/switch_controller",
        help="controller switch service name",
    )
    parser.add_argument(
        "--list-service",
        default="/humanoid_controller/get_controller_list",
        help="controller list service name",
    )
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("--count must be positive")
    if args.interval < 0:
        parser.error("--interval must be non-negative")
    return args


def main():
    args = parse_args()
    rospy.init_node("controller_switch_cycle_test", anonymous=True)

    output_path = args.output or (
        "controller_switch_cycle_"
        + datetime.now().strftime("%Y%m%d_%H%M%S")
        + ".csv"
    )
    output_dir = os.path.dirname(os.path.abspath(output_path))
    os.makedirs(output_dir, exist_ok=True)

    rospy.loginfo("Waiting for controller services")
    rospy.wait_for_service(args.list_service)
    rospy.wait_for_service(args.switch_service)
    get_list = rospy.ServiceProxy(args.list_service, getControllerList)
    switch = rospy.ServiceProxy(args.switch_service, switchController)

    fields = [
        "sequence",
        "request_wall_time",
        "response_wall_time",
        "request_ros_time",
        "response_ros_time",
        "source_controller",
        "target_controller",
        "service_success",
        "actual_controller_after",
        "verified",
        "elapsed_ms",
        "response_message",
        "error",
    ]

    rows = []
    success_durations = []
    success_count = 0
    failure_count = 0
    stopped_early = False

    with open(output_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()

        for sequence in range(1, args.count + 1):
            request_wall = wall_timestamp()
            request_ros = rospy.Time.now().to_sec()
            response_wall = request_wall
            response_ros = request_ros
            source = ""
            target = ""
            actual_after = ""
            service_success = False
            verified = False
            elapsed_ms = ""
            response_message = ""
            error = ""

            try:
                source = get_current_controller(get_list)
                if source not in VALID_CONTROLLERS:
                    raise RuntimeError(
                        "current controller is not one of {}: {}".format(
                            sorted(VALID_CONTROLLERS), source
                        )
                    )
                target = (
                    DEPTH_CONTROLLER if source == AMP_CONTROLLER else AMP_CONTROLLER
                )

                start = time.monotonic()
                response = switch(controller_name=target)
                elapsed_ms = round((time.monotonic() - start) * 1000.0, 3)
                response_wall = wall_timestamp()
                response_ros = rospy.Time.now().to_sec()
                service_success = bool(response.success)
                response_message = response.message

                try:
                    actual_after = get_current_controller(get_list)
                    verified = service_success and actual_after == target
                    if service_success and not verified:
                        error = "service succeeded but actual controller is '{}'".format(
                            actual_after
                        )
                except Exception as verify_error:
                    error = "post-switch verification failed: {}".format(verify_error)

                if verified:
                    success_count += 1
                    success_durations.append(float(elapsed_ms))
                else:
                    failure_count += 1
                    if not error:
                        error = response_message or "service returned success=false"

            except (rospy.ServiceException, RuntimeError, OSError) as exc:
                response_wall = wall_timestamp()
                response_ros = rospy.Time.now().to_sec()
                error = str(exc)
                failure_count += 1

                try:
                    actual_after = get_current_controller(get_list)
                except Exception:
                    actual_after = "unknown"

                if actual_after not in VALID_CONTROLLERS:
                    stopped_early = True

            row = {
                "sequence": sequence,
                "request_wall_time": request_wall,
                "response_wall_time": response_wall,
                "request_ros_time": request_ros,
                "response_ros_time": response_ros,
                "source_controller": source,
                "target_controller": target,
                "service_success": service_success,
                "actual_controller_after": actual_after,
                "verified": verified,
                "elapsed_ms": elapsed_ms,
                "response_message": response_message,
                "error": error,
            }
            write_row(writer, row)
            csv_file.flush()
            rows.append(row)

            rospy.loginfo(
                "[%d/%d] %s -> %s | service=%s verified=%s actual=%s elapsed=%sms | %s",
                sequence,
                args.count,
                source or "unknown",
                target or "unknown",
                service_success,
                verified,
                actual_after or "unknown",
                elapsed_ms or "n/a",
                response_message or error,
            )

            if stopped_early:
                rospy.logerr("Stopping: actual controller state is unknown")
                break
            if args.interval:
                rospy.sleep(args.interval)

    print("\nController switch test finished")
    print("  Requested transitions: {}".format(args.count))
    print("  Recorded transitions: {}".format(len(rows)))
    print("  Verified success: {}".format(success_count))
    print("  Failure: {}".format(failure_count))
    if success_durations:
        print("  Verified switch service time (ms): min={:.3f}, avg={:.3f}, max={:.3f}".format(
            min(success_durations),
            sum(success_durations) / len(success_durations),
            max(success_durations),
        ))
    print("  CSV: {}".format(os.path.abspath(output_path)))
    return 0 if success_count == len(rows) == args.count else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(2)

