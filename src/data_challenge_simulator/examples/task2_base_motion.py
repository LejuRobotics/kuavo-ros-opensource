#!/usr/bin/env python3
"""Closed-loop Scene 2 chassis motion, isolated from box manipulation.

The script publishes body-frame ``vx``, ``vy`` and ``wz`` to ``/cmd_vel``.
The existing wheel controller and MuJoCo node own the four-wheel steering and
rolling allocation; task code must not command individual wheel joints.
"""

import argparse
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(orientation):
    sin_yaw = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y)
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z)
    return math.atan2(sin_yaw, cos_yaw)


def clamp(value, limit):
    return max(-limit, min(limit, value))


def planned_speed(error, gain, maximum, minimum, tolerance):
    if abs(error) <= tolerance:
        return 0.0
    command = clamp(gain * error, maximum)
    if abs(command) < minimum:
        return minimum if error > 0.0 else -minimum
    return command


class ChassisMotion:
    def __init__(
            self, linear_speed, angular_speed,
            minimum_linear_speed, minimum_angular_speed,
            position_tolerance, yaw_tolerance_deg, publish_rate=100.0):
        self.linear_speed = float(linear_speed)
        self.angular_speed = float(angular_speed)
        self.minimum_linear_speed = float(minimum_linear_speed)
        self.minimum_angular_speed = float(minimum_angular_speed)
        self.position_tolerance = float(position_tolerance)
        self.yaw_tolerance = math.radians(float(yaw_tolerance_deg))
        self.publish_rate = float(publish_rate)
        self._lock = threading.Lock()
        self._pose = None
        self._velocity = None
        self._publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._subscriber = rospy.Subscriber(
            "/ground_truth/state", Odometry, self._odom_callback,
            queue_size=1)

    def _odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        pose = (
            float(position.x),
            float(position.y),
            yaw_from_quaternion(orientation),
        )
        twist = message.twist.twist
        velocity = (
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.linear.z),
            float(twist.angular.x),
            float(twist.angular.y),
            float(twist.angular.z),
        )
        with self._lock:
            self._pose = pose
            self._velocity = velocity

    def pose(self):
        """Latest measured pose from ``/ground_truth/state``.

        Read off the ``imu`` site, which the model places at zero offset from
        ``base_link``, so this is ``base_link``'s world pose.  x/y are the
        base's position in the plane it drives in.
        """
        with self._lock:
            return self._pose

    def velocity(self):
        """Latest measured twist from ``/ground_truth/state``.

        The MuJoCo node's ``BodyVel``/``BodyGyro`` sensors are the simulator
        ground truth, not an odometry estimate, so no differentiation of the
        measured position is needed.  Note the sensor site is the torso ``imu``
        site, not ``base_link``; the twist carries a yaw-rotation lever arm and
        is not the wheel-contact velocity.
        """
        with self._lock:
            return self._velocity

    def wait_until_ready(self, timeout=30.0):
        deadline = time.time() + float(timeout)
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.pose() is not None and self._publisher.get_num_connections():
                return
            rate.sleep()
        raise RuntimeError(
            "Timed out waiting for /ground_truth/state and /cmd_vel subscriber")

    def publish(self, vx=0.0, vy=0.0, wz=0.0):
        command = Twist()
        command.linear.x = float(vx)
        command.linear.y = float(vy)
        command.angular.z = float(wz)
        self._publisher.publish(command)

    def stop(self, settle_time=0.6):
        deadline = time.time() + float(settle_time)
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown() and time.time() < deadline:
            self.publish()
            rate.sleep()

    def move_to_pose(self, target_x, target_y, target_yaw, timeout):
        start = time.time()
        last_log = 0.0
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            elapsed = time.time() - start
            if elapsed > timeout:
                self.stop()
                raise RuntimeError(
                    "Chassis target timed out after {:.1f}s".format(elapsed))

            current = self.pose()
            if current is None:
                rate.sleep()
                continue
            current_x, current_y, current_yaw = current
            error_world_x = target_x - current_x
            error_world_y = target_y - current_y
            cos_yaw = math.cos(current_yaw)
            sin_yaw = math.sin(current_yaw)
            error_body_x = (
                cos_yaw * error_world_x + sin_yaw * error_world_y)
            error_body_y = (
                -sin_yaw * error_world_x + cos_yaw * error_world_y)
            distance = math.hypot(error_world_x, error_world_y)
            yaw_error = normalize_angle(target_yaw - current_yaw)

            if (
                    distance <= self.position_tolerance
                    and abs(yaw_error) <= self.yaw_tolerance):
                self.stop()
                settled = self.pose()
                if settled is None:
                    rate.sleep()
                    continue
                settled_distance = math.hypot(
                    target_x - settled[0], target_y - settled[1])
                settled_yaw_error = normalize_angle(
                    target_yaw - settled[2])
                if (
                        settled_distance <= self.position_tolerance and
                        abs(settled_yaw_error) <= self.yaw_tolerance):
                    rospy.loginfo(
                        "Chassis target reached after zero-command settle: "
                        "pose=(%.3f, %.3f, %.1f deg)",
                        settled[0], settled[1], math.degrees(settled[2]))
                    return settled
                rospy.loginfo(
                    "Chassis coasted outside target tolerance after stop: "
                    "pose=(%.3f, %.3f, %.1f deg), resuming the same "
                    "closed-loop command",
                    settled[0], settled[1], math.degrees(settled[2]))
                rate.sleep()
                continue

            axis_tolerance = min(0.01, self.position_tolerance)
            vx = planned_speed(
                error_body_x, 1.0, self.linear_speed,
                self.minimum_linear_speed, axis_tolerance)
            vy = planned_speed(
                error_body_y, 1.0, self.linear_speed,
                self.minimum_linear_speed, axis_tolerance)
            wz = planned_speed(
                yaw_error, 1.5, self.angular_speed,
                self.minimum_angular_speed, self.yaw_tolerance)
            if distance <= self.position_tolerance:
                vx = 0.0
                vy = 0.0
            if abs(yaw_error) <= self.yaw_tolerance:
                wz = 0.0
            self.publish(vx, vy, wz)

            if elapsed - last_log >= 0.5:
                rospy.loginfo(
                    "Chassis moving: pose=(%.3f, %.3f, %.1f deg) "
                    "error_body=(%.3f, %.3f, %.1f deg) "
                    "cmd=(%.3f, %.3f, %.3f)",
                    current_x, current_y, math.degrees(current_yaw),
                    error_body_x, error_body_y, math.degrees(yaw_error),
                    vx, vy, wz)
                last_log = elapsed
            rate.sleep()

        raise rospy.ROSInterruptException()

    def move_open_loop(self, target_x, target_y):
        """Publish one timed body-frame command toward ``(target_x, target_y)``.

        Open loop: the measured pose is read once to size the command, the
        velocity is published for the computed duration, then zero commands
        are published for one cycle to cut it.  No arrival check, no timeout
        and no failure branch -- a short or long hop is reported to the caller
        through the returned plan, never raised.

        Used by the Task 2 initialization path only.  ``move_to_pose`` and its
        closed-loop tolerances are left as they are for the tasks sharing them.
        """
        current = self.pose()
        if current is None:
            raise RuntimeError("no measured chassis pose available")
        error_world_x = target_x - current[0]
        error_world_y = target_y - current[1]
        cos_yaw = math.cos(current[2])
        sin_yaw = math.sin(current[2])
        # World-frame displacement rotated into the body frame at command
        # time.  The chassis holds this heading for the whole hop, so the
        # command stays valid while the robot moves.
        error_body_x = cos_yaw * error_world_x + sin_yaw * error_world_y
        error_body_y = -sin_yaw * error_world_x + cos_yaw * error_world_y
        distance = math.hypot(error_world_x, error_world_y)
        plan = {
            "start": tuple(current),
            "target": (float(target_x), float(target_y)),
            "distance": distance,
            "command": (0.0, 0.0),
            "duration": 0.0,
            "cut_pose": tuple(current),
        }
        rospy.loginfo(
            "Open-loop move: target=(%.3f, %.3f) start=(%.3f, %.3f, %.1f deg) "
            "error_body=(%.3f, %.3f) distance=%.3f m",
            target_x, target_y, current[0], current[1],
            math.degrees(current[2]), error_body_x, error_body_y, distance)
        if distance <= 0.0:
            rospy.loginfo(
                "Open-loop move skipped: measured pose already equals the "
                "target, no command published")
            return plan

        vx = self.linear_speed * error_body_x / distance
        vy = self.linear_speed * error_body_y / distance
        duration = distance / self.linear_speed
        plan["command"] = (vx, vy)
        plan["duration"] = duration
        rospy.loginfo(
            "Open-loop command: vx=%.3f vy=%.3f for %.3f s",
            vx, vy, duration)
        deadline = time.time() + duration
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown() and time.time() < deadline:
            self.publish(vx, vy)
            rate.sleep()
        # Cut the command with one explicit zero cycle so the controller is
        # not left holding the last nonzero reference.
        self.publish()
        plan["cut_pose"] = tuple(self.pose())
        rospy.loginfo(
            "Open-loop cut-off: pose=(%.3f, %.3f, %.1f deg) after %.3f s",
            plan["cut_pose"][0], plan["cut_pose"][1],
            math.degrees(plan["cut_pose"][2]), duration)
        return plan

    def rotate_open_loop(self, angle_rad):
        """Publish one timed yaw command without an arrival decision.

        The requested relative angle only determines command direction and
        duration.  The measured final yaw is logged and returned; it is never
        compared with a tolerance and cannot raise an arrival timeout.
        """
        current = self.pose()
        if current is None:
            raise RuntimeError("no measured chassis pose available")
        angle = float(angle_rad)
        plan = {
            "start": tuple(current),
            "angle": angle,
            "command": 0.0,
            "duration": 0.0,
            "cut_pose": tuple(current),
        }
        rospy.loginfo(
            "Open-loop rotation: start_yaw=%.1f deg requested=%.1f deg",
            math.degrees(current[2]), math.degrees(angle))
        if angle == 0.0:
            rospy.loginfo(
                "Open-loop rotation skipped: requested angle is zero")
            return plan

        wz = self.angular_speed if angle > 0.0 else -self.angular_speed
        duration = abs(angle) / self.angular_speed
        plan["command"] = wz
        plan["duration"] = duration
        rospy.loginfo(
            "Open-loop rotation command: wz=%.3f for %.3f s",
            wz, duration)
        deadline = time.time() + duration
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown() and time.time() < deadline:
            self.publish(wz=wz)
            rate.sleep()
        self.publish()
        plan["cut_pose"] = tuple(self.pose())
        rospy.loginfo(
            "Open-loop rotation cut-off: yaw=%.1f deg after %.3f s",
            math.degrees(plan["cut_pose"][2]), duration)
        return plan

    def wait_until_stopped(self, speed_threshold, timeout):
        """Wait for the measured twist to fall below ``speed_threshold``.

        Reads the twist straight from ``/ground_truth/state``; the wheels are
        stopped by static friction well below the minimum commanded speed, so
        a zero command really does reach zero speed.  The timeout bounds a
        watchdog that should never fire -- running out of time is reported,
        not raised.
        """
        threshold = float(speed_threshold)
        deadline = time.time() + float(timeout)
        rate = rospy.Rate(self.publish_rate)
        last_log = 0.0
        start = time.time()
        while not rospy.is_shutdown() and time.time() < deadline:
            velocity = self.velocity()
            if velocity is not None:
                speed = math.hypot(velocity[0], velocity[1])
                yaw_rate = abs(velocity[5])
                if speed < threshold and yaw_rate < threshold:
                    rospy.loginfo(
                        "Open-loop settle: measured speed=%.4f m/s "
                        "yaw_rate=%.4f rad/s below %.4f after %.3f s",
                        speed, yaw_rate, threshold, time.time() - start)
                    return True
                elapsed = time.time() - start
                if elapsed - last_log >= 0.5:
                    rospy.loginfo(
                        "Open-loop settle: measured speed=%.4f m/s "
                        "yaw_rate=%.4f rad/s",
                        speed, yaw_rate)
                    last_log = elapsed
            rate.sleep()
        rospy.logwarn(
            "Open-loop settle: measured speed did not fall below %.4f within "
            "%.3f s; continuing without waiting", threshold,
            float(timeout))
        return False

    def translate_relative(self, forward_distance, timeout):
        current_x, current_y, current_yaw = self.pose()
        target_x = current_x + forward_distance * math.cos(current_yaw)
        target_y = current_y + forward_distance * math.sin(current_yaw)
        rospy.loginfo(
            "Relative translation: %.3f m -> target=(%.3f, %.3f)",
            forward_distance, target_x, target_y)
        return self.move_to_pose(
            target_x, target_y, current_yaw, timeout=timeout)

    def rotate_relative(self, angle_rad, timeout):
        current_x, current_y, current_yaw = self.pose()
        target_yaw = normalize_angle(current_yaw + angle_rad)
        rospy.loginfo(
            "Relative rotation: %.1f deg -> target_yaw=%.1f deg",
            math.degrees(angle_rad), math.degrees(target_yaw))
        return self.move_to_pose(
            current_x, current_y, target_yaw, timeout=timeout)


def positive_distance(value):
    value = float(value)
    if value < 0.0:
        raise argparse.ArgumentTypeError("distance must be non-negative")
    return value


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Validate Scene 2 base motion: back up, turn, then move forward. "
            "All distances are explicit; zero skips a stage."))
    parser.add_argument("--backward", type=positive_distance, default=0.0)
    parser.add_argument("--turn-deg", type=float, default=0.0)
    parser.add_argument("--forward", type=positive_distance, default=0.0)
    parser.add_argument("--linear-speed", type=positive_distance, default=0.12)
    parser.add_argument("--angular-speed", type=positive_distance, default=0.25)
    parser.add_argument("--minimum-linear-speed", type=positive_distance, default=0.06)
    parser.add_argument("--minimum-angular-speed", type=positive_distance, default=0.06)
    parser.add_argument("--position-tolerance", type=positive_distance, default=0.03)
    parser.add_argument("--yaw-tolerance-deg", type=positive_distance, default=3.0)
    parser.add_argument("--timeout", type=positive_distance, default=30.0)
    args = parser.parse_args()

    if args.backward == 0.0 and args.turn_deg == 0.0 and args.forward == 0.0:
        parser.error("provide at least one non-zero movement stage")
    if args.linear_speed == 0.0 or args.angular_speed == 0.0:
        parser.error("linear and angular speeds must be positive")
    if args.minimum_linear_speed > args.linear_speed:
        parser.error("minimum linear speed cannot exceed linear speed")
    if args.minimum_angular_speed > args.angular_speed:
        parser.error("minimum angular speed cannot exceed angular speed")

    rospy.init_node("task2_base_motion", anonymous=False)
    motion = ChassisMotion(
        linear_speed=args.linear_speed,
        angular_speed=args.angular_speed,
        minimum_linear_speed=args.minimum_linear_speed,
        minimum_angular_speed=args.minimum_angular_speed,
        position_tolerance=args.position_tolerance,
        yaw_tolerance_deg=args.yaw_tolerance_deg,
    )
    try:
        motion.wait_until_ready()
        initial = motion.pose()
        rospy.loginfo(
            "Initial chassis pose: (%.3f, %.3f, %.1f deg)",
            initial[0], initial[1], math.degrees(initial[2]))
        if args.backward:
            motion.translate_relative(-args.backward, timeout=args.timeout)
        if args.turn_deg:
            motion.rotate_relative(
                math.radians(args.turn_deg), timeout=args.timeout)
        if args.forward:
            motion.translate_relative(args.forward, timeout=args.timeout)
        final = motion.pose()
        rospy.loginfo(
            "Movement sequence complete: pose=(%.3f, %.3f, %.1f deg)",
            final[0], final[1], math.degrees(final[2]))
    finally:
        motion.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
