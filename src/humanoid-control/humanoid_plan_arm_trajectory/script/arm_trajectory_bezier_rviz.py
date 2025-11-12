#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import json
import numpy as np
import threading

import rospy
from sensor_msgs.msg import JointState
from ocs2_msgs.msg import mpc_observation
from trajectory_msgs.msg import JointTrajectory
import tf2_ros
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from humanoid_plan_arm_trajectory.srv import (
    planArmTrajectoryBezierCurve,
    planArmTrajectoryBezierCurveRequest,
    ExecuteArmAction,
    ExecuteArmActionResponse,
)
from humanoid_plan_arm_trajectory.msg import (
    bezierCurveCubicPoint,
    jointBezierTrajectory,
)
from kuavo_msgs.srv import changeArmCtrlMode


def deg2rad_list(vals):
    return [math.radians(v) for v in vals]


class CombinedArmTrajectoryNode(object):
    INIT_ARM_POS = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def __init__(self):
        rospy.init_node('arm_trajectory_bezier_rviz')

        # ---- Params for bridge ----
        self.input_topic = rospy.get_param('~input_topic', '/kuavo_arm_traj')
        self.output_topic = rospy.get_param('~output_topic', '/joint_states')
        self.input_unit = rospy.get_param('~input_unit', 'deg')  # deg | rad

        self.urdf_joint_names = rospy.get_param('~urdf_joint_names', [
            'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint',
            'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint',
            'zarm_r1_joint', 'zarm_r2_joint', 'zarm_r3_joint', 'zarm_r4_joint',
            'zarm_r5_joint', 'zarm_r6_joint', 'zarm_r7_joint',
        ])

        self.publish_zero_on_idle = rospy.get_param('~publish_zero_on_idle', True)
        self.zero_publish_hz = float(rospy.get_param('~zero_publish_hz', 10.0))

        # Minimal params retained; markers/auto-trigger removed
        # Auto trigger (optional): trigger execute service once at startup
        self.auto_trigger = rospy.get_param('~auto_trigger_action', False)
        self.auto_action_name = rospy.get_param('~auto_action_name', 'welcome')
        self.mpc_topic = rospy.get_param('~mpc_topic', '/humanoid_mpc_observation')
        self.wait_mpc_before_trigger = rospy.get_param('~wait_mpc_before_trigger', False)
        self.wait_mpc_timeout = float(rospy.get_param('~wait_mpc_timeout', 5.0))
        self.auto_trigger_delay = float(rospy.get_param('~auto_trigger_delay', 1.0))

        # ---- Publishers/Subscribers for bridge ----
        self.js_pub = rospy.Publisher(self.output_topic, JointState, queue_size=10)
        self.js_sub = rospy.Subscriber(self.input_topic, JointState, self._bridge_cb, queue_size=10, tcp_nodelay=True)
        self.received_first = False
        self.zero_timer = None
        if self.publish_zero_on_idle:
            period = 1.0 / max(self.zero_publish_hz, 1e-3)
            self.zero_timer = rospy.Timer(rospy.Duration(period), self._publish_zero_if_needed)
            self._publish_zero_if_needed(None)

        # ---- Markers (end-effector path in RViz) ----
        self.ee_enable = rospy.get_param('~show_end_effector', True)
        self.base_frame = rospy.get_param('~base_frame', 'odom')
        self.left_link = rospy.get_param('~left_link', 'zarm_l7_end_effector')
        self.right_link = rospy.get_param('~right_link', 'zarm_r7_end_effector')
        self.marker_topic = rospy.get_param('~marker_topic', '/visualization_marker')
        self.marker_hz = float(rospy.get_param('~marker_hz', 30.0))
        self.show_path = rospy.get_param('~show_path', True)
        self.max_path_points = int(rospy.get_param('~max_path_points', 600))
        self.point_scale = float(rospy.get_param('~point_scale', 0.03))
        self.line_scale = float(rospy.get_param('~line_scale', 0.01))

        if self.ee_enable:
            self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10)
            self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self.tfl = tf2_ros.TransformListener(self.tfbuf)
            self.left_path = []
            self.right_path = []
            period = 1.0 / max(self.marker_hz, 1e-3)
            self.marker_timer = rospy.Timer(rospy.Duration(period), self._publish_markers)
        else:
            self.marker_pub = None
            self.marker_timer = None

        # ---- IO Topics ----
        self.joint_state = JointState()
        self.traj_sub = rospy.Subscriber('/bezier/arm_traj', JointTrajectory, self._traj_cb, queue_size=1, tcp_nodelay=True)
        self.kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
        self.mpc_obs_sub = rospy.Subscriber('/humanoid_mpc_observation', mpc_observation, self._mpc_obs_cb)

        # ---- Minimal planning state & service ----
        self.START_FRAME_TIME = 0.0
        self.END_FRAME_TIME = 10000.0
        self.x_shift = self.START_FRAME_TIME
        self.current_arm_joint_state = []

        # Action files path and execute service (minimal)
        self.action_files_path = rospy.get_param('~action_files_path')
        if not self.action_files_path:
            rospy.logerr("[arm_trajectory_bezier_rviz.py] ROS参数 ~action_files_path 未设置，请在launch文件中通过 <param name=\"action_files_path\" value=\"...\" /> 指定！")
            exit(1)
        self.execute_service = rospy.Service('/execute_arm_action', ExecuteArmAction, self._handle_execute_action)

        # Optionally auto trigger once
        if self.auto_trigger:
            threading.Thread(target=self._auto_trigger_once, daemon=True).start()

        rospy.loginfo('Combined node ready: %s -> %s (unit=%s, joints=%d)'
                      % (self.input_topic, self.output_topic, self.input_unit, len(self.urdf_joint_names)))

    # ---------------- Bridge callbacks ----------------
    def _bridge_cb(self, msg: JointState):
        if not self.received_first:
            self.received_first = True
            if self.zero_timer is not None:
                try:
                    self.zero_timer.shutdown()
                except Exception:
                    pass

        out = JointState()
        out.header.stamp = rospy.Time.now()
        out.name = list(self.urdf_joint_names)

        in_pos = list(msg.position[:14]) if msg.position else []
        in_vel = list(msg.velocity[:14]) if msg.velocity else []
        in_eff = list(msg.effort[:14]) if msg.effort else []

        if self.input_unit.lower() in ('deg', 'degree', 'degrees'):
            out.position = deg2rad_list(in_pos)
            out.velocity = deg2rad_list(in_vel)
        else:
            out.position = in_pos
            out.velocity = in_vel

        out.effort = in_eff if in_eff else [0.0] * len(out.position)

        N = len(self.urdf_joint_names)
        if len(out.position) < N:
            need = N - len(out.position)
            out.position += [0.0] * need
            out.velocity += [0.0] * need
            out.effort += [0.0] * need

        self.js_pub.publish(out)

    def _publish_zero_if_needed(self, _evt):
        if self.received_first:
            return
        N = len(self.urdf_joint_names)
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = list(self.urdf_joint_names)
        js.position = [0.0] * N
        js.velocity = [0.0] * N
        js.effort = [0.0] * N
        self.js_pub.publish(js)

    # ---------------- Marker helpers ----------------
    def _lookup_point(self, target_frame):
        try:
            tf = self.tfbuf.lookup_transform(self.base_frame, target_frame, rospy.Time(0), rospy.Duration(0.05))
            p = Point()
            p.x = tf.transform.translation.x
            p.y = tf.transform.translation.y
            p.z = tf.transform.translation.z
            return p
        except Exception:
            return None

    def _make_point_marker(self, frame_id, p, ns, mid, r, g, b):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = p
        m.pose.orientation.w = 1.0
        m.scale.x = self.point_scale
        m.scale.y = self.point_scale
        m.scale.z = self.point_scale
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0)
        return m

    def _make_path_marker(self, frame_id, pts, ns, mid, r, g, b):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = self.line_scale
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.8
        m.points = pts
        m.lifetime = rospy.Duration(0)
        return m

    def _publish_markers(self, _evt):
        if not self.marker_pub:
            return
        lp = self._lookup_point(self.left_link)
        rp = self._lookup_point(self.right_link)

        if lp is not None and not rospy.is_shutdown():
            try:
                self.marker_pub.publish(self._make_point_marker(self.base_frame, lp, 'ee_left', 1, 0.1, 0.9, 0.1))
                if self.show_path:
                    self.left_path.append(lp)
                    if len(self.left_path) > self.max_path_points:
                        self.left_path = self.left_path[-self.max_path_points:]
                    self.marker_pub.publish(self._make_path_marker(self.base_frame, self.left_path, 'ee_left_path', 2, 0.1, 0.6, 0.1))
            except rospy.ROSException:
                pass

        if rp is not None and not rospy.is_shutdown():
            try:
                self.marker_pub.publish(self._make_point_marker(self.base_frame, rp, 'ee_right', 3, 0.9, 0.1, 0.1))
                if self.show_path:
                    self.right_path.append(rp)
                    if len(self.right_path) > self.max_path_points:
                        self.right_path = self.right_path[-self.max_path_points:]
                    self.marker_pub.publish(self._make_path_marker(self.base_frame, self.right_path, 'ee_right_path', 4, 0.6, 0.1, 0.1))
            except rospy.ROSException:
                pass

    # ---------------- Auto trigger ----------------
    # ---------------- Auto trigger removed ----------------

    # ---------------- Bezier demo core ----------------
    def _mpc_obs_cb(self, msg: mpc_observation):
        # Keep latest arm joints from MPC observation if needed by up-stream users
        self.current_arm_joint_state = msg.state.value[24:]

    def _traj_cb(self, msg: JointTrajectory):
        if len(msg.points) == 0:
            return
        point = msg.points[0]
        # Prepare JointState in degrees for /kuavo_arm_traj (bridge converts to /joint_states in radians)
        self.joint_state.name = [
            "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
            "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
            "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
            "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
        ]
        self.joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
        if point.velocities:
            self.joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
        else:
            self.joint_state.velocity = [0.0] * 14
        self.joint_state.effort = [0] * 14
        # Publish immediately to /kuavo_arm_traj; bridge will forward to /joint_states
        try:
            self.kuavo_arm_traj_pub.publish(self.joint_state)
        except Exception as e:
            rospy.logerr(f"Failed to publish /kuavo_arm_traj: {e}")

    def _call_change_arm_ctrl_mode_service(self, arm_ctrl_mode: int) -> bool:
        result = True
        service_name = "humanoid_change_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            change_mode = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            change_mode(control_mode=arm_ctrl_mode)
            rospy.loginfo("Service call successful")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)
            result = False
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
            result = False
        finally:
            return result

    def _load_json_file(self, file_path):
        try:
            with open(file_path, "r") as f:
                return json.load(f)
        except IOError as e:
            rospy.logerr(f"Error reading file {file_path}: {e}")
            return None

    def _add_init_frame(self, frames):
        action_data = {}
        for frame in frames:
            servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
            for index, value in enumerate(servos):
                key = index + 1
                if key == 29:
                    break
                if key not in action_data:
                    action_data[key] = []
                    if keyframe != 0 and len(action_data[key]) == 0:
                        if key <= len(self.INIT_ARM_POS):
                            action_data[key].append([
                                [0, math.radians(self.INIT_ARM_POS[key-1])],
                                [0, math.radians(self.INIT_ARM_POS[key-1])],
                                [0, math.radians(self.INIT_ARM_POS[key-1])],
                            ])
                if value is not None:
                    CP = attribute[str(key)]["CP"]
                    left_CP, right_CP = CP
                    action_data[key].append([
                        [round(keyframe/100, 1), math.radians(value)],
                        [round((keyframe+left_CP[0])/100, 1), math.radians(value+left_CP[1])],
                        [round((keyframe+right_CP[0])/100, 1), math.radians(value+right_CP[1])],
                    ])
        return action_data

    def _filter_data(self, action_data):
        filtered_action_data = {}
        for key, frames in action_data.items():
            filtered_frames = []
            found_start = False
            skip_next = False
            for i in range(-1, len(frames)):
                frame = frames[i]
                if i == len(frames) - 1:
                    next_frame = frame
                else:
                    next_frame = frames[i+1]
                end_time = next_frame[0][0]
                if not found_start and end_time >= self.START_FRAME_TIME:
                    found_start = True
                    # Use current arm joint state as smooth start; if unavailable, fallback to 0.0
                    base_idx = key - 15
                    current = 0.0
                    if 0 <= base_idx < len(self.current_arm_joint_state):
                        current = self.current_arm_joint_state[base_idx]
                    p0 = np.array([0, current])
                    p3 = np.array([next_frame[0][0] - self.x_shift, next_frame[0][1]])

                    curve_length = np.linalg.norm(p3 - p0)
                    p1 = p0 + curve_length * 0.25 * np.array([1, 0])
                    p2 = p3 - curve_length * 0.25 * np.array([1, 0])

                    frame1 = [p0.tolist(), p0.tolist(), p1.tolist()]
                    next_frame[1] = p2.tolist()
                    filtered_frames.append(frame1)
                    skip_next = True

                if found_start:
                    if skip_next:
                        skip_next = False
                        continue
                    end_point = [round(frame[0][0] - self.x_shift, 1), round(frame[0][1], 1)]
                    left_control_point = [round(frame[1][0] - self.x_shift, 1), round(frame[1][1], 1)]
                    right_control_point = [round(frame[2][0] - self.x_shift, 1), round(frame[2][1], 1)]
                    filtered_frames.append([end_point, left_control_point, right_control_point])

            filtered_action_data[key] = filtered_frames
        return filtered_action_data

    # ---------------- Minimal planning service support ----------------
    def _create_bezier_request(self, action_data):
        req = planArmTrajectoryBezierCurveRequest()
        for _, value in action_data.items():
            msg = jointBezierTrajectory()
            for frame in value:
                point = bezierCurveCubicPoint()
                point.end_point, point.left_control_point, point.right_control_point = frame
                msg.bezier_curve_points.append(point)
            req.multi_joint_bezier_trajectory.append(msg)
        req.start_frame_time = self.START_FRAME_TIME
        req.end_frame_time = self.END_FRAME_TIME
        req.joint_names = [
            "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
            "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
            "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
            "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
        ]
        return req

    def _plan_arm_trajectory(self, req: planArmTrajectoryBezierCurveRequest) -> bool:
        service_name = '/bezier/plan_arm_trajectory'
        rospy.wait_for_service(service_name)
        try:
            plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
            res = plan_service(req)
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def _load_json_file(self, file_path):
        try:
            with open(file_path, "r") as f:
                return json.load(f)
        except IOError as e:
            rospy.logerr(f"Error reading file {file_path}: {e}")
            return None

    def _handle_execute_action(self, req):
        action_name = req.action_name
        file_path = f"{self.action_files_path}/{action_name}.tact"
        data = self._load_json_file(file_path)
        if not data:
            return ExecuteArmActionResponse(success=False, message=f"Action file {action_name} not found")

        # Initialize timeline
        first_value = data.get("first", 0)
        self.START_FRAME_TIME = round(first_value * 0.01, 2)
        self.x_shift = self.START_FRAME_TIME
        self.END_FRAME_TIME = data.get("finish", 0) * 0.01

        # Build request and trigger planner
        action_data = self._add_init_frame(data["frames"])
        filtered_data = self._filter_data(action_data)
        bezier_request = self._create_bezier_request(filtered_data)
        rospy.loginfo(f"Planning arm trajectory for action: {action_name}...")
        success = self._plan_arm_trajectory(bezier_request)
        if success:
            rospy.loginfo("Arm trajectory planned successfully")
            return ExecuteArmActionResponse(success=True, message="Action executed successfully")
        else:
            rospy.logerr("Failed to plan arm trajectory")
            return ExecuteArmActionResponse(success=False, message="Failed to execute action")

    def _auto_trigger_once(self):
        try:
            rospy.sleep(self.auto_trigger_delay)
            # wait for our own execute service to be available
            rospy.wait_for_service('/execute_arm_action', timeout=10.0)
            if self.wait_mpc_before_trigger:
                try:
                    rospy.wait_for_message(self.mpc_topic, rospy.AnyMsg, timeout=self.wait_mpc_timeout)
                except Exception:
                    rospy.logwarn('Auto-trigger: did not receive %s within %.1fs, proceeding.' % (self.mpc_topic, self.wait_mpc_timeout))
            exec_srv = rospy.ServiceProxy('/execute_arm_action', ExecuteArmAction)
            resp = exec_srv(action_name=self.auto_action_name)
            if resp.success:
                rospy.loginfo('Auto-triggered action: %s' % self.auto_action_name)
            else:
                rospy.logwarn('Auto-trigger action failed: %s' % resp.message)
        except Exception as e:
            rospy.logwarn('Auto-trigger error: %s' % e)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = CombinedArmTrajectoryNode()
    node.spin()
