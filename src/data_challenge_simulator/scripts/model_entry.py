#!/usr/bin/env python3
"""Launch one task simulator, initialize the scene, wait for an external model.

The initialization stage mirrors what the accepted task entries do before
their first task motion (see ``examples/task1.py``, ``task2.py``,
``task3.py``): randomize the layout, retreat the base, raise the arm to the
fixed ready posture, and return once to the measured random base.  Only after
that does this entry hand over arm control and publish
``/model_simulator/ready``. No task policy and no rosbag are started; the
external inference process owns the arm, hand and base action topics from that
point on.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import rospy
import rostopic
from kuavo_humanoid_sdk.interfaces.data_types import KuavoArmCtrlMode
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLES_DIR = os.path.join(PACKAGE_DIR, "examples")
for path in (PACKAGE_DIR, EXAMPLES_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

from utils.shutdown_guard import (
    ShutdownGuard,
    clear_dead_simulator_nodes,
    clear_simulator_nodes,
    clear_stale_processes,
)
from utils.model_master import ModelMaster
from utils.init_notification import notify_init


MODEL_LAUNCH_FILES = {
    1: "load_kuavo_mujoco_model1.launch",
    2: "load_kuavo_mujoco_model2.launch",
    3: "load_kuavo_mujoco_model3.launch",
}

SENSOR_TOPIC = "/sensors_data_raw"
ROUND_RETRY_DELAY = 1.0


class TopicTimeout(RuntimeError):
    """A required simulator topic did not publish before its deadline."""

    def __init__(self, topic):
        super().__init__("timed out waiting for {}".format(topic))
        self.topic = topic


OBJECT_TOPICS = {
    1: "/mujoco/cylinder_1/pose",
    2: "/mujoco/box_1/pose",
    3: "/mujoco/task3_hollow_cylinder/pose",
}

# All three model preprocessors use fixed joint trajectories.  IK belongs to
# the external policy process, not this simulator entry, so match the working
# Task 2 entry and do not make SDK IK startup a prerequisite for seed placement.
SDK_OPTIONS = {1: "Normal", 2: "Normal", 3: "Normal"}

# Retry the init notification without restarting the prepared simulator.
# Each RPC is bounded; the overall wait for the external peer is unlimited.
INIT_RETRY_INTERVAL = 1.0
SUCCESS_POLL_INTERVAL = 0.1
ROUND_READY_TIMEOUT = 120.0
CONTROLLER_READY_TIMEOUT = 45.0

# Upstream's ``head_init`` for simulation, taken verbatim from
# ``configs/deploy/kuavo_env.yaml`` in kuavo_data_challenge ("Robot head
# initial position.  Use this value for simulation so that the observation is
# consistent.").  It is not zero on purpose: the model was trained against
# this head pose, so resetting to it is what keeps the head camera view
# comparable to training.
UPSTREAM_HEAD_INIT = (0.0, 0.209)


class ResetHandoff(object):
    """Keep ROS services and start waiters alive across simulator replacements."""

    def __init__(self):
        self._lock = threading.Lock()
        self._response_sent = threading.Event()
        self._seed = None
        self._condition = threading.Condition(self._lock)
        self._protocol = None
        self._generation = 0
        self._ready = False
        self._closed = False
        self._services = []

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def serve(self):
        if not self._services:
            self._services = [
                rospy.Service("/simulator/reset", Trigger, self.reset),
                rospy.Service("/simulator/start", Trigger, self.start)]
            rospy.on_shutdown(self.close)

    def close(self):
        with self._condition:
            self._closed = True
            self._ready = False
            self._condition.notify_all()
        for service in self._services:
            service.shutdown("model evaluation session ended")
        self._services = []

    def attach(self, protocol):
        with self._condition:
            self._generation += 1
            protocol.generation = self._generation
            self._protocol = protocol
            self._ready = False
            self._condition.notify_all()

    def detach(self, protocol):
        with self._condition:
            if self._protocol is protocol:
                self._protocol = None
                self._ready = False
                self._condition.notify_all()

    def is_current(self, protocol, ready=False):
        with self._condition:
            return (not self._closed and self._protocol is protocol
                    and protocol.generation == self._generation
                    and (not ready or self._ready))

    def mark_ready(self, protocol):
        with self._condition:
            if (self._closed or self._protocol is not protocol
                    or protocol.generation != self._generation
                    or protocol.relaunch.is_set()):
                return False
            self._ready = True
            self._condition.notify_all()
            return True

    def reset(self, request):
        with self._condition:
            protocol = self._protocol
            closed = self._closed
        if closed or protocol is None:
            return TriggerResponse(success=False, message="simulator is not available")
        return protocol._reset(request)

    def start(self, request):
        deadline = time.monotonic() + ROUND_READY_TIMEOUT
        while True:
            with self._condition:
                if self._closed or rospy.is_shutdown():
                    return TriggerResponse(success=False, message="evaluation is stopping")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return TriggerResponse(
                        success=False, message="scene is still initializing after {:.0f}s".format(
                            ROUND_READY_TIMEOUT))
                protocol = self._protocol
                if (not self._ready or protocol is None
                        or protocol.generation != self._generation
                        or protocol.relaunch.is_set()):
                    self._condition.wait(timeout=min(.1, remaining))
                    continue
            # Never hold the handoff lock while waiting for physical observations.
            # The round rechecks admission under its lifecycle lock, since reset
            # can retire it after the condition above has released the lock.
            response = protocol._start(request)
            if response is not None:
                return response

    def prepare(self, seed):
        with self._lock:
            self._seed = int(seed)
            self._response_sent.clear()
            # A replacement may reuse the seed; identity must still change.
            self._generation += 1
            self._ready = False
            self._condition.notify_all()

    def mark_response_sent(self):
        self._response_sent.set()

    def retarget(self, seed):
        """Move an open reset handoff to a replacement simulator attempt.

        The harness has already returned from reset and is waiting for init,
        so an internal retry must preserve ``_response_sent`` while changing
        only the seed that will eventually satisfy that wait.
        """
        with self._lock:
            self._seed = int(seed)

    def pending_seed(self):
        """Return the seed of an answered reset, or ``None`` if none exists."""
        if not self._response_sent.is_set():
            return None
        with self._lock:
            return self._seed

    def wait_for(self, seed, timeout):
        if not self._response_sent.wait(timeout=timeout):
            return False
        with self._lock:
            return self._seed == int(seed)


def request_shutdown(_signum, _frame):
    """Turn SIGTERM from the wrapper's stop command into normal cleanup."""
    raise KeyboardInterrupt


def ensure_clean_graph(task_id, timeout=10.0):
    """Kill any simulator left over from an interrupted round, then proceed.

    An interrupted round leaves its nodelets registered with the master.  They
    used to make the next round refuse to start; clearing them here means a
    round always gets a clean graph, exactly as helperfunc.py does.  The ROS
    master also outlives each roslaunch, so remove the old qpos-ready parameter
    only after its writer is gone.  Otherwise the next controller consumes the
    previous round's base spawn before the new MuJoCo node can publish its own.
    """
    clear_stale_processes(task_id)
    clear_simulator_nodes(timeout=timeout)
    try:
        rospy.delete_param("/robot_init_state_param")
        print("[INFO] Cleared stale simulator initial state", flush=True)
    except KeyError:
        pass


def wait_for_topic(topic, timeout):
    deadline = time.time() + timeout
    while not rospy.is_shutdown() and time.time() < deadline:
        message_class, _, _ = rostopic.get_topic_class(topic)
        if message_class is not None:
            try:
                rospy.wait_for_message(topic, message_class, timeout=1.0)
                return
            except rospy.ROSException:
                pass
        time.sleep(0.2)
    raise TopicTimeout(topic)


def initial_base_for(task_id, seed):
    """Return the launch-time random base translation for one round.

    Same per-task mechanism the collection entry uses in
    ``examples/helperfunc.py``: every task reads its own saved layout
    catalogue by seed.
    """
    if task_id == 1:
        from utils.task1_v2_randomization import Task1V2RandomizationPlanner
        base = Task1V2RandomizationPlanner().plan(seed).initial_base
        return float(base[0]), float(base[1])
    if task_id == 2:
        from helperfunc import task2_initial_base_translation
        return task2_initial_base_translation(seed)
    from helperfunc import task3_initial_base_translation
    return task3_initial_base_translation(seed)


def initialize_task1(robot, robot_state, seed, trajectory):
    from task1_v2_initialize import run_initialization

    return run_initialization(
        robot, robot_state, seed, trajectory=trajectory)


def load_config(path):
    with open(path, "r", encoding="utf-8") as stream:
        config = json.load(stream)
    if config.get("schema_version") != 1:
        raise RuntimeError("unsupported config schema: {}".format(path))
    return config


def initialize_task2(robot, robot_state, seed, trajectory):
    """Initialize Task 2 from the same seeded scene as task/collect."""
    from task2_initialize import (
        DEFAULT_CONFIG, DEFAULT_SCENE, make_chassis, run_initialization)
    from utils.object_randomizer import ObjectRandomizer
    from utils.task2_pick_planner import Task2PickPlanner
    from utils.task2_randomization import Task2RandomizationPlanner

    # Same placement as task2.main(); reproduced here because that helper
    # calls rospy.init_node itself and this entry is already a node.  The
    # base translation comes from the same catalogue entry through
    # initial_base_for().
    layout = Task2RandomizationPlanner().plan(seed)
    randomizer = ObjectRandomizer(timeout=30.0)
    for box in layout.boxes:
        result = randomizer.set_object_position(
            box.name,
            position={"x": box.position[0], "y": box.position[1],
                      "z": box.position[2]},
            orientation={"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
        if not result["success"]:
            raise RuntimeError(
                "failed to place {}: {}".format(box.name, result["message"]))
    rospy.sleep(1.0)
    print("TASK2 model seed {} placed generated boxes".format(seed))

    config = load_config(DEFAULT_CONFIG)
    pick_config = load_config(
        os.path.join(PACKAGE_DIR, "config/task2_pick.json"))
    chassis = make_chassis(
        SimpleNamespace(
            linear_speed=None,
            minimum_linear_speed=0.06,
            position_tolerance=0.03),
        config)
    try:
        return run_initialization(
            robot, robot_state, chassis, config, DEFAULT_SCENE,
            Task2PickPlanner(ik_config=pick_config["ik"]),
            pick_config["grasp"]["outward_angle_deg"],
            trajectory=trajectory)
    finally:
        from utils.model_base_motion import close_chassis
        close_chassis(chassis)


def initialize_task3(robot, robot_state, seed, trajectory):
    from task3_model_initialize import (
        DEFAULT_CONFIG, run_initialization,
        wait_for_controller_initialization)
    from utils.object_randomizer import ObjectRandomizer
    from utils.task3_randomization import Task3RandomizationPlanner
    from task2_base_motion import ChassisMotion

    config = load_config(DEFAULT_CONFIG)

    # Same placement as task3.apply_plan(); reproduced here because that
    # helper calls rospy.init_node itself and this entry is already a node.
    plan = Task3RandomizationPlanner().plan(seed)
    randomizer = ObjectRandomizer(timeout=30.0)
    for ring in plan.rings:
        result = randomizer.set_object_position(
            ring.name,
            position={"x": ring.position[0], "y": ring.position[1],
                      "z": ring.position[2]},
            orientation={"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
        if not result["success"]:
            raise RuntimeError(
                "failed to place {}: {}".format(ring.name, result["message"]))
    rospy.sleep(1.0)
    print("TASK3 model seed {} placed rings from layout {}".format(
        seed, plan.catalogue_seed))

    wait_for_controller_initialization()
    chassis = ChassisMotion(
        linear_speed=0.20, angular_speed=0.20, minimum_linear_speed=0.08,
        minimum_angular_speed=0.06, position_tolerance=0.03,
        yaw_tolerance_deg=3.0)
    chassis.wait_until_ready(timeout=30.0)
    try:
        return run_initialization(
            robot_state, chassis, config, trajectory)
    finally:
        from utils.model_base_motion import close_chassis
        close_chassis(chassis)


INITIALIZERS = {1: initialize_task1, 2: initialize_task2, 3: initialize_task3}


class EvaluationProtocol:
    """Expose the upstream multi-episode simulator handshake.

    The harness in ``sim_auto_test.py`` expects three services and one topic
    from the simulator side:

    * ``/simulator/reset`` -- re-randomize the scene and report when done;
    * ``/simulator/start`` -- episode is live, start judging success;
    * ``/simulator/init``  -- called back once the environment is ready;
    * ``/simulator/success`` -- ``True`` ends the episode.

    Two ordering details of that harness shape this class:

    the harness clears its ``init_evt`` *after* a reset returns, so the
    ``/simulator/init`` notification for a fresh round cannot be sent from
    inside the reset handler -- it would be wiped before the harness reaches
    its wait loop.  The reset therefore only records the next seed and asks
    :func:`main` to relaunch.  The fresh protocol created after that relaunch
    performs initialization and sends the notification.  It must never
    initialize in a background thread owned by the old protocol: main is
    tearing that protocol's simulator down at exactly that point.  Resetting
    also never re-places the scene in place: the base spawn is a launch
    argument, and Task 1's saved layouts pair each object placement with its
    own spawn point, so a reset stops the node and main starts a fresh one with
    the new round's spawn.

    A round also owns its own SDK objects.  ``KuavoSDK.Init`` reassigns the
    SDK's module-level core, so :func:`main` builds a fresh ``KuavoRobot`` and
    ``KuavoRobotState`` after each Init rather than caching ones bound to the
    simulator that was just torn down.
    """

    def __init__(
            self, task_id, robot, robot_state, seed, ready_publisher,
            command_accept_publisher, reset_handoff=None, score_store=None):
        self.task_id = task_id
        self.robot = robot
        self.robot_state = robot_state
        self.seed = seed
        self.ready_publisher = ready_publisher
        self.command_accept_publisher = command_accept_publisher
        self.success_publisher = rospy.Publisher(
            "/simulator/success", Bool, queue_size=1)
        self.lock = threading.Lock()
        # Serialize score startup, reset and teardown without holding the
        # readiness/state lock during ROS observation waits.
        self._lifecycle_lock = threading.Lock()
        self.episode_started = False
        # Reset bookkeeping.  One reset runs at a time; main() watches
        # ``relaunch``/``reset_seed`` and restarts the node with that round's
        # spawn point.
        self.round_seed = seed
        self.reset_seed = seed
        self.relaunch = threading.Event()
        self.initialized_evt = threading.Event()
        # /simulator/reset is served by the old protocol, while
        # /simulator/init is sent by the fresh one after roslaunch is rebuilt.
        # The process-wide handoff routes start to the current ready generation,
        # including replacements that reuse the same scene seed.
        self.reset_handoff = (
            reset_handoff if reset_handoff is not None else ResetHandoff())
        self.score_store = score_store
        self.score_file = None
        self.score_finish_reason = None
        self.round_complete = False
        self.evaluation_started = False
        self.infrastructure_failure = None
        self.invalid_round = False
        self._reset_running = False
        self._init_wait_logged = False
        self.observer = None
        self.observer_thread = None
        self.reset_handoff.attach(self)

    # -- initialization and the /simulator/init callback ------------------
    def _initialize(self):
        # A fresh observer per round: the scorers' "ever grasped"/"ever
        # placed" state is sticky, so carrying it over would score the new
        # round from the old round's poses.
        self._drop_observer()
        # A relaunch must not expose the previous round's True while this round
        # is still initializing.
        self.ready_publisher.publish(Bool(data=False))
        self.command_accept_publisher.publish(Bool(data=False))
        self._wait_for_controller_ready()

        # Match the accepted task entries: one publisher is live from the
        # measured arm pose before any scene or chassis work, and remains live
        # throughout retreat -> arm ready -> return.
        from utils.trajectory_controller import TrajectoryController
        # Do not mistake KuavoRobotState's correctly-sized all-zero startup
        # cache for a measured pose.  This controller exists only for fixed
        # initialization and is stopped before external model handover.
        from utils.arm_state import wait_for_first_arm_state
        initial_arm = wait_for_first_arm_state()
        if len(initial_arm) != 14:
            raise RuntimeError(
                "expected 14 measured arm joints, got {}".format(
                    len(initial_arm)))
        initialization_trajectory = TrajectoryController(
            self.robot, initial_positions=initial_arm)
        try:
            INITIALIZERS[self.task_id](
                self.robot, self.robot_state, self.seed,
                initialization_trajectory)
        finally:
            # Stop first: every publisher call automatically requests
            # ExternalControl, so switching to ArmFixed while this thread is
            # alive would immediately reopen the input.
            initialization_trajectory.stop()

        # A reset received during initialization retires this whole simulator.
        # Finishing its fixed trajectory must not reopen readiness or commands.
        if self.relaunch.is_set() or not self.reset_handoff.is_current(self):
            return

        # control_arm_joint_positions() switches to ExternalControl as a side
        # effect. Close that input before handing control to the model.
        self._close_external_arm_input()

        # Completion means the fixed task initialization is done. Episode-time
        # navigation, including Task 3 ring docking, belongs to the model.
        # Open the arm/hand command path before ready: a model that uses ready
        # as its only barrier must never observe True while initialization
        # still owns either control path.
        handed_over = self._hand_over_arm_control()
        with self.lock:
            if self.relaunch.is_set() or not self.reset_handoff.is_current(self):
                return
            self.command_accept_publisher.publish(Bool(data=bool(handed_over)))
            if not handed_over:
                raise RuntimeError(
                    "external arm-control handoff failed; model remains not ready")
            self.ready_publisher.publish(Bool(data=True))
            self.publish_success(False)
            self.round_complete = False
            self.initialized_evt.set()
            self.reset_handoff.mark_ready(self)
        # main() retries notification while also monitoring reset and roslaunch.

    @staticmethod
    def _wait_for_controller_ready():
        """Wait until the fixed initialization commands can be consumed."""
        services = (
            "/wheel_arm_change_arm_ctrl_mode",
            "/humanoid_get_arm_ctrl_mode",
        )
        for service in services:
            rospy.wait_for_service(service, timeout=CONTROLLER_READY_TIMEOUT)
        rospy.loginfo(
            "model entry: controller ready; starting fixed initialization")

    def _close_external_arm_input(self):
        """Hold the initialized arm pose until model handover."""
        try:
            closed = self.robot.set_fixed_arm_mode()
        except Exception as error:
            raise RuntimeError(
                "could not close external arm input after initialization: {}"
                .format(error))
        if not closed:
            raise RuntimeError(
                "controller rejected ArmFixed after initialization")
        rospy.loginfo(
            "model entry: external arm input closed at initialized ready pose")

    def publish_success(self, value):
        """Publish ``/simulator/success``; ``True`` ends the harness episode."""
        self.success_publisher.publish(Bool(data=bool(value)))

    def _hand_over_arm_control(self):
        """Hand the arm to the external model, as the upstream reset expects.

        Upstream ``KuavoBaseRosEnv.reset()`` (kuavo_deploy, sim_auto_test.py's
        ``run_single_episode`` calls it at the start of every episode) does two
        things before it calls ``/simulator/start``: it switches the arm to
        external control and it resets the head to ``head_init``.  Both are
        requests the simulator side has to honour -- without them the model
        believes it owns the arm while the simulator is still driving it, and
        the episode never gets going.

        ``KuavoSimEnv`` overrides ``_reset_eef`` with an empty body, so the
        gripper part of that upstream reset is a no-op in simulation and there
        is nothing to mirror here.

        ``[0, 0.209]`` is upstream's ``head_init`` for simulation (see
        ``configs/deploy/kuavo_env.yaml``, "use this value for simulation so
        that the observation is consistent"); it is deliberately not zero.
        """
        # The two services this needs are advertised by the controller, not by
        # the launch, so they may not be up yet at this point -- the same
        # race that makes /humanoid_get_arm_ctrl_mode time out during SDK
        # construction.  Probe before calling so the log says whether a
        # failure was "service absent" or "service present but refused".
        for service in ("/wheel_arm_change_arm_ctrl_mode",
                        "/humanoid_get_arm_ctrl_mode"):
            try:
                rospy.wait_for_service(service, timeout=5.0)
            except Exception as error:
                rospy.logwarn(
                    "model entry: %s is not advertised yet (%s); the "
                    "handover below will likely fail", service, error)
            else:
                rospy.loginfo(
                    "model entry: %s is available, handover can proceed",
                    service)

        started = time.time()
        handed_over = False
        try:
            handed_over = self.robot.set_external_control_arm_mode()
        except Exception as error:
            rospy.logwarn(
                "model entry: could not hand the arm to external control "
                "after %.3fs (%s); the episode may not start",
                time.time() - started, error)
        else:
            rospy.loginfo(
                "model entry: arm control handed to the external model in "
                "%.3fs (set_external_control_arm_mode -> %s)",
                time.time() - started, handed_over)
            # The return value only says the service answered.  Read the mode
            # back so the log distinguishes "the switch took effect" from
            # "the call was accepted but nothing changed" -- that difference
            # is the whole point of this handover.
            self._log_arm_control_mode()
        started = time.time()
        try:
            head_ok = self.robot.control_head(*UPSTREAM_HEAD_INIT)
        except Exception as error:
            rospy.logwarn(
                "model entry: could not reset the head to %s after %.3fs "
                "(%s)", list(UPSTREAM_HEAD_INIT), time.time() - started,
                error)
        else:
            rospy.loginfo(
                "model entry: head reset to upstream head_init %s in %.3fs "
                "(-> %s)", list(UPSTREAM_HEAD_INIT), time.time() - started,
                head_ok)
            self._log_head_angles()
        return bool(handed_over)

    def _log_arm_control_mode(self):
        """Report the arm control mode actually in effect, read back."""
        try:
            mode = self.robot_state.arm_control_mode()
        except Exception as error:
            rospy.logwarn(
                "model entry: could not read the arm control mode back (%s)",
                error)
            return
        expected = KuavoArmCtrlMode.ExternalControl
        detail = "READ BACK OK" if mode == expected else \
            "MISMATCH: expected {}".format(expected)
        rospy.loginfo(
            "model entry: arm control mode is now %s (%s)", mode, detail)

    def _log_head_angles(self):
        """Report the measured head angles after the reset command."""
        try:
            positions = list(self.robot_state.head_joint_state().position)
        except Exception as error:
            rospy.logwarn(
                "model entry: could not read the head angles back (%s)", error)
            return
        rospy.loginfo(
            "model entry: measured head angles after reset: %s rad "
            "(commanded %s)", positions, list(UPSTREAM_HEAD_INIT))

    def _notify_initialized(self):
        """Try once; a missing, stalled or rejecting peer never resets the scene."""
        if self._reset_running or self.relaunch.is_set() or rospy.is_shutdown():
            return False
        # A ready-only client may start without implementing /simulator/init.
        if self.evaluation_started:
            return True
        # Keep reset-response ordering, but do not block the main control loop.
        matched_reset = self.reset_handoff.wait_for(self.round_seed, timeout=0.0)
        if not matched_reset and self.reset_handoff.pending_seed() is not None:
            return False
        acknowledged = notify_init(
            lambda: self._reset_running or self.relaunch.is_set() or rospy.is_shutdown())
        if self._reset_running or self.relaunch.is_set() or rospy.is_shutdown():
            return False
        if acknowledged:
            rospy.loginfo(
                "model entry: /simulator/init acknowledged for seed %d",
                self.round_seed)
            return True
        if not self._init_wait_logged:
            rospy.loginfo(
                "model entry: waiting for /simulator/init acknowledgement; "
                "keeping task %d seed %d and the prepared scene; no timeout restart",
                self.task_id, self.round_seed)
            self._init_wait_logged = True
        return False

    def _next_reset_seed(self):
        """Advance after a scored episode; startup retries reuse its seed."""
        return self.round_seed + (1 if self.score_file is not None else 0)

    def _reset(self, _request):
        """Start a fresh round and answer immediately.

        The response must go out *before* this round's ``/simulator/init``,
        because the harness clears ``init_evt`` as soon as a reset returns and
        only then waits on it again -- a notification sent first is wiped and
        the harness waits forever.  The reset therefore does not initialize
        anything.  It records the next seed, opens the process-wide response
        gate and asks main() to replace this simulator.  The fresh protocol
        initializes only after the replacement roslaunch is ready.

        Not waiting here is also what keeps :meth:`_start` free to wait: the
        harness calls ``/simulator/start`` from inside its episode, after this
        handler has already returned.
        """
        with self._lifecycle_lock:
            with self.lock:
                if (self._reset_running or self.relaunch.is_set()
                        or not self.reset_handoff.is_current(self)):
                    return TriggerResponse(success=False, message="a reset is already running")
            try:
                self._finalize_score("reset")
            except Exception as error:
                self._invalidate_active_round(
                    "score finalization failed during reset: {}".format(error))
                return TriggerResponse(
                    success=False,
                    message="score finalization failed; operator action is "
                            "required")
            with self.lock:
                if self._reset_running:
                    return TriggerResponse(
                        success=False, message="a reset is already running")
                # Left True for good: this round's simulator is torn down and
                # rebuilt by main(), so this protocol object never serves a second
                # reset.  Keeping the guard set rejects a reset that races the
                # relaunch instead of silently starting a round nobody will run.
                self._reset_running = True
                self.reset_seed = self._next_reset_seed()
                seed = self.reset_seed
                self.initialized_evt.clear()
                self.ready_publisher.publish(Bool(data=False))
                self.command_accept_publisher.publish(Bool(data=False))
                self.reset_handoff.prepare(seed)
                self.episode_started = False
                if self.score_store is not None:
                    self.score_store.prepare(seed)

            response = TriggerResponse(
                success=True,
                message="round seed {}: re-initializing the scene".format(seed))
            # main() may tear down this protocol as soon as it observes relaunch,
            # so construct the response and open the cross-round gate first.
            self.reset_handoff.mark_response_sent()
            self.relaunch.set()
            return response

    # -- episode start and success judging --------------------------------
    def _start(self, _request):
        """Accept only this generation; the session handoff owns readiness waits."""
        with self._lifecycle_lock:
            with self.lock:
                if (self.relaunch.is_set() or self._reset_running
                        or not self.reset_handoff.is_current(self, ready=True)):
                    return None  # Reset won admission; wait for its replacement.
                if self.episode_started or self.round_complete:
                    return TriggerResponse(success=True, message="episode already started")
                self.evaluation_started = True
            self.publish_success(False)
            if not self._ensure_observer():
                return TriggerResponse(
                    success=False,
                    message="scoring infrastructure failed; operator action is required")
            with self.lock:
                self.episode_started = True
                self.round_complete = False
            return TriggerResponse(success=True, message="episode started")

    def _ensure_observer(self):
        """Start the success watcher on first use."""
        if self.observer is None:
            try:
                from utils.episode_success import EpisodeSuccessObserver
                if self.score_store is None:
                    raise RuntimeError("model score store is not configured")
                self.score_file = self.score_store.allocate(self.round_seed)
                self.observer = EpisodeSuccessObserver(
                    self.task_id, score_file=self.score_file,
                    metadata=self.score_store.metadata(self.score_file))
                self.observer.start()
                rospy.loginfo(
                    "model entry: %s; score file %s",
                    self.observer.describe(), self.score_file)
            except Exception as error:
                rospy.logerr(
                    "model entry: success observer unavailable, "
                    "the round is invalid and requires operator action: %s",
                    error)
                self._invalidate_active_round(
                    "success observer unavailable: {}".format(error))
                return False
        if self.observer_thread is not None and self.observer_thread.is_alive():
            return True
        self.observer_thread = threading.Thread(target=self._watch_success)
        self.observer_thread.daemon = True
        self.observer_thread.start()
        return True

    def _invalidate_active_round(self, reason):
        """Stop a broken scored round and leave recovery to the operator.

        Startup and initialization failures are safe to retry internally before
        an episode is accepted.  Once /simulator/start succeeds, silently
        relaunching would leave the external harness running the retired
        episode.  Mark the score invalid, close command admission, and wake the
        main supervisor so it can exit loudly instead.
        """
        with self.lock:
            if self.infrastructure_failure is None:
                self.infrastructure_failure = str(reason)
            self.invalid_round = True
            self.episode_started = False
        # Wake the supervisor even if ROS is already degraded enough that one
        # of the diagnostic/gating publishes below raises.
        self.relaunch.set()
        if self.observer is not None:
            self.observer.invalidate(reason)
        if self.score_store is not None:
            self.score_store.invalidate(reason)
        self.publish_success(False)
        self.ready_publisher.publish(Bool(data=False))
        self.command_accept_publisher.publish(Bool(data=False))

    def _drop_observer(self):
        """Release the previous round's observer, if any."""
        observer = self.observer
        self.observer = None
        if observer is not None:
            observer.close()

    def shutdown(self):
        """Stop the watcher before the simulator this round owns goes away."""
        with self._lifecycle_lock:
            self.reset_handoff.detach(self)
            self.relaunch.set()
            thread = self.observer_thread
            if thread is not None and thread.is_alive():
                thread.join(timeout=5.0)
            self.observer_thread = None
            if self.invalid_round:
                rospy.logerr(
                    "model entry: discarding invalid task %d seed %d round: %s",
                    self.task_id, self.round_seed, self.infrastructure_failure)
            else:
                self._finalize_score("shutdown")
            self._drop_observer()

    def _finalize_score(self, reason):
        """Write one numeric result for a started model episode."""
        with self.lock:
            observer = self.observer
            if observer is None:
                return None
            # A successful episode is finalized by the watcher before the harness
            # asks for reset.  Do not run the scorer or session accumulator again
            # on that later reset/shutdown path.
            if self.score_finish_reason is not None:
                return None
            result = observer.finish()
            if result is None:
                return None
            total, components = result
            self.score_store.annotate(self.score_file, reason)
            self.score_finish_reason = reason
            rospy.loginfo(
                "model entry: task %d seed %d score=%d reason=%s components=%s "
                "file=%s",
                self.task_id, self.round_seed, int(round(total)),
                self.score_finish_reason,
                components, self.score_file)
            return result

    def _watch_success(self):
        """Judge the running episode and end it through /simulator/success.

        One ``True`` per episode: the harness ends its rollout on the first
        one and clears its event afterwards, so repeating it would only risk
        ending the *next* episode early.
        """
        while not rospy.is_shutdown() and not self.relaunch.is_set():
            if not self.episode_started or self.round_complete:
                time.sleep(SUCCESS_POLL_INTERVAL)
                continue
            try:
                _state, complete = self.observer.sample()
                if complete:
                    self.round_complete = True
                    self._finalize_score("success")
            except Exception as error:
                rospy.logerr(
                    "model entry: scoring failed; the round is "
                    "invalid and requires operator action: %s", error)
                self._invalidate_active_round(
                    "scoring failed: {}".format(error))
                return
            else:
                if complete:
                    rospy.loginfo(
                        "model entry: task %d success, publishing "
                        "/simulator/success=True", self.task_id)
                    self.publish_success(True)
                    continue
            time.sleep(SUCCESS_POLL_INTERVAL)


def main():
    parser = argparse.ArgumentParser(
        description="Start one task simulator, initialize it, wait for a model")
    parser.add_argument("--task-id", type=int, choices=(1, 2, 3), required=True)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--model-name", default="anonymous")
    args = parser.parse_args()

    from utils.model_score_store import ModelScoreStore
    from utils.task2_randomization import seed_for_round, task_scene
    try:
        score_store = ModelScoreStore(args.task_id, args.model_name)
    except ValueError as error:
        parser.error(str(error))
    print(
        "[INFO] model entry: started a fresh score session; latest score={} "
        "average={}".format(
            score_store.score_file, score_store.average_file),
        flush=True)

    # All entries share the same per-round seed; startup retries reuse it.
    round_seed = args.seed if args.seed is not None else seed_for_round(1)
    # The old protocol answers reset and the replacement protocol sends init.
    # Their ordering gate must therefore outlive each round-local object.
    reset_handoff = ResetHandoff()

    signal.signal(signal.SIGTERM, request_shutdown)

    os.environ.setdefault("KUAVO_LOG_SERVER", "0")
    os.environ.setdefault("KUAVO_LEG_SERVICE", "0")

    # A round's scene layout and its base spawn point are paired: Task 1's
    # saved layouts are only verified feasible from their own initial_base,
    # and that spawn is a roslaunch argument.  So each round owns a fresh
    # simulator process, and /simulator/reset starts the next one instead of
    # re-placing objects in place.
    with reset_handoff, ShutdownGuard(args.task_id) as guard, ModelMaster() as master:
        while True:
            # The master belongs to the session, never to a disposable round.
            master.check()
            seed = round_seed
            score_store.prepare(seed)
            process = None
            protocol = None
            round_error = None
            relaunching = False
            manual_intervention = False
            scene_context = None
            try:
                ensure_clean_graph(args.task_id)
                scene_context = task_scene(args.task_id, seed)
                scene_file = scene_context.__enter__()
                base_x, base_y = initial_base_for(args.task_id, seed)
                print(
                    "[INFO] model entry: task {} seed {} initial_base="
                    "({:.4f}, {:.4f})".format(
                        args.task_id, seed, base_x, base_y), flush=True)

                environment = os.environ.copy()
                if args.headless:
                    environment["MUJOCO_HEADLESS"] = "1"
                process = guard.add(subprocess.Popen(
                    ["roslaunch", "data_challenge_simulator",
                     MODEL_LAUNCH_FILES[args.task_id],
                     "initial_base_x:={:.9f}".format(base_x),
                     "initial_base_y:={:.9f}".format(base_y),
                     "task_light_seed:={}".format(seed)] + (
                         ["legged_robot_scene:={}".format(scene_file)] if scene_file else []),
                    env=environment, start_new_session=True),
                    process_group=True)

                # Duplicate init_node calls are allowed as long as the arguments
                # are identical (rospy's #972 guard), so a relaunch reuses the
                # same node -- and its still-registered services -- rather than
                # re-registering under a new one.
                rospy.init_node("model_simulator_entry", disable_signals=True)

                required = (
                    SENSOR_TOPIC,
                    # Advertised services and /cmd_vel connections appear
                    # before the MRT has consumed its first policy.  The first
                    # MPC observation is the runtime boundary after reset at
                    # which wheel commands are actually effective.
                    "/mobile_manipulator_mpc_observation",
                    "/cam_h/color/image_raw/compressed",
                    "/cam_l/color/image_raw/compressed",
                    "/cam_r/color/image_raw/compressed",
                    "/sg100_hand_state",
                    "/mujoco/l_hand_base/pose",
                    "/mujoco/r_hand_base/pose",
                    OBJECT_TOPICS[args.task_id],
                )
                for topic in required:
                    print(
                        "[INFO] model entry: waiting for {}".format(topic),
                        flush=True)
                    wait_for_topic(topic, timeout=60.0)
                print(
                    "[INFO] model entry: all required topics are live",
                    flush=True)

                from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
                # Built once and re-Init'd for later rounds.  Re-Init is
                # deliberate -- the SDK holds publisher/subscriber connections to
                # the simulator that was just torn down -- but it reassigns the
                # module-level core, so a cached KuavoRobot/KuavoRobotState would
                # keep reading the dead one.  Both objects are therefore made
                # fresh after every Init.
                # SDK initialization queries the arm mode immediately.
                # Live observation topics alone do not imply these services exist.
                EvaluationProtocol._wait_for_controller_ready()
                sdk = KuavoSDK()
                if not sdk.Init(
                        options=getattr(KuavoSDK.Options, SDK_OPTIONS[args.task_id])):
                    raise RuntimeError("KuavoSDK initialization failed")

                print("[INFO] model entry: initializing task {} ...".format(
                    args.task_id), flush=True)
                robot = KuavoRobot()
                robot_state = KuavoRobotState()
                ready_publisher = rospy.Publisher(
                    "/model_simulator/ready", Bool, queue_size=1, latch=True)
                command_accept_publisher = rospy.Publisher(
                    "/model_simulator/accept_commands", Bool,
                    queue_size=1, latch=True)
                protocol = EvaluationProtocol(
                    args.task_id, robot, robot_state, seed, ready_publisher,
                    command_accept_publisher,
                    reset_handoff=reset_handoff, score_store=score_store)
                reset_handoff.serve()
                protocol._initialize()
                if not protocol.relaunch.is_set():
                    print("[INFO] model entry: initialization complete", flush=True)
                    rospy.loginfo(
                        "Task %d model simulator ready at seed %d; scene initialized "
                        "and the arm is at the fixed ready posture. No task policy and "
                        "no rosbag were started. Waiting for external inference.",
                        args.task_id, seed)
                init_acknowledged = False
                next_init_attempt = 0.0
                while (not rospy.is_shutdown() and process.poll() is None
                       and not protocol.relaunch.is_set()):
                    if not init_acknowledged and time.monotonic() >= next_init_attempt:
                        init_acknowledged = protocol._notify_initialized()
                        next_init_attempt = time.monotonic() + INIT_RETRY_INTERVAL
                    protocol.relaunch.wait(0.1)
                if protocol.infrastructure_failure is not None:
                    raise RuntimeError(protocol.infrastructure_failure)
                if process.poll() is not None:
                    raise RuntimeError(
                        "model roslaunch exited with code {}".format(
                            process.returncode))
                round_seed = protocol.reset_seed
                print("[INFO] model entry: /simulator/reset -> starting round with "
                      "seed {}".format(round_seed), flush=True)
            except Exception as error:
                # Before /simulator/start, a broken topic, SDK/init failure or
                # failed init callback invalidates only the disposable startup
                # attempt.  After scoring starts, an automatic relaunch would
                # desynchronise the external harness from its active episode;
                # make that failure loud and require operator intervention.
                round_error = error
                manual_intervention = bool(
                    protocol is not None and protocol.evaluation_started)
                if manual_intervention:
                    protocol._invalidate_active_round(error)
                else:
                    score_store.invalidate(error)
                print(
                    "[ERROR] model entry: task {} seed {} failed before "
                    "teardown: {}; {}".format(
                        args.task_id, seed, error,
                        "operator action is required" if manual_intervention
                        else "the startup attempt will be skipped"),
                    file=sys.stderr, flush=True)
            finally:
                # Read the relaunch request *before* shutdown(), which sets it.
                relaunching = protocol is not None and protocol.relaunch.is_set()
                if protocol is not None:
                    try:
                        protocol.shutdown()
                    except Exception as cleanup_error:
                        print(
                            "[ERROR] model entry: task {} seed {} protocol "
                            "cleanup failed: {}".format(
                                args.task_id, seed, cleanup_error),
                            file=sys.stderr, flush=True)
                        if round_error is None:
                            round_error = cleanup_error
                            manual_intervention = bool(
                                protocol.evaluation_started)
                # Ctrl+C during the topic waits or the initialization above
                # unwinds through here with the simulator still up, so this
                # teardown has to be the one that runs -- the guard keeps a
                # second Ctrl+C from aborting it half way.
                try:
                    guard.sweep()
                    clear_dead_simulator_nodes()
                finally:
                    if scene_context is not None:
                        scene_context.__exit__(None, None, None)

            if round_error is not None:
                # A lost master invalidates the evaluation session, not just
                # one layout. Do not silently recreate its registration graph.
                master.check()
                if manual_intervention:
                    raise round_error
                if rospy.is_shutdown():
                    break
                # Honour a harness reset that raced the failed initialization.
                # Otherwise choose a fresh internal seed.  Retargeting keeps an
                # already-answered reset gate open, so the harness remains in
                # the same wait and the first usable replacement calls init.
                if relaunching:
                    round_seed = protocol.reset_seed
                else:
                    round_seed = seed
                    reset_handoff.retarget(round_seed)
                print(
                    "[WARN] model entry: continuing after failed task {} "
                    "round; next seed {}".format(args.task_id, round_seed),
                    file=sys.stderr, flush=True)
                time.sleep(ROUND_RETRY_DELAY)
                continue
            if not relaunching:
                break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as error:
        print("[ERROR] {}".format(error), file=sys.stderr)
        sys.exit(1)
    finally:
        # The supervisor is finished; repeated stop requests must let rospy's
        # atexit handlers finish releasing their remaining threads.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
