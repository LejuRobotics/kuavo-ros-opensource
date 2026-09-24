import subprocess
import time
import signal
import os
import json
from datetime import datetime
import rospy
import rostopic
from std_msgs.msg import Empty
from std_srvs.srv import Trigger, TriggerResponse
import argparse
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

from utils.shutdown_guard import (
    ShutdownGuard,
    clear_simulator_nodes,
    clear_stale_processes,
)
from utils.collect_score_store import allocate_collect_score_file

TASK_SCRIPTS = {
    1: 'task1.py',
    2: 'task2.py',
    3: 'task3.py',
}

SIMULATOR_LAUNCH_FILES = {
    1: 'load_kuavo_mujoco_sim1.launch',
    2: 'load_kuavo_mujoco_sim2.launch',
    3: 'load_kuavo_mujoco_sim3.launch',
}

# Passive score observer (upstream deploy/eval*.py pattern): a separate
# process that watches /mujoco/<body>/pose and applies the official scoring
# rules.  Failures here must never break the round.
SCORER_SCRIPT = os.path.join(PACKAGE_DIR, 'utils', 'task_scorer.py')
SCORES_DIR = os.path.join(SCRIPT_DIR, 'scores')
SCORER_FINALIZE_TIMEOUT = 10.0

# Every task script writes its outcome here (relative to EXAMPLES_DIR, which
# is the working directory), and helperfunc.py reads and deletes it.
TASK_RESULT_FILE = "task_result.txt"


class RoundStartupError(RuntimeError):
    """The simulator did not become usable for the current round."""


def clear_task_result():
    """Drop any leftover task_result.txt before a round starts."""
    try:
        os.remove(TASK_RESULT_FILE)
    except OSError:
        pass


def read_task_result(round_id):
    """Consume this round's outcome, then delete the file.

    Reading and deleting have to stay together: the file is shared by all
    task scripts, so anything left behind is read as the *next* round's
    outcome -- a stale "success" would pass a round that actually failed.
    """
    try:
        with open(TASK_RESULT_FILE, "r") as stream:
            result = stream.read().strip()
    except OSError:
        print(f"[WARNING] Round {round_id}: 未生成任务结果文件，默认失败")
        return False
    finally:
        try:
            os.remove(TASK_RESULT_FILE)
        except OSError:
            pass
    return result == "success"


def clear_score_result(task_id, round_id):
    """Remove score artifacts so a failed startup cannot reuse an old score."""
    score_file = os.path.join(
        SCORES_DIR, 'score_task{}_round{}.txt'.format(task_id, round_id))
    try:
        os.makedirs(SCORES_DIR, exist_ok=True)
        for path in (score_file, os.path.splitext(score_file)[0] + '.json'):
            if os.path.exists(path):
                os.remove(path)
    except OSError as error:
        print('[WARNING] 评分文件清理失败: {}'.format(error))
    return score_file


def start_scorer(task_id, round_id, score_file=None):
    """Launch the scorer subprocess and return (process, score_file)."""
    if score_file is None:
        # Non-recording task runs keep their accepted replace-in-place path.
        score_file = clear_score_result(task_id, round_id)
    environment = os.environ.copy()
    environment['SCORE_FILE'] = score_file
    environment.setdefault('KUAVO_LOG_SERVER', '0')
    environment.setdefault('KUAVO_LEG_SERVICE', '0')
    process = subprocess.Popen(
        ['python3', SCORER_SCRIPT, '--task-id', str(task_id)],
        env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    print('[INFO] Round {}: 评分器已启动 (task {}, {})'.format(
        round_id, task_id, score_file))
    return process, score_file


def read_score(task_id, round_id, score_file=None):
    """Read the round score written by the scorer (None if missing)."""
    if score_file is None:
        score_file = os.path.join(
            SCORES_DIR, 'score_task{}_round{}.txt'.format(task_id, round_id))
    try:
        with open(score_file, 'r') as stream:
            value = stream.read().strip()
        return float(value) if value else None
    except (OSError, ValueError):
        return None


def stop_scorer(task_id, round_id, scorer_process, score_file):
    """Finalize the scorer, print its score, and reap the subprocess."""
    total = None
    components = None
    if scorer_process is not None and scorer_process.poll() is None:
        try:
            rospy.wait_for_service(
                '/task_scorer/finalize', timeout=SCORER_FINALIZE_TIMEOUT)
            proxy = rospy.ServiceProxy('/task_scorer/finalize', Trigger)
            response = proxy()
            payload = json.loads(response.message)
            total = payload.get('total')
            components = payload.get('components')
        except Exception as error:
            print('[WARNING] 评分器 finalize 失败: {}'.format(error))
        finally:
            try:
                os.killpg(os.getpgid(scorer_process.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                scorer_process.terminate()
            try:
                scorer_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(scorer_process.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    scorer_process.kill()
    if total is None:
        # Fall back to whatever the scorer managed to write on its own.
        try:
            with open(score_file, 'r') as stream:
                total = stream.read().strip()
            detail = os.path.splitext(score_file)[0] + '.json'
            if os.path.exists(detail):
                with open(detail, 'r') as stream:
                    components = json.load(stream).get('components')
        except (OSError, ValueError):
            total = None
    if total is None:
        print('[WARNING] Round {}: 评分器未产出分数'.format(round_id))
    else:
        print('[INFO] Round {}: 📊 得分 {} | 分项 {}'.format(
            round_id, total, components))
    return total, components


LIMIT_ANGLE = [
        [-180, 90], #L1
        [-20, 120], #L2
        [-90, 90], #L3
        [-150, 0], #L4
        [-90, 90], #L5
        [-75, 40], #L6
        [-40, 40], #L7

        [-180, 90], #R1
        [-120, 20], #R2
        [-90, 90], #R3
        [-150, 0], #R4
        [-90, 90], #R5
        [-40, 75], #R6
        [-40, 40], #R7
]

ROS_TOPICS = [
    '/sensors_data_raw',
    '/kuavo_arm_traj',
    '/joint_cmd',
    '/tf',
    '/tf_static',
    '/sg100_hand_command',
    '/sg100_hand_state',
    '/cam_h/color/image_raw/compressed',
    '/cam_l/color/image_raw/compressed',
    '/cam_r/color/image_raw/compressed',
    '/cmd_vel',
    '/kuavo_arm_traj_synced',
]

# Runtime-only hand state, IK/FK compatibility outputs, and the simulator's
# ground-truth base pose stay published but are intentionally excluded here.
# Task-object ground-truth topics (/mujoco/<object>/pose and the task
# grasp/state flags) are intentionally NOT recorded: the bags are training
# data for external contestants, and the simulator's true object poses and
# grasp-latch states would leak the answer.  The topics stay published for
# task-logic subscribers and wait_for_topics readiness checks.

TASK2_ROS_TOPICS = list(ROS_TOPICS)

TASK3_ROS_TOPICS = list(ROS_TOPICS)


def task2_initial_base_translation(round_id):
    """Read Scene 2's launch-time base pose from the saved layout."""
    from utils.task2_randomization import Task2RandomizationPlanner

    return Task2RandomizationPlanner().plan(round_id).initial_base[:2]


def task3_initial_base_translation(round_id):
    """Read Scene 3's launch-time base pose from the saved layout."""
    from utils.task3_randomization import Task3RandomizationPlanner

    return Task3RandomizationPlanner().plan(round_id).initial_base[:2]


def simulator_launch_command(task_id, round_id):
    try:
        launch_file = SIMULATOR_LAUNCH_FILES[task_id]
    except KeyError as error:
        raise ValueError('unsupported task id: {}'.format(task_id)) from error
    command = [
        'roslaunch', 'data_challenge_simulator',
        launch_file,
        'task_light_seed:={}'.format(round_id),
    ]
    if task_id == 1:
        from utils.task1_v2_randomization import Task1V2RandomizationPlanner
        initial_base = Task1V2RandomizationPlanner().plan(round_id).initial_base
        command.extend([
            f'initial_base_x:={initial_base[0]:.9f}',
            f'initial_base_y:={initial_base[1]:.9f}',
        ])
        print(
            f"[INFO] Round {round_id}: Task 1 initial base "
            f"x={initial_base[0]:.4f}, y={initial_base[1]:.4f}")
    if task_id == 2:
        base_x, base_y = task2_initial_base_translation(round_id)
        command.extend([
            f'initial_base_x:={base_x:.9f}',
            f'initial_base_y:={base_y:.9f}',
        ])
        print(
            f"[INFO] Round {round_id}: Task 2 initial base "
            f"x={base_x:.4f}, y={base_y:.4f}")
    if task_id == 3:
        base_x, base_y = task3_initial_base_translation(round_id)
        command.extend([
            f'initial_base_x:={base_x:.9f}',
            f'initial_base_y:={base_y:.9f}',
        ])
        print(
            f"[INFO] Round {round_id}: Task 3 initial base "
            f"x={base_x:.4f}, y={base_y:.4f}")
    return command


def ensure_clean_simulator_graph(task_id, timeout=10.0):
    """Kill any simulator left over from an interrupted round, then proceed."""
    clear_stale_processes(task_id)
    clear_simulator_nodes(timeout=timeout)
    # roscore stays alive across rounds, so parameters outlive the controller
    # and MuJoCo node that produced them.  MuJoCo treats mere existence of
    # robot_init_state_param as its qpos-ready signal; remove the old signal
    # after all old writers are gone so this round must wait for its controller
    # to publish a fresh initial state from the current launch arguments.
    try:
        rospy.delete_param('/robot_init_state_param')
        print("[INFO] Cleared stale simulator initial state")
    except KeyError:
        pass

def record_topics(task_id):
    if task_id == 2:
        return TASK2_ROS_TOPICS
    if task_id == 3:
        return TASK3_ROS_TOPICS
    return ROS_TOPICS


def advertise_recording_gate(round_id, task_id, bag_filename, guard):
    """Start rosbag only when the task reports initialization complete.

    The task blocks in the service call until rosbag has had the same one
    second subscription warm-up used by the old pre-task recording path.  It
    therefore cannot begin the scored action while rosbag is still starting.
    """
    state = {"process": None}

    def start_recording(_request):
        if state["process"] is not None:
            return TriggerResponse(
                success=True, message="recording already started")
        try:
            process = subprocess.Popen(
                ['rosbag', 'record', '-O', bag_filename]
                + record_topics(task_id),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True)
            state["process"] = guard.add(
                process, graceful=True, process_group=True)
            print(
                f"[INFO] Round {round_id}: Initialization complete; "
                f"recording → {bag_filename}")
            time.sleep(1)
            if process.poll() is not None:
                return TriggerResponse(
                    success=False,
                    message="rosbag record exited during startup")
            return TriggerResponse(success=True, message="recording started")
        except (OSError, ValueError) as error:
            return TriggerResponse(success=False, message=str(error))

    service = rospy.Service(
        '/data_collector/start_recording', Trigger, start_recording)
    print(
        f"[INFO] Round {round_id}: Rosbag is waiting for task "
        "initialization to complete")
    return service


def wait_for_topics(task_id=1, timeout=15):
    """等待关键话题开始发布数据"""
    print("[INFO] 等待关键话题发布数据...")
    start_time = time.time()
    object_topic = {
        1: '/mujoco/cylinder_1/pose',
        2: '/mujoco/box_1/pose',
        3: '/mujoco/task3_hollow_cylinder/pose',
    }[task_id]
    key_topics = [
        '/sensors_data_raw',
        '/joint_cmd',
        '/sg100_hand_state',
        object_topic,
    ]

    while time.time() - start_time < timeout:
        try:
            # 检查关键话题是否有数据发布
            missing_topics = []
            for topic in key_topics:
                try:
                    # 尝试获取话题的最新消息，超时时间设为1秒
                    msg_class, _, _ = rostopic.get_topic_class(topic)
                    if msg_class is None:
                        missing_topics.append(topic)
                        continue

                    # 检查是否有消息发布
                    data = rospy.wait_for_message(topic, msg_class, timeout=1.0)
                    if data is None:
                        missing_topics.append(topic)
                except Exception:
                    missing_topics.append(topic)

            if not missing_topics:
                print(f"[INFO] 关键话题就绪，耗时 {time.time() - start_time:.1f} 秒")
                return True
            else:
                print(f"[INFO] 等待话题: {missing_topics}")
                time.sleep(1)

        except Exception:
            time.sleep(1)

    print("[WARNING] 等待话题超时")
    return False


def wait_for_simulator_ready(task_id, timeout=60):
    """Require both observation topics and the MuJoCo control service."""
    if not wait_for_topics(task_id=task_id, timeout=timeout):
        raise RoundStartupError(
            "key simulator topics did not become ready within {} seconds".format(
                timeout))
    try:
        rospy.wait_for_service('/set_object_position', timeout=timeout)
    except rospy.ROSException as error:
        raise RoundStartupError(
            "/set_object_position did not become ready within {} seconds".format(
                timeout)) from error


def report_startup_failure(round_id, start_time, bag_filename=None):
    """Record one failed attempt without aborting the remaining rounds."""
    clear_task_result()
    if bag_filename and os.path.exists(bag_filename):
        os.remove(bag_filename)
        print(
            f"[INFO] Round {round_id}: Removed incomplete bag "
            f"{bag_filename}")
    duration = time.time() - start_time
    print(
        f"[INFO] Round {round_id}: ❌ 仿真启动失败，本轮计入失败并继续下一轮")
    print(
        f"[{datetime.now()}] Round {round_id} finished in "
        f"{duration:.2f} sec.\n")
    return duration, False


def run_only_task(
        round_id: int, task_id: int, headless: bool,
        keep_sim_on_failure: bool = False):
    start_time = time.time()
    ensure_clean_simulator_graph(task_id)
    clear_task_result()
    clear_score_result(task_id, round_id)

    task_script = os.path.join(SCRIPT_DIR, TASK_SCRIPTS[task_id])

    if headless:
        # # 设置 DISPLAY 环境变量
        task_env = os.environ.copy()
        # task_env["DISPLAY"] = display_num
        task_env["MUJOCO_HEADLESS"] = "1"
    else:
        task_env = os.environ.copy()  # 不改变 DISPLAY

    try:
        with ShutdownGuard(task_id) as guard:
            return _run_only_task(
                round_id, task_id, headless, keep_sim_on_failure, task_script,
                task_env, start_time, guard)
    except RoundStartupError as error:
        print(f"[ERROR] Round {round_id}: {error}")
        return report_startup_failure(round_id, start_time)


def _run_only_task(
        round_id, task_id, headless, keep_sim_on_failure, task_script,
        task_env, start_time, guard):
    # 启动仿真环境
    guard.add(subprocess.Popen(
        simulator_launch_command(task_id, round_id),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
        env = task_env
    ), process_group=True)
    print(f"[INFO] Round {round_id}: Launched Task {task_id} simulator")
    time.sleep(2)

    try:
        rospy.init_node('data_collector', anonymous=True, disable_signals=True)
    except rospy.exceptions.ROSException:
        pass
    wait_for_simulator_ready(task_id=task_id, timeout=60)

    # 在后台启动任务脚本但不等待完成
    print(f"[INFO] Round {round_id}: Starting {task_script} in background")
    task_process_env = os.environ.copy()
    task_process_env['TASK_SEED'] = str(round_id)
    # No-record runs are action experiments: task-level acceptance failures
    # are reported, but must not prevent later independent action stages from
    # running. Recorded collection keeps fail-fast acceptance semantics.
    task_process_env['TASK_EXPERIMENT_MODE'] = '1'
    # 仿真流程里没有 8889 日志服务器（它只在上位机 plan_arm_action_websocket_server
    # 里起），也没有 /lb_leg_control_srv（只在 motion_capture_ik 的
    # leg_ik_service_node 里提供）：整步跳过这两处连接/等待，避免每次启动白等。
    task_process_env['KUAVO_LOG_SERVER'] = '0'
    task_process_env['KUAVO_LEG_SERVICE'] = '0'
    task_process_env['TASK_SCORER_ENABLED'] = '1'
    task_process_env['TASK_RECORD_AFTER_INIT'] = '0'

    # The scorer observes from the beginning, but its time-penalty clock is
    # started explicitly by the task after fixed initialization completes.
    scorer_process, score_file = start_scorer(task_id, round_id)
    guard.add(scorer_process, process_group=True)
    task_process = guard.add(subprocess.Popen(
        ['python3', task_script], env=task_process_env,
        start_new_session=True), process_group=True)

    task_completed = False
    try:
        # 等待任务脚本完成
        print(f"[INFO] Round {round_id}: Waiting for task completion")
        task_process.wait()
        task_completed = True
    finally:
        # An interrupted task must stop publishing before optional scorer
        # finalization consumes its timeout budget.
        if task_completed:
            stop_scorer(task_id, round_id, scorer_process, score_file)
        print(f"[INFO] Round {round_id}: Stopping simulation...")
        guard.sweep()

    # 结果文件由这里读走并删掉，不在 finally 里删：finally 先于这里执行，
    # 在那里删会把本轮结果抹掉，让每轮都变成"未生成结果文件，默认失败"。
    is_success = read_task_result(round_id)

    if not is_success:
        print(f"[INFO] Round {round_id}: ❌ 任务失败")
        if keep_sim_on_failure:
            try:
                input("[INFO] 仿真窗口已保留，检查现场后按回车关闭：")
            except (EOFError, KeyboardInterrupt):
                pass
    else:
        print(f"[INFO] Round {round_id}: ✅ 任务成功")

    end_time = time.time()
    duration = end_time - start_time
    print(f"[{datetime.now()}] Round {round_id} finished in {duration:.2f} sec.\n")

    return duration, is_success


def run_once(round_id: int, task_id: int,headless: bool):
    start_time = time.time()
    ensure_clean_simulator_graph(task_id)
    bag_filename = f"data_round_{round_id:07d}.bag"
    # The task result is transient, but collect score files are persistent and
    # allocated only after simulator startup; never delete an earlier score.
    clear_task_result()

    task_script = os.path.join(SCRIPT_DIR, TASK_SCRIPTS[task_id])

    if headless:
        # # 设置 DISPLAY 环境变量
        task_env = os.environ.copy()
        # task_env["DISPLAY"] = display_num
        task_env["MUJOCO_HEADLESS"] = "1"
    else:
        task_env = os.environ.copy()  # 不改变 DISPLAY

    try:
        with ShutdownGuard(task_id) as guard:
            return _run_once(
                round_id, task_id, bag_filename, task_script,
                task_env, start_time, guard)
    except RoundStartupError as error:
        print(f"[ERROR] Round {round_id}: {error}")
        duration, is_success = report_startup_failure(
            round_id, start_time, bag_filename)
        return duration, is_success, None


def _run_once(
        round_id, task_id, bag_filename, task_script,
        task_env, start_time, guard):
    # 启动仿真环境
    guard.add(subprocess.Popen(
        simulator_launch_command(task_id, round_id),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
        env = task_env
    ), process_group=True)
    print(f"[INFO] Round {round_id}: Launched Task {task_id} simulator")
    time.sleep(2)

    # 等待关键话题准备就绪
    try:
        rospy.init_node('data_collector', anonymous=True, disable_signals=True)
    except rospy.exceptions.ROSException:
        pass

    print(f"[INFO] Round {round_id}: 等待话题发布...")
    wait_for_simulator_ready(task_id=task_id, timeout=60)

    # 上一轮若是被 Ctrl+C 打断，结果文件可能残留；先删掉，
    # 避免本轮读到上一轮的结果。
    clear_task_result()

    # The task performs initialization first, then blocks on this service until
    # rosbag is ready.  Only post-initialization task action is recorded.
    recording_service = advertise_recording_gate(
        round_id, task_id, bag_filename, guard)
    print(f"[INFO] Round {round_id}: Starting {task_script} in background")
    task_process_env = os.environ.copy()
    task_process_env['TASK_SEED'] = str(round_id)
    task_process_env['TASK_EXPERIMENT_MODE'] = '0'
    # 仿真流程里没有 8889 日志服务器（它只在上位机 plan_arm_action_websocket_server
    # 里起），也没有 /lb_leg_control_srv（只在 motion_capture_ik 的
    # leg_ik_service_node 里提供）：整步跳过这两处连接/等待，避免每次启动白等。
    task_process_env['KUAVO_LOG_SERVER'] = '0'
    task_process_env['KUAVO_LEG_SERVICE'] = '0'
    task_process_env['TASK_SCORER_ENABLED'] = '1'
    task_process_env['TASK_RECORD_AFTER_INIT'] = '1'

    # Observe the whole round while delaying only the time-penalty clock until
    # the task reports that its fixed initialization has completed.
    score_file = allocate_collect_score_file(
        SCORES_DIR, task_id, round_id)
    scorer_process, score_file = start_scorer(
        task_id, round_id, score_file=score_file)
    guard.add(scorer_process, process_group=True)
    task_process = guard.add(subprocess.Popen(
        ['python3', task_script], env=task_process_env,
        start_new_session=True), process_group=True)

    task_completed = False
    try:
        # 等待任务脚本完成
        print(f"[INFO] Round {round_id}: Waiting for task completion")
        task_process.wait()
        task_completed = True
    finally:
        recording_service.shutdown('round complete')
        if task_completed:
            # Only a naturally completed task needs its final compatibility
            # sample and score.  On interruption, stop command publishers first.
            try:
                rospy.wait_for_service(
                    '/rosbag_compat/publish_current_base_command', timeout=2.0)
                response = rospy.ServiceProxy(
                    '/rosbag_compat/publish_current_base_command', Trigger)()
                if response.success:
                    print("[INFO] Recorded final /cmd_pose_world compatibility sample")
                    time.sleep(0.5)
                else:
                    print(f"[WARNING] Failed to publish /cmd_pose_world: {response.message}")
            except (rospy.ROSException, rospy.ServiceException) as error:
                print(f"[WARNING] Failed to call cmd-pose compatibility service: {error}")

            stop_scorer(task_id, round_id, scorer_process, score_file)

        # 停止 rosbag 和 simulation。放在 finally 里：Ctrl+C 打断等待时
        # 也必须把这两个进程组收掉，否则残留会让下一轮起不来。
        # ShutdownGuard 保证第二次 Ctrl+C 不会把这段清理打断，
        # 并按名字兜底扫掉句柄已失效的残留（rosbag 先收 SIGINT 以便写完文件）。
        print(f"[INFO] Round {round_id}: Stopping rosbag and simulation...")
        guard.sweep()

    # 结果文件由这里读走并删掉，不在 finally 里删：finally 先于这里执行，
    # 在那里删会把本轮结果抹掉，让每轮都变成"未生成结果文件，默认失败"。
    is_success = read_task_result(round_id)

    if not is_success:
        print(f"[INFO] Round {round_id}: ❌ 任务失败，删除 {bag_filename}")
        if os.path.exists(bag_filename):
            os.remove(bag_filename)
    else:
        print(f"[INFO] Round {round_id}: ✅ 任务成功，保留 {bag_filename}")

    end_time = time.time()
    duration = end_time - start_time
    print(f"[{datetime.now()}] Round {round_id} finished in {duration:.2f} sec.\n")

    return duration, is_success, score_file


def main(
        headless, task_id=None, record_choice=None, repeat_times=None,
        keep_sim_on_failure=False, start_seed=1):
    print("========== 数据采集任务启动 ==========")
    print("当前可用任务:")
    print("1: 场景1 —— 三圆柱分拣")
    print("2: 场景2 —— 双臂搬运两个箱子")
    print("3: 场景3 —— 右手三指内撑搬运中空圆柱")

    # The prompts below block on input().  Without a guard, Ctrl+C during a
    # prompt kills the process outright -- there is nothing to tear down yet,
    # but the traceback is noise the acceptance run does not need.
    with ShutdownGuard(task_id if task_id in (1, 2, 3) else 1):
        return _main(
            headless, task_id, record_choice, repeat_times,
            keep_sim_on_failure, start_seed)


def _main(
        headless, task_id, record_choice, repeat_times,
        keep_sim_on_failure, start_seed):
    try:
        if task_id is None:
            task_id = int(input("请输入任务编号 (1/2/3): ").strip())
        if task_id not in (1, 2, 3):
            raise ValueError()

        if record_choice is None:
            record_choice = int(input("是否录制 rosbag？1=是，0=否：").strip())
        if record_choice not in [0, 1]:
            raise ValueError()

        if repeat_times is None:
            repeat_times = int(input("请输入循环次数: ").strip())
        if repeat_times <= 0 or start_seed < 0:
            raise ValueError()
    except ValueError:
        print("[ERROR] 输入无效，请输入有效数字。")
        return

    durations = []
    scores = []
    if record_choice == 0:
        all_start_time = time.time()
        success_count = 0

        for i in range(repeat_times):
            round_id = start_seed + i
            print(f"\n====== 🚀 Round {i+1}/{repeat_times} ======")
            duration, is_success = run_only_task(
                round_id, task_id, headless, keep_sim_on_failure)
            durations.append(duration)
            scores.append(read_score(task_id, round_id))
            if is_success:
                success_count += 1
            time.sleep(1)

    if record_choice == 1:
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        root_dir = os.path.join("bags", f"run_{timestamp}")
        os.makedirs(root_dir, exist_ok=True)
        os.chdir(root_dir)

        all_start_time = time.time()
        success_count = 0

        for i in range(repeat_times):
            round_id = start_seed + i
            print(f"\n====== 🚀 Round {i+1}/{repeat_times} ======")
            duration, is_success, score_file = run_once(
                round_id, task_id, headless)
            durations.append(duration)
            scores.append(
                None if score_file is None else read_score(
                    task_id, round_id, score_file=score_file))
            if is_success:
                success_count += 1
            time.sleep(3)

    all_end_time = time.time()
    total_time = all_end_time - all_start_time

    with open("result.txt", "w") as f:
        f.write("============== ✅ 所有采集完成 ==============\n")
        for i, d in enumerate(durations):
            f.write(f"⏱️ Round {i+1}: {d:.2f} 秒\n")
        f.write(f"\n📊 总次数: {repeat_times}\n")
        f.write(f"✅ 成功次数: {success_count}\n")
        f.write(f"❌ 失败次数: {repeat_times - success_count}\n")
        f.write(f"📈 成功率: {(success_count / repeat_times) * 100:.2f}%\n")
        valid_scores = [s for s in scores if s is not None]
        if valid_scores:
            score_labels = [
                '%.1f' % score if score is not None else 'N/A'
                for score in scores
            ]
            f.write(f"🏆 各轮得分: {score_labels}\n")
            f.write(
                f"🏆 平均分: {sum(valid_scores) / len(valid_scores):.2f} "
                f"(有效 {len(valid_scores)}/{repeat_times} 轮)\n")
        f.write(f"🧾 总耗时: {total_time:.2f} 秒\n")

    print(open("result.txt").read())
    os.remove("result.txt")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="是否使用无头模式")
    parser.add_argument("--task-id", type=int, choices=[1, 2, 3])
    parser.add_argument("--record", type=int, choices=[0, 1])
    parser.add_argument("--repeat", type=int)
    parser.add_argument(
        "--start-seed", type=int, default=1,
        help="首轮 TASK_SEED，默认 1")
    parser.add_argument(
        "--keep-sim-on-failure", action="store_true",
        help="不录包调试失败时保留仿真窗口，按回车后关闭")
    args = parser.parse_args()
    try:
        main(
            args.headless,
            task_id=args.task_id,
            record_choice=args.record,
            repeat_times=args.repeat,
            keep_sim_on_failure=args.keep_sim_on_failure,
            start_seed=args.start_seed,
        )
    except KeyboardInterrupt:
        print("[INFO] 数据采集已中断，清理完成。")
        sys.exit(130)
