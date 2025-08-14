import subprocess
import time
import signal
import os
from datetime import datetime
import rospy
import rostopic

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

ROS_TOPICS = [
    '/gripper/command',
    '/gripper/state',
    '/sensors_data_raw',
    '/kuavo_arm_traj',
    '/joint_cmd',
    '/cam_h/color/image_raw/compressed',
    '/cam_l/color/image_raw/compressed',
    '/cam_r/color/image_raw/compressed',

    '/cam_h/depth/image_raw/compressedDepth',
    '/cam_r/depth/image_rect_raw/compressedDepth',
    '/cam_l/depth/image_rect_raw/compressedDepth',
]

def wait_for_topics(timeout=15):
    """等待关键话题开始发布数据"""
    print(f"[INFO] 等待关键话题发布数据...")
    start_time = time.time()
    key_topics = ['/kuavo_arm_traj', '/gripper/command','/sensors_data_raw']
    
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
                except:
                    missing_topics.append(topic)
            
            if not missing_topics:
                print(f"[INFO] 关键话题就绪，耗时 {time.time() - start_time:.1f} 秒")
                return True
            else:
                print(f"[INFO] 等待话题: {missing_topics}")
                time.sleep(1)
                
        except Exception as e:
            time.sleep(1)
            
    print(f"[WARNING] 等待话题超时，继续执行...")
    return False

def run_only_task(task_id: int):
    task_script = os.path.join(SCRIPT_DIR, f"task{task_id}.py")
    launch_file = f"load_kuavo_mujoco_sim{task_id}.launch"

    print(f"[INFO] 正在运行任务 {task_id}（不录制 rosbag）")

    # 启动 Mujoco 仿真环境
    launch_process = subprocess.Popen(
        ['roslaunch', 'data_challenge_simulator', launch_file],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    print(f"[INFO] 启动仿真环境：{launch_file}")
    time.sleep(3)  # 给仿真环境更多启动时间

    # 启动任务脚本
    print(f"[INFO] 运行任务脚本：{task_script}")
    subprocess.run(['python3', task_script])

    # 关闭仿真环境
    print(f"[INFO] 关闭仿真环境...")
    os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
    time.sleep(1)

    print(f"[INFO] 任务 {task_id} 执行完成。\n")


def run_once(round_id: int, task_id: int):
    start_time = time.time()
    bag_filename = f"data_round_{round_id:07d}.bag"

    launch_file = f"load_kuavo_mujoco_sim{task_id}.launch"
    task_script = os.path.join(SCRIPT_DIR, f"task{task_id}.py")

    # 启动仿真环境
    launch_process = subprocess.Popen(
        ['roslaunch', 'data_challenge_simulator', launch_file],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    print(f"[INFO] Round {round_id}: Launched {launch_file}")
    time.sleep(3)

    # 在后台启动任务脚本但不等待完成
    print(f"[INFO] Round {round_id}: Starting {task_script} in background")
    task_process = subprocess.Popen(['python3', task_script])

    # # 等待关键话题准备就绪
    # try:
    #     rospy.init_node('data_collector', anonymous=True, disable_signals=True)
    # except:
    #     pass 
    
    # print(f"[INFO] Round {round_id}: 等待话题发布...")
    # wait_for_topics(timeout=15)

    # 启动 rosbag record
    bag_process = subprocess.Popen(
        ['rosbag', 'record', '-O', bag_filename] + ROS_TOPICS,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    print(f"[INFO] Round {round_id}: Recording → {bag_filename}")
    time.sleep(1)  # 让rosbag开始录制

    # 等待任务脚本完成
    print(f"[INFO] Round {round_id}: Waiting for task completion")
    task_process.wait()

    # 停止 rosbag 和 simulation
    print(f"[INFO] Round {round_id}: Stopping rosbag and simulation...")
    try:
        os.killpg(os.getpgid(bag_process.pid), signal.SIGINT)  # 优雅停止 rosbag
    except ProcessLookupError:
        pass
    time.sleep(2)  # 给rosbag更多时间完成写入
    try:
        os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    time.sleep(1)

    # 判断任务是否成功
    task_result_file = "task_result.txt"
    is_success = False

    if os.path.exists(task_result_file):
        with open(task_result_file, "r") as f:
            result = f.read().strip()
            is_success = result == "success"
        os.remove(task_result_file)
    else:
        print(f"[WARNING] Round {round_id}: 未生成任务结果文件，默认失败")

    if not is_success:
        print(f"[INFO] Round {round_id}: ❌ 任务失败，删除 {bag_filename}")
        if os.path.exists(bag_filename):
            os.remove(bag_filename)
    else:
        print(f"[INFO] Round {round_id}: ✅ 任务成功，保留 {bag_filename}")

    end_time = time.time()
    duration = end_time - start_time
    print(f"[{datetime.now()}] Round {round_id} finished in {duration:.2f} sec.\n")

    return duration, is_success


def main():
    print("========== 数据采集任务启动 ==========")
    print("请选择场景与任务编号（1-4）:")
    print("1: 任务1 —— 传送带物品分拣")
    print("2: 任务2 —— 传送带物品称重")
    print("3: 任务3 —— 物品翻面")
    print("4: 任务4 —— 货架物品运送")

    try:
        task_id = int(input("请输入任务编号 (1-4): ").strip())
        if task_id not in [1, 2, 3, 4]:
            raise ValueError()

        record_choice = int(input("是否录制 rosbag？1=是，0=否：").strip())
        if record_choice not in [0, 1]:
            raise ValueError()

        if record_choice == 1:
            repeat_times = int(input("请输入采集循环次数: ").strip())
            if repeat_times <= 0:
                raise ValueError()
    except ValueError:
        print("[ERROR] 输入无效，请输入有效数字。")
        return

    # 不录制模式
    if record_choice == 0:
        run_only_task(task_id)
        return

    # 录制模式
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    root_dir = os.path.join("bags", f"run_{timestamp}")
    os.makedirs(root_dir, exist_ok=True)
    os.chdir(root_dir)

    all_start_time = time.time()
    durations = []
    success_count = 0

    for i in range(repeat_times):
        print(f"\n====== 🚀 Round {i+1}/{repeat_times} ======")
        duration, is_success = run_once(i + 1, task_id)
        durations.append(duration)
        if is_success:
            success_count += 1

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
        f.write(f"🧾 总耗时: {total_time:.2f} 秒\n")

    print(open("result.txt").read())
    os.remove("result.txt")


if __name__ == '__main__':
    main()