#!/usr/bin/env python3
"""
仿真环境自动启动器
选手脚本通过此模块自动拉起仿真环境、设置随机初始位姿。

使用方法（在选手脚本中）：
    from sim_launcher import SimLauncher

    launcher = SimLauncher(scene="scene1", seed=42)
    launcher.start()   # 启动仿真 + 初始化ROS节点 + 等待就绪
    # ... 选手任务逻辑 ...
    launcher.stop()    # 结束时关闭仿真
"""

import atexit
import math
import os
import signal
import subprocess
import sys
import time

# craic_secret.so 位于 ../lib/ 目录
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))

try:
    from craic_secret import get_random_init_state, verify_scene_files
except ImportError:
    print("[FATAL] sim_launcher: 无法导入 craic_secret 模块（.so 文件缺失）")
    sys.exit(1)


class SimLauncher:
    """仿真环境自动启动器"""

    def __init__(self, scene, seed, robot_version=None):
        """
        Args:
            scene: "scene1" 或 "scene2"
            seed: 随机种子（控制机器人初始位姿）
            robot_version: 机器人版本号（默认从环境变量 ROBOT_VERSION 读取，未设置则为 47）
        """
        self.scene = scene
        self.seed = seed
        self.robot_version = robot_version or int(os.environ.get("ROBOT_VERSION", 47))
        self._launch_proc = None

    def start(self, node_name="sim_task", timeout=120):
        """
        启动仿真环境并等待就绪。

        流程：
        1. 校验场景文件完整性（XML 不被修改，哈希固定）
        2. 生成随机初始位姿
        3. 启动 roslaunch（自带 roscore）
        4. 初始化 ROS 节点
        5. 等待仿真就绪
        6. 通过 set_object_position 服务随机化物体位置

        Args:
            node_name: ROS 节点名称
            timeout: 等待仿真就绪的超时时间（秒）
        """
        # 校验场景文件完整性（XML 不会被修改，哈希固定）
        pkg_path = os.path.join(os.path.dirname(__file__), '..')
        passed, messages = verify_scene_files(os.path.abspath(pkg_path), self.robot_version)
        for msg in messages:
            print(f"[{'INFO' if passed else 'ERROR'}] sim_launcher: {msg}")
        if not passed:
            print("[FATAL] sim_launcher: 场景文件校验失败，禁止启动仿真！")
            sys.exit(1)
        print("[INFO] sim_launcher: 场景文件校验通过。")

        # 生成随机初始位姿（不需要 ROS）
        init_state = get_random_init_state(self.scene, self.robot_version, self.seed)
        yaw_deg = self._quat_to_yaw_deg(init_state[3:7])
        print(f"[INFO] sim_launcher: 随机初始位姿 (seed={self.seed}): "
              f"x={init_state[0]:.2f}, y={init_state[1]:.2f}, yaw={yaw_deg:.2f}°")

        # 将初始位姿转为 YAML 数组字符串，通过 launch arg 传入
        init_state_str = "[" + ", ".join(f"{v:.6f}" for v in init_state) + "]"

        # 启动 roslaunch（会自动启动 roscore）
        cmd = [
            "roslaunch", "craic_simulator", "load_kuavo_mujoco_craic.launch",
            f"robot_version:={self.robot_version}",
            f"init_state:={init_state_str}",
        ]
        print(f"[INFO] sim_launcher: 启动仿真: {' '.join(cmd)}")
        self._launch_proc = subprocess.Popen(
            cmd,
            # stdout=subprocess.PIPE,
            # stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        # 注册退出清理，确保 Ctrl+C 或异常退出时关闭仿真
        atexit.register(self.stop)

        # 等待 roscore 就绪后初始化 ROS 节点
        self._wait_for_roscore(timeout=30)
        import rospy
        rospy.init_node(node_name, anonymous=True)

        # 等待仿真就绪（通过检查关键 topic 是否发布）
        rospy.loginfo("sim_launcher: 等待仿真就绪...")
        self._wait_for_sim(timeout)
        rospy.loginfo("sim_launcher: 仿真环境就绪。")

        # 仿真就绪后，通过 ROS 服务随机化物体位置
        self._randomize_object_positions(rospy)

    def _randomize_object_positions(self, rospy):
        """仿真启动后，通过 set_object_position 服务随机化物体位置"""
        import random
        import yaml
        from kuavo_msgs.srv import SetObjectPosition
        from std_srvs.srv import SetBool
        from geometry_msgs.msg import Point, Quaternion

        try:
            rospy.wait_for_service('set_object_position', timeout=10)
            rospy.wait_for_service('sim_start', timeout=5)
        except rospy.ROSException:
            rospy.logwarn("sim_launcher: set_object_position/sim_start 服务不可用，跳过物体随机化")
            return

        # 暂停仿真，避免物体移动时被看到瞬移
        sim_ctrl = rospy.ServiceProxy('sim_start', SetBool)
        sim_ctrl(False)
        rospy.loginfo("sim_launcher: 暂停仿真，开始随机化物体位置...")

        set_pos = rospy.ServiceProxy('set_object_position', SetObjectPosition)

        # 读取随机化配置
        pkg_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        config_file = os.path.join(pkg_path, 'config', 'craic_room.yaml')
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        rand_cfg = config.get('randomization')
        if not rand_cfg:
            sim_ctrl(True)
            return

        rng = random.Random(self.seed)

        # ---- 1. 范围随机物体（如路障锥）：均匀分布 + 最小间距 ----
        random_objects = rand_cfg.get('random_objects', {})
        for group_name, group_cfg in random_objects.items():
            names = group_cfg.get('names', [])
            x_min, x_max = group_cfg.get('x_min', -5.8), group_cfg.get('x_max', 3.0)
            y_min, y_max = group_cfg.get('y_min', 1.8), group_cfg.get('y_max', 4.2)
            z = group_cfg.get('z', 0.0)
            min_dist = group_cfg.get('min_distance', 1.0)

            # 从 objects 配置中获取物体尺寸（直径），用于间距计算
            obj_radius = 0.0
            for obj in config.get('objects', []):
                if obj.get('name') == names[0] if names else '':
                    target_size = obj.get('target_size', '0.33 0.6')
                    obj_radius = float(target_size.split()[0]) / 2.0
                    break

            # 实际最小间距 = 配置间距 + 物体直径（避免重叠）
            effective_min_dist = min_dist + obj_radius * 2

            # 生成随机位置，保证最小间距
            placed = []
            for name in names:
                for attempt in range(100):
                    x = rng.uniform(x_min, x_max)
                    y = rng.uniform(y_min, y_max)
                    # 检查与已放置物体的间距
                    too_close = False
                    for px, py in placed:
                        dist = math.sqrt((x - px) ** 2 + (y - py) ** 2)
                        if dist < effective_min_dist:
                            too_close = True
                            break
                    if not too_close:
                        placed.append((x, y))
                        break
                else:
                    # 100次尝试都失败，放宽间距
                    x = rng.uniform(x_min, x_max)
                    y = rng.uniform(y_min, y_max)
                    placed.append((x, y))
                    rospy.logwarn(f"  {name}: 无法满足最小间距 {effective_min_dist:.2f}m，强制放置")

                try:
                    resp = set_pos(
                        object_name=name,
                        position=Point(x=x, y=y, z=z),
                        orientation=Quaternion(x=0, y=0, z=0, w=0),
                        randomize=False,
                        x_min=0, x_max=0, y_min=0, y_max=0, z_min=0, z_max=0,
                    )
                    if resp.success:
                        rospy.loginfo(f"  {name} -> ({x:.2f}, {y:.2f}, {z:.2f})")
                    else:
                        rospy.logwarn(f"  {name} 移动失败: {resp.message}")
                except Exception as e:
                    rospy.logwarn(f"  {name} 服务调用失败: {e}")

            rospy.loginfo(f"  {group_name}: {len(names)} 个物体已随机放置 (间距>={effective_min_dist:.2f}m)")

        # ---- 2. 预设位置物体（料盘、零件等） ----
        presets = rand_cfg.get('presets', [])
        if presets:
            preset_idx = self.seed % len(presets)
            preset = presets[preset_idx]
            rospy.loginfo(f"sim_launcher: 应用预设位置 (preset={preset_idx})")

            for obj_name, overrides in preset.items():
                pos_str = overrides.get('pos', '0 0 0')
                parts = pos_str.split()
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])

                quat = Quaternion(x=0, y=0, z=0, w=0)
                euler_str = overrides.get('euler')
                # 如果 preset 没有指定 euler，从 objects 默认配置中读取
                if not euler_str:
                    for obj in config.get('objects', []):
                        if obj.get('name') == obj_name:
                            euler_str = obj.get('euler')
                            break
                if euler_str:
                    ep = euler_str.split()
                    roll, pitch, yaw = float(ep[0]), float(ep[1]), float(ep[2])
                    cr, sr = math.cos(roll/2), math.sin(roll/2)
                    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
                    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
                    quat = Quaternion(
                        w=cr*cp*cy + sr*sp*sy,
                        x=sr*cp*cy - cr*sp*sy,
                        y=cr*sp*cy + sr*cp*sy,
                        z=cr*cp*sy - sr*sp*cy,
                    )

                try:
                    resp = set_pos(
                        object_name=obj_name,
                        position=Point(x=x, y=y, z=z),
                        orientation=quat,
                        randomize=False,
                        x_min=0, x_max=0, y_min=0, y_max=0, z_min=0, z_max=0,
                    )
                    if resp.success:
                        rospy.loginfo(f"  {obj_name} -> ({x:.2f}, {y:.2f}, {z:.2f})")
                    else:
                        rospy.logwarn(f"  {obj_name} 移动失败: {resp.message}")
                except Exception as e:
                    rospy.logwarn(f"  {obj_name} 服务调用失败: {e}")

        # ---- 3. 零件位置随机交换 + 微偏移 ----
        shuffle_cfg = rand_cfg.get('shuffleable_parts', {})
        shuffle_names = shuffle_cfg.get('names', [])
        xy_offset = shuffle_cfg.get('xy_offset', 0.02)

        if len(shuffle_names) >= 2:
            # 从 objects 配置中读取默认位置
            positions = []
            for name in shuffle_names:
                for obj in config.get('objects', []):
                    if obj.get('name') == name:
                        positions.append({'pos': obj.get('pos', '0 0 0'), 'euler': obj.get('euler')})
                        break

            # 随机打乱位置
            rng.shuffle(positions)

            for i, name in enumerate(shuffle_names):
                if i >= len(positions):
                    break
                pos_str = positions[i].get('pos', '0 0 0')
                parts = pos_str.split()
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])

                # 添加 ±offset 微偏移（仅 X/Y）
                x += rng.uniform(-xy_offset, xy_offset)
                y += rng.uniform(-xy_offset, xy_offset)

                quat = Quaternion(x=0, y=0, z=0, w=0)
                euler_str = positions[i].get('euler')
                if euler_str:
                    ep = euler_str.split()
                    roll, pitch, yaw = float(ep[0]), float(ep[1]), float(ep[2])
                    cr, sr = math.cos(roll/2), math.sin(roll/2)
                    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
                    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
                    quat = Quaternion(
                        w=cr*cp*cy + sr*sp*sy,
                        x=sr*cp*cy - cr*sp*sy,
                        y=cr*sp*cy + sr*cp*sy,
                        z=cr*cp*sy - sr*sp*cy,
                    )

                try:
                    resp = set_pos(
                        object_name=name,
                        position=Point(x=x, y=y, z=z),
                        orientation=quat,
                        randomize=False,
                        x_min=0, x_max=0, y_min=0, y_max=0, z_min=0, z_max=0,
                    )
                    if resp.success:
                        rospy.loginfo(f"  shuffle: {name} -> ({x:.3f}, {y:.3f}, {z:.2f})")
                except Exception as e:
                    rospy.logwarn(f"  shuffle {name} 失败: {e}")

        # ---- 4. 按钮亮暗随机化 ----
        button_groups = rand_cfg.get('button_groups', [])
        if button_groups:
            from kuavo_msgs.srv import SetButtonState
            try:
                rospy.wait_for_service('set_button_state', timeout=5)
                set_btn = rospy.ServiceProxy('set_button_state', SetButtonState)
                for group in button_groups:
                    if len(group) < 2:
                        continue
                    # 随机选一个亮
                    bright_idx = rng.randint(0, len(group) - 1)
                    for i, btn_name in enumerate(group):
                        is_bright = (i == bright_idx)
                        try:
                            resp = set_btn(button_name=btn_name, is_bright=is_bright)
                            if resp.success:
                                rospy.loginfo(f"  {btn_name}: {'亮' if is_bright else '暗'}")
                            else:
                                rospy.logwarn(f"  {btn_name} 设置失败: {resp.message}")
                        except Exception as e:
                            rospy.logwarn(f"  {btn_name} 服务调用失败: {e}")
            except rospy.ROSException:
                rospy.logwarn("sim_launcher: set_button_state 服务不可用，跳过按钮随机化")

        # 恢复仿真
        sim_ctrl(True)
        rospy.loginfo("sim_launcher: 物体位置随机化完成，仿真已恢复。")

        # 通知监测模块：初始化完成，开始监测
        from std_msgs.msg import String
        init_done_pub = rospy.Publisher('/cheat_monitor/init_done', String, queue_size=1, latch=True)
        init_done_pub.publish(String(data="init_done"))
        rospy.loginfo("sim_launcher: 初始化完成，已开始监测。")

    def stop(self):
        """关闭仿真环境"""
        if self._launch_proc is not None:
            print("[INFO] sim_launcher: 关闭仿真环境...")
            try:
                os.killpg(os.getpgid(self._launch_proc.pid), signal.SIGTERM)
                self._launch_proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._launch_proc.pid), signal.SIGKILL)
                self._launch_proc.wait(timeout=5)
            except Exception:
                pass
            self._launch_proc = None

    def _wait_for_roscore(self, timeout):
        """等待 roscore 就绪"""
        import xmlrpc.client
        master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        start = time.time()
        while time.time() - start < timeout:
            try:
                master = xmlrpc.client.ServerProxy(master_uri)
                master.getSystemState("")
                return
            except Exception:
                time.sleep(0.5)
        print(f"[WARN] sim_launcher: 等待 roscore 超时 ({timeout}s)")

    def _wait_for_sim(self, timeout):
        """等待仿真关键节点就绪"""
        import rospy
        start = time.time()
        while time.time() - start < timeout:
            if rospy.is_shutdown():
                return
            try:
                topics = [t[0] for t in rospy.get_published_topics()]
                if "/sensors_data_raw" in topics:
                    return
            except Exception:
                pass
            time.sleep(1.0)
        rospy.logwarn("sim_launcher: 等待仿真就绪超时 (%ds)，继续执行...", timeout)

    @staticmethod
    def _quat_to_yaw_deg(quat):
        """四元数 [qw, qx, qy, qz] → yaw 角度"""
        qw, qx, qy, qz = quat
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw_rad)
