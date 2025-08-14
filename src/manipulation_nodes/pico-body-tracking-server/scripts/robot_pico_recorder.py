#!/usr/bin/env python3
# coding: utf-8

import argparse
import subprocess
import sys
import os
import time
import signal
import json
from common.logger import SDKLogger
import rospy
from kuavo_msgs.srv import changeArmCtrlMode
from kuavo_msgs.msg import twoArmHandPoseCmd, footPoseTargetTrajectories
import threading
import rosbag
from std_msgs.msg import Int32


class RobotPicoRecorder:
    """Pico手臂/腿控bag录制与回放工具类"""
    
    def __init__(self):
        # 需要录制/回放的典型话题
        self.record_topics = [
            "/ik/two_arm_hand_pose_cmd",
            "/mm/two_arm_hand_pose_cmd",
            "/humanoid_mpc_foot_pose_world_target_trajectories",
            # 控制模式切换话题
            "/pico/arm_ctrl_mode_change",
            "/pico/mobile_ctrl_mode_change",
            "/pico/mm_wbc_arm_ctrl_mode_change"
        ]
        
        # 话题对应的消息类型
        self.topic_msg_types = {
            "/ik/two_arm_hand_pose_cmd": twoArmHandPoseCmd,
            "/mm/two_arm_hand_pose_cmd": twoArmHandPoseCmd,
            "/humanoid_mpc_foot_pose_world_target_trajectories": footPoseTargetTrajectories,
            # 控制模式切换话题的消息类型
            "/pico/arm_ctrl_mode_change": Int32,
            "/pico/mobile_ctrl_mode_change": Int32,
            "/pico/mm_wbc_arm_ctrl_mode_change": Int32
        }
        
        # 开始播放需要调用的控制服务列表
        self.control_services_enable = {
            "/change_arm_ctrl_mode" : 2,
            "/mobile_manipulator_mpc_control" : 1, 
            "/enable_mm_wbc_arm_trajectory_control" : 1
        }

        # 停止播放需要调用的控制服务列表
        self.control_services_disable = {
            "/change_arm_ctrl_mode" : 1,
            "/mobile_manipulator_mpc_control" : 0, 
            "/enable_mm_wbc_arm_trajectory_control" : 0
        }

        # 录制进程句柄（非阻塞）
        self.record_process = None
        
        # 播放相关变量
        self.play_thread = None
        self.stop_playback = False
        self.bag_data = {}  # 存储消息和时间戳
        self.publishers = {}

        # 从 config 目录读取额外要录制的话题
        self.extra_record_topics = self._load_extra_topics()
    
    def _get_default_extra_topics_path(self) -> str:
        """获取默认的额外话题配置文件路径。"""
        package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
        return os.path.join(package_root, "config", "extra_bag_topics.json")

    def _load_extra_topics(self):
        """从 JSON 文件加载额外话题，若不存在或格式错误则返回空列表。"""
        extra_path = self._get_default_extra_topics_path()
        if not os.path.exists(extra_path):
            SDKLogger.warning(f"Extra topics file not found: {extra_path}. Using no extra topics.")
            return []
        try:
            with open(extra_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            # 支持两种格式：直接数组 或 包含 'topics'/'extra_topics' 键的对象
            if isinstance(data, list):
                topics = data
            elif isinstance(data, dict):
                topics = data.get("extra_topics") or data.get("topics") or []
            else:
                topics = []
            # 仅保留字符串条目
            topics = [t for t in topics if isinstance(t, str) and t.strip()]
            # 去重并保持原有顺序
            seen = set()
            deduped = []
            for t in topics:
                if t not in seen:
                    seen.add(t)
                    deduped.append(t)
            if deduped:
                SDKLogger.info(f"Loaded extra topics from JSON: {deduped}")
            return deduped
        except Exception as e:
            SDKLogger.error(f"Failed to load extra topics: {e}")
            return []
    
    def _combine_topics(self):
        """合并默认话题与额外话题，去重保持顺序。"""
        combined = list(dict.fromkeys(self.record_topics + self.extra_record_topics))
        return combined
    
    def record_bag(self, bag_path):
        """以非阻塞方式启动录制bag文件"""
        if self.record_process is not None and self.record_process.poll() is None:
            SDKLogger.warning("Recording already in progress. Stop it before starting a new one.")
            return
        
        topics = self._combine_topics()
        cmd = ["rosbag", "record", "-O", bag_path] + topics
        SDKLogger.info(f"Recording bag to {bag_path} with topics: {topics}")
        SDKLogger.info(f"Command: {' '.join(cmd)}")
        try:
            # 使用新进程组以便发送SIGINT到整个组
            self.record_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
            SDKLogger.info(f"rosbag record started (pid={self.record_process.pid}) in background.")
        except Exception as e:
            SDKLogger.error(f"Error starting recording: {e}")
            self.record_process = None
    
    def stop_recording(self, timeout_seconds: float = 5.0):
        """停止录制，发送SIGINT，必要时再强制终止。"""
        if self.record_process is None or self.record_process.poll() is not None:
            SDKLogger.info("No active recording process to stop.")
            self.record_process = None
            return
        
        try:
            pgid = os.getpgid(self.record_process.pid)
            SDKLogger.info(f"Stopping rosbag record (pid={self.record_process.pid}, pgid={pgid}) with SIGINT...")
            os.killpg(pgid, signal.SIGINT)
            try:
                self.record_process.wait(timeout=timeout_seconds)
                SDKLogger.info("Recording stopped.")
            except subprocess.TimeoutExpired:
                SDKLogger.warning("Graceful stop timed out, sending SIGTERM...")
                os.killpg(pgid, signal.SIGTERM)
                try:
                    self.record_process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    SDKLogger.warning("Terminate timed out, sending SIGKILL...")
                    os.killpg(pgid, signal.SIGKILL)
                    self.record_process.wait()
        except Exception as e:
            SDKLogger.error(f"Failed to stop recording: {e}")
        finally:
            self.record_process = None
    
    def call_ctrl_services_disable(self):
        """调用控制服务"""
        for service_name, arg in self.control_services_disable.items():
            try:
                rospy.wait_for_service(service_name, timeout=5)
                proxy = rospy.ServiceProxy(service_name, changeArmCtrlMode)
                proxy(arg)
                SDKLogger.info(f"Called service {service_name} with arg {arg}.")
            except Exception as e:
                SDKLogger.error(f"Failed to call service {service_name}: {e}")
    
    def call_ctrl_services_enable(self):
        """调用控制服务"""
        for service_name, arg in self.control_services_enable.items():
            try:
                rospy.wait_for_service(service_name, timeout=5)
                proxy = rospy.ServiceProxy(service_name, changeArmCtrlMode)
                proxy(arg)
                SDKLogger.info(f"Called service {service_name} with arg {arg}.")
            except Exception as e:
                SDKLogger.error(f"Failed to call service {service_name}: {e}")

    def call_arm_ctrl_mode_service(self, mode: int):
        """调用手臂控制模式切换服务"""
        try:
            rospy.wait_for_service("/change_arm_ctrl_mode", timeout=5)
            proxy = rospy.ServiceProxy("/change_arm_ctrl_mode", changeArmCtrlMode)
            proxy(mode)
            SDKLogger.info(f"Playback: Called /change_arm_ctrl_mode service with mode {mode}")
        except Exception as e:
            SDKLogger.error(f"Playback: Failed to call /change_arm_ctrl_mode service: {e}")

    def call_mobile_ctrl_mode_service(self, mode: int):
        """调用移动控制模式切换服务"""
        try:
            rospy.wait_for_service("/mobile_manipulator_mpc_control", timeout=5)
            proxy = rospy.ServiceProxy("/mobile_manipulator_mpc_control", changeArmCtrlMode)
            proxy(mode)
            SDKLogger.info(f"Playback: Called /mobile_manipulator_mpc_control service with mode {mode}")
        except Exception as e:
            SDKLogger.error(f"Playback: Failed to call /mobile_manipulator_mpc_control service: {e}")

    def call_mm_wbc_arm_ctrl_mode_service(self, mode: int):
        """调用MM WBC手臂控制模式切换服务"""
        try:
            rospy.wait_for_service("/enable_mm_wbc_arm_trajectory_control", timeout=5)
            proxy = rospy.ServiceProxy("/enable_mm_wbc_arm_trajectory_control", changeArmCtrlMode)
            proxy(mode)
            SDKLogger.info(f"Playback: Called /enable_mm_wbc_arm_trajectory_control service with mode {mode}")
        except Exception as e:
            SDKLogger.error(f"Playback: Failed to call /enable_mm_wbc_arm_trajectory_control service: {e}")
    
    def _get_bag_info(self, bag_path):
        """获取bag文件的详细信息"""
        try:
            with rosbag.Bag(bag_path, 'r') as bag:
                bag_info = bag.get_type_and_topic_info()
                topics = bag_info[1]  # topic_info
                
                info = {
                    "duration": bag.get_end_time() - bag.get_start_time(),
                    "start_time": bag.get_start_time(),
                    "end_time": bag.get_end_time(),
                    "message_count": bag.get_message_count(),
                    "topics": {}
                }
                
                for topic_name, topic_info in topics.items():
                    info["topics"][topic_name] = {
                        "message_count": topic_info.message_count,
                        "frequency": topic_info.frequency,
                        "type": topic_info.msg_type
                    }
                
                return info
                    
        except Exception as e:
            SDKLogger.error(f"Failed to get bag info: {e}")
            return {"error": str(e)}

    def _load_bag_data(self, bag_path):
        """从bag文件加载数据到内存，包含时间戳信息"""
        try:
            SDKLogger.info(f"Loading bag data using rosbag package from {bag_path}...")
            
            # 首先获取bag信息
            bag_info = self._get_bag_info(bag_path)
            if "error" in bag_info:
                SDKLogger.warning(f"Could not get bag info: {bag_info['error']}")
            
            with rosbag.Bag(bag_path, 'r') as bag:
                # 为每个话题创建消息列表
                for topic in self.record_topics:
                    if topic in self.topic_msg_types:
                        self.bag_data[topic] = []
                        
                        # 读取该话题的所有消息和时间戳
                        message_count = 0
                        try:
                            for topic_name, msg, t in bag.read_messages(topics=[topic]):
                                try:
                                    # 存储消息和时间戳
                                    self.bag_data[topic].append({
                                        'message': msg,
                                        'timestamp': t.to_sec()  # 转换为秒
                                    })
                                    message_count += 1
                                    
                                    # 限制内存使用，如果消息太多则采样
                                    if message_count > 1000:
                                        SDKLogger.warning(f"Topic {topic} has too many messages, sampling every 10th message")
                                        # 保留前1000条消息，然后每10条保留1条
                                        if message_count % 10 == 0:
                                            self.bag_data[topic].append({
                                                'message': msg,
                                                'timestamp': t.to_sec()
                                            })
                                    
                                except Exception as e:
                                    SDKLogger.warning(f"Failed to process message from {topic}: {e}")
                                    continue
                            
                            # 按时间戳排序
                            self.bag_data[topic].sort(key=lambda x: x['timestamp'])
                            SDKLogger.info(f"Loaded {len(self.bag_data[topic])} messages from topic {topic}")
                            
                        except Exception as e:
                            SDKLogger.error(f"Failed to read messages from topic {topic}: {e}")
                        
                        # 如果没有消息，记录警告
                        if not self.bag_data[topic]:
                            SDKLogger.warning(f"No messages found for topic {topic}")
            
            # 检查是否成功加载了数据
            total_messages = sum(len(messages) for messages in self.bag_data.values())
            if total_messages == 0:
                SDKLogger.error("No messages loaded from any topic")
                return False
            
            SDKLogger.info(f"Successfully loaded {total_messages} total messages from bag file")
            return True
            
        except Exception as e:
            SDKLogger.error(f"Failed to load bag data: {e}")
            return False

    def _prepare_sequential_playback(self):
        """准备顺序播放：将所有消息按时间戳排序，预计算间隔"""
        all_messages = []
        
        # 收集所有话题的消息
        for topic, messages in self.bag_data.items():
            for msg_data in messages:
                all_messages.append({
                    'topic': topic,
                    'message': msg_data['message'],
                    'timestamp': msg_data['timestamp']
                })
        
        # 按时间戳排序
        all_messages.sort(key=lambda x: x['timestamp'])
        
        # 计算每个消息之间的时间间隔
        playback_sequence = []
        intervals = []
        for i, msg_data in enumerate(all_messages):
            if i == 0:
                # 第一条消息，间隔为0
                interval = 0.0
            else:
                # 计算与前一条消息的时间间隔
                interval = msg_data['timestamp'] - all_messages[i-1]['timestamp']
                # 确保间隔不为负数
                if interval < 0:
                    interval = 0.001  # 最小间隔1ms
                intervals.append(interval)
            
            playback_sequence.append({
                'topic': msg_data['topic'],
                'message': msg_data['message'],
                'timestamp': msg_data['timestamp'],
                'interval': interval
            })
        
        # 显示间隔统计信息
        if intervals:
            SDKLogger.info(f"Interval statistics:")
            SDKLogger.info(f"  Total intervals: {len(intervals)}")
            SDKLogger.info(f"  Average interval: {sum(intervals)/len(intervals):.6f}s")
            SDKLogger.info(f"  Min interval: {min(intervals):.6f}s")
            SDKLogger.info(f"  Max interval: {max(intervals):.6f}s")
            SDKLogger.info(f"  First few intervals: {intervals[:5]}")
        
        SDKLogger.info(f"Prepared {len(playback_sequence)} messages for sequential playback")
        SDKLogger.info(f"Total duration: {playback_sequence[-1]['timestamp'] - playback_sequence[0]['timestamp']:.3f}s")
        
        return playback_sequence

    def _setup_publishers(self):
        """设置发布者"""
        for topic, msg_type in self.topic_msg_types.items():
            try:
                self.publishers[topic] = rospy.Publisher(topic, msg_type, queue_size=10)
                rospy.sleep(0.1)  # 等待发布者初始化
                SDKLogger.info(f"Publisher set up for {topic}")
            except Exception as e:
                SDKLogger.error(f"Failed to set up publisher for {topic}: {e}")

    def get_playback_status(self):
        """获取播放状态信息"""
        status = {
            "is_playing": self.play_thread is not None and self.play_thread.is_alive(),
            "stop_requested": self.stop_playback,
            "topics": {}
        }
        
        for topic, messages in self.bag_data.items():
            status["topics"][topic] = {
                "total_messages": len(messages),
                "publisher_ready": topic in self.publishers
            }
        
        return status

    def _playback_loop(self, loop=False):
        """播放循环，按实际时间间隔顺序播放"""
        if not self.bag_data:
            SDKLogger.error("No bag data loaded")
            return
        
        try:
            iteration = 0
            while not self.stop_playback and not rospy.is_shutdown():
                iteration += 1
                SDKLogger.info(f"Playback iteration {iteration}")
                
                # 计算播放时间
                start_time = time.time()
                
                # 准备顺序播放序列
                playback_sequence = self._prepare_sequential_playback()
                
                if not playback_sequence:
                    SDKLogger.error("No messages to play")
                    return
                
                SDKLogger.info(f"Starting sequential playback of {len(playback_sequence)} messages")
                
                # 按顺序播放所有消息
                for i, msg_data in enumerate(playback_sequence):
                    if self.stop_playback:
                        break
                    
                    topic = msg_data['topic']
                    message = msg_data['message']
                    interval = msg_data['interval']
                    
                    # 检查发布者是否可用
                    if topic not in self.publishers:
                        SDKLogger.warning(f"Publisher not available for topic {topic}, skipping message")
                        continue
                    
                    try:
                        
                        # 如果是控制模式切换话题，调用对应的服务
                        if topic == "/pico/arm_ctrl_mode_change":
                            self.call_arm_ctrl_mode_service(message.data)
                        elif topic == "/pico/mobile_ctrl_mode_change":
                            self.call_mobile_ctrl_mode_service(message.data)
                        elif topic == "/pico/mm_wbc_arm_ctrl_mode_change":
                            self.call_mm_wbc_arm_ctrl_mode_service(message.data)
                        else:
                            # 发布消息
                            self.publishers[topic].publish(message)
                        
                        # 每10条消息显示一次进度
                        if (i + 1) % 50 == 0:
                            progress = (i + 1) / len(playback_sequence) * 100
                            SDKLogger.info(f"Playback progress: {progress:.1f}% ({i + 1}/{len(playback_sequence)}) - Topic: {topic}")
                        
                        # 控制发布间隔
                        if i > 0:  # 第一条消息不需要等待
                            if interval > 0:
                                # SDKLogger.debug(f"Sleeping for {interval:.6f}s (actual bag interval)")
                                rospy.sleep(interval)
                            else:
                                # 如果间隔为0，使用最小间隔
                                # SDKLogger.debug(f"Interval was {interval:.6f}s, using minimum 0.001s")
                                rospy.sleep(0.001)  # 1ms
                                
                    except Exception as e:
                        SDKLogger.warning(f"Failed to publish message {i} to topic {topic}: {e}")
                
                # 计算本次播放耗时
                playback_time = time.time() - start_time
                SDKLogger.info(f"Playback iteration {iteration} completed in {playback_time:.2f} seconds")
                
                if not loop:
                    break
                    
                if not self.stop_playback:
                    SDKLogger.info("Playback completed, restarting..." if loop else "Playback completed")
                    
        except Exception as e:
            SDKLogger.error(f"Error in playback loop: {e}")

    def play_bag(self, bag_path, loop=False):
        """回放bag文件（非阻塞循环发送）"""
        rospy.init_node("robot_pico_recorder_play", anonymous=True)
        
        if not os.path.exists(bag_path):
            SDKLogger.error(f"Bag file {bag_path} does not exist!")
            sys.exit(1)
        
        # 显示bag文件信息
        bag_info = self._get_bag_info(bag_path)
        if "error" not in bag_info:
            if "duration" in bag_info:
                SDKLogger.info(f"Bag duration: {bag_info['duration']:.2f} seconds")
                SDKLogger.info(f"Total messages: {bag_info['message_count']}")
            SDKLogger.info(f"Topics: {list(bag_info.get('topics', {}).keys())}")
        
        # 加载bag数据
        if not self._load_bag_data(bag_path):
            SDKLogger.error("Failed to load bag data")
            return
        
        # 设置发布者
        self._setup_publishers()
        
        # 调用控制服务启用
        self.call_ctrl_services_enable()
        
        # 启动播放线程
        self.stop_playback = False
        self.play_thread = threading.Thread(target=self._playback_loop, args=(loop,))
        self.play_thread.daemon = True
        self.play_thread.start()
        
        SDKLogger.info(f"Started non-blocking playback of {bag_path} {'in loop' if loop else ''}")
        SDKLogger.info("Press Ctrl+C to stop playback")
        
        try:
            # 主线程等待播放完成或中断，同时显示状态信息
            last_status_time = time.time()
            while self.play_thread.is_alive() and not rospy.is_shutdown():
                rospy.sleep(0.1)
                
                # 每5秒显示一次播放状态
                current_time = time.time()
                if current_time - last_status_time > 5.0:
                    status = self.get_playback_status()
                    SDKLogger.info(f"Playback status: {status['is_playing']}, Topics: {len(status['topics'])}")
                    last_status_time = current_time
                    
        except KeyboardInterrupt:
            SDKLogger.info("Playback interrupted by user.")
        finally:
            # 停止播放
            self.stop_playback = True
            if self.play_thread and self.play_thread.is_alive():
                self.play_thread.join(timeout=2.0)
            
            # 调用控制服务禁用
            self.call_ctrl_services_disable()
            SDKLogger.info("Playback stopped")
    
    def run(self, mode, bag_path, loop=False):
        """运行录制或回放功能"""
        if mode == "record":
            self.record_bag(bag_path)
        elif mode == "play":
            self.play_bag(bag_path, loop)
        else:
            raise ValueError(f"Unknown mode: {mode}")


def main():
    parser = argparse.ArgumentParser(description="Pico手臂/腿控bag录制与回放工具")
    subparsers = parser.add_subparsers(dest="mode", help="模式: record 或 play")

    # 录制子命令
    parser_record = subparsers.add_parser("record", help="录制bag")
    parser_record.add_argument("bag", type=str, help="保存的bag文件名")

    # 回放子命令
    parser_play = subparsers.add_parser("play", help="回放bag")
    parser_play.add_argument("bag", type=str, help="要回放的bag文件名")
    parser_play.add_argument("--loop", "-l", action="store_true", help="循环回放")

    args = parser.parse_args()

    if not args.mode:
        parser.print_help()
        sys.exit(1)

    # 根据模式分别处理，record 模式下捕获 Ctrl+C 并调用 stop_recording
    recorder = RobotPicoRecorder()
    if args.mode == "record":
        try:
            recorder.record_bag(args.bag)
            SDKLogger.info("Recording started. Press Ctrl+C to stop.")
            # 等待直到进程结束或收到 Ctrl+C
            while True:
                time.sleep(0.5)
                if recorder.record_process is None:
                    break
                if recorder.record_process.poll() is not None:
                    SDKLogger.info("rosbag record exited.")
                    break
        except KeyboardInterrupt:
            SDKLogger.info("Ctrl+C detected. Stopping recording...")
        finally:
            recorder.stop_recording()
    else:
        try:
            recorder.play_bag(args.bag, getattr(args, 'loop', False))
        except Exception as e:
            SDKLogger.error(f"Error: {e}")
            sys.exit(1)


if __name__ == "__main__":
    main()
