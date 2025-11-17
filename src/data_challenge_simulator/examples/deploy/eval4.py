import os, sys, time, math, threading, traceback

current_dir = os.path.dirname(os.path.abspath(__file__))
data_challenge_simulator_dir = os.path.dirname(current_dir)
if data_challenge_simulator_dir not in sys.path:
    sys.path.insert(0, data_challenge_simulator_dir)

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
from utils.gripper_controller import GripperController
from utils.conveyor_controller import ConveyorController
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.trajectory_controller import TrajectoryController
from utils.utils import Utils
from utils.evaluator import ScoringEvaluator4, ScoringConfig4
import rospy
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import numpy as np
import json

class SimulatorTask4():
    def __init__(self,seed):
        rospy.init_node('simulator_task4', anonymous=False)

        self.init_service = rospy.ServiceProxy('/simulator/init', Trigger)
        self.pub_success = rospy.Publisher('/simulator/success',Bool, queue_size=10)
        # 等待外部信号
        self.start_service = rospy.Service('/simulator/start', Trigger, self._on_start_service)
        self.reset_service = rospy.Service('/simulator/reset', Trigger, self._on_reset_service)

        # 事件控制
        self.start_evt = threading.Event()
        self.reset_evt = threading.Event()

        # 设备与控制器
        self.robot = None
        self.robot_state = None
        self.gripper_ctrl = None
        self.traj_ctrl = None
        self.pose_ctrl = None
        self.obj_pos = None
        self.score = 0
        default_score_file = "/tmp/simulator_score_last.txt"
        self.score_file = os.environ.get("SCORE_FILE", default_score_file)
        # 成功状态
        self.started = False
        self.already_reported_success = False

        self.seed = seed
        self.obj_pos = ObjectPose()
        self.obj_pos_set = ObjectRandomizer()

        # 记录分项是否达成
        self.comp_item1_pos = False
        self.comp_item2_pos = False
        self.comp_robot_pos1 = False
        self.comp_robot_pos2 = False
        self.comp_robot_pos3 = False
        self.comp_robot_pos4 = False
        self.comp_time_score = 0


    # ========== 服务回调 - 等待外部信号 ==========
    def _on_start_service(self, req):
        """等待外部代码发送 start 信号"""
        rospy.loginfo("[sim] 收到外部 start 信号，开始执行任务")
        self.started = True
        self.start_evt.set()
        return TriggerResponse(success=True, message="Task started successfully")

    def _on_reset_service(self, req):
        """等待外部代码发送 reset 信号"""
        rospy.loginfo("[sim] 收到外部 reset 信号，准备重置任务")
        self.reset_evt.set()
        return TriggerResponse(success=True, message="Task reset triggered")

    def _sample_position_with_seed(self,seed: int, region: dict):
        """
        使用固定seed采样单个物体的位置
        region: {"x": (xmin, xmax), "y": (ymin, ymax), "z": (zmin, zmax)}
        """
        rng = np.random.default_rng(int(seed))
        x = float(rng.uniform(*region["x"]))
        y = float(rng.uniform(*region["y"]))
        z = float(rng.uniform(*region["z"]))
        return x, y, z
    
    def _randomize_objects_with_seed(self,seed: int, regions: dict, bin_ori: list, item_ori: list, bin_list: list):
        """
        使用固定seed随机化多个物体的位置和朝向
        regions: dict
        FRONT/BACK: 四元数 [w, x, y, z]
        """

        object_configs = []
        for name, region in regions.items():
            x, y, z = self._sample_position_with_seed(seed, region)
            if name in bin_list:
                qw, qx, qy, qz = bin_ori
            else:
                qw, qx, qy, qz = item_ori

            object_configs.append({
                "name": name,
                "position_ranges": {
                    "x": [x,x],
                    "y": [y,y],
                    "z": [z,z],
                },
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}
            })
            seed+=1000


        # 应用到仿真
        self.obj_pos_set.randomize_multiple_objects(object_configs)
    
    # ========== 主流程 ==========
    def run(self):
        try:
            self.reset_evt.clear()
            self.start_evt.clear()

            # 1) 初始化 SDK 与控制器
            if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
                print("Init KuavoSDK failed, exit!")
                return  # 直接退出，deploy.py 会重启一轮

            self.robot = KuavoRobot()
            self.robot_state = KuavoRobotState()

            REGIONS = {
                "item1": {"x": (1.28, 1.38), "y": (-0.6, -0.1), "z": (0.9, 0.9)},  # 区域 A
                "item2": {"x": (1.28, 1.38), "y": (0.1, 0.6), "z": (0.9, 0.9)},  # 区域 B
                "left_bin_A": {"x": (-1.35, -1.35), "y": (0.15, 0.6), "z": (0.76, 0.76)},  # 区域 C
                "left_bin_B": {"x": (-1.35, -1.35), "y": (-0.6, -0.15), "z": (0.76, 0.76)},  # 区域 C
            }

            bin_ori = [0.7071, 0, 0, 0.7071]  # (w, x, y, z)
            item_ori = [1, 0, 0, 0]
            bin_list = ["left_bin_A", "left_bin_B"]
            # 随机化物体位置
            self._randomize_objects_with_seed(seed=self.seed, regions=REGIONS, bin_ori=bin_ori, item_ori=item_ori, bin_list=bin_list)

            self.item1_pos = self.obj_pos.wait_for_position("item1",timeout=5.0)
            self.item2_pos = self.obj_pos.wait_for_position("item2",timeout=5.0)
            self.left_bin_A_pos = self.obj_pos.wait_for_position("left_bin_A", timeout=5.0)
            self.left_bin_B_pos = self.obj_pos.wait_for_position("left_bin_B", timeout=5.0)

            self.target_region1 = [
                (self.left_bin_A_pos[0]-0.14, self.left_bin_A_pos[0]+0.14),   # x 范围
                (self.left_bin_A_pos[1]-0.14, self.left_bin_A_pos[1]+0.14),   # y 范围
                (0.84, 0.865)  # z 范围
            ]

            self.target_region2 = [
                (self.left_bin_B_pos[0]-0.14, self.left_bin_B_pos[0]+0.14),   # x 范围
                (self.left_bin_B_pos[1]-0.14, self.left_bin_B_pos[1]+0.14),   # y 范围
                (0.84, 0.865)  # z 范围
            ]

            self.robot_target_region1 = [
            (0.83, 0.87),                                                   # x 范围
            (self.item1_pos[1]+0.30-0.075, self.item1_pos[1]+0.30+0.075),   # y 范围
            (0.82, 0.83)                                                    # z 范围
            ]
            self.robot_target_region2 = [
            (-0.84, -0.8),   # x 范围
            (self.left_bin_A_pos[1]-0.30-0.075, self.left_bin_A_pos[1]-0.30+0.075),   # y 范围
            (0.82, 0.83)  # z 范围
            ]
            self.robot_target_region3 = [
            (0.83, 0.87),   # x 范围
            (self.item2_pos[1]-0.30-0.075, self.item2_pos[1]-0.30+0.075),   # y 范围
            (0.82, 0.83)  # z 范围
            ]
            self.robot_target_region4 = [
            (-0.84, -0.8),   # x 范围
            (self.left_bin_B_pos[1]+0.30-0.075, self.left_bin_B_pos[1]+0.30+0.075),   # y 范围
            (0.82, 0.83)  # z 范围
            ]

            self.evaluator = ScoringEvaluator4(
                ScoringConfig4(
                    target_region1=self.target_region1,
                    target_region2=self.target_region2,

                    robot_target_region1=self.robot_target_region1,
                    robot_target_region2=self.robot_target_region2,
                    robot_target_region3=self.robot_target_region3,
                    robot_target_region4=self.robot_target_region4,

                    time_full=10,
                    time_threshold_sec=65,
                    time_penalty_per_sec=1,

                    x_L_in=0.4,
                    x_L_out=0.2,
                    x_R_in=-0.4,
                    x_R_out=-0.2

                ),
                is_in_region_fn=lambda pos, region: Utils.is_in_target_region(pos, region),
                )
            # 2) 预抓位
            self.robot.control_head(yaw=0, pitch=math.radians(12))
            self.robot.set_external_control_arm_mode()
            # 3) 发布 msg0：init=True
            rospy.wait_for_service('/simulator/init')
            try:
                rospy.loginfo("[sim] 发布 /simulator/init 服务完成")
                resp = self.init_service(TriggerRequest())
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")


            # 4) 等待外部代码调用 start 服务
            rospy.loginfo("[sim] 等待外部代码调用 /simulator/start 服务...")
            while not rospy.is_shutdown() and not self.reset_evt.is_set():
                if self.start_evt.wait(timeout=0.1):
                    break

            if self.reset_evt.is_set() or rospy.is_shutdown():
                rospy.logwarn("[sim] 等待 start 期间收到 reset / shutdown，退出")
                self._cleanup()
                sys.exit(0)

            # 5) 收到外部 start 信号 → 开传送带，循环上报 success
            rospy.loginfo("[sim] 收到外部 start 信号，开始传送带，持续上报 success")

            rate = rospy.Rate(10)  # 10Hz 上报
            self.already_reported_success = False
            # start_time = time.time()  # 循环开始前计时
            self.evaluator.reset()
            
            while not rospy.is_shutdown() and not self.reset_evt.is_set():
                try:
                    item1_pos = self.obj_pos.get_position("item1")
                    item2_pos = self.obj_pos.get_position("item2")
                    robot_pos = self.robot_state.odometry.position

                except Exception as e:
                    rospy.logwarn(f"[sim] 获取位置出错：{e}")
                    item1_pos, item2_pos,robot_pos = None, None, None

                if item1_pos is None or item2_pos is None or robot_pos is None:
                    # 取不到传感就当未成功
                    # 未成功阶段持续发 False
                    self.pub_success.publish(Bool(data=False))
                    # 持续发布分数
                    self.pub_score.publish(Int32(data=self.evaluator.score))
                    rate.sleep()
                    continue

                # 调用通用评估器
                out = self.evaluator.evaluate(pos_xyz_item1= item1_pos,pos_xyz_item2=item2_pos, robot_pos= robot_pos, now=time.time())

                if out.get("item1_triggered"): self.comp_item1_pos = True
                if out.get("item2_triggered"): self.comp_item2_pos = True
                if out.get("robot_pos1_triggered"): self.comp_robot_pos1 = True
                if out.get("robot_pos2_triggered"): self.comp_robot_pos2 = True
                if out.get("robot_pos3_triggered"): self.comp_robot_pos3 = True
                if out.get("robot_pos4_triggered"): self.comp_robot_pos4 = True

                ts_add = out.get("time_score_added")
                if isinstance(ts_add, (int, float)):
                    self.comp_time_score += float(ts_add)

                # 按评估器的“建议标志”做 ROS 行为
                if out["need_publish_success_true"]:
                    rospy.loginfo("[sim] ✅ 任务成功，发布 /simulator/success=True")
                    self.pub_success.publish(Bool(data=True))

                elif out["need_publish_success_false"]:
                    self.pub_success.publish(Bool(data=False))

                if out["item1_pos_added"]:
                    parts = []
                    parts.append("+25(物料1)")
                    rospy.loginfo(f"[sim] 🟡 物料1放置成功：{' '.join(parts)}，总分 {out['total_score']}")

                if out["item2_pos_added"]:
                    parts = []
                    parts.append("+25(物料2)")
                    rospy.loginfo(f"[sim] 🟡 物料2放置成功：{' '.join(parts)}，总分 {out['total_score']}")

                if out["robot_pos1_triggered"]:
                    parts = []
                    parts.append("+10(抓取位置1)")
                    rospy.loginfo(f"[sim] 🟡 抓取位置1到达：{' '.join(parts)}，总分 {out['total_score']}")

                if out["robot_pos2_triggered"]:
                    parts = []
                    parts.append("+10(放置位置1)")
                    rospy.loginfo(f"[sim] 🟡 放置位置1到达：{' '.join(parts)}，总分 {out['total_score']}")

                if out["robot_pos3_triggered"]:
                    parts = []
                    parts.append("+10(抓取位置2)")
                    rospy.loginfo(f"[sim] 🟡 抓取位置2到达：{' '.join(parts)}，总分 {out['total_score']}")

                if out["robot_pos4_triggered"]:
                    parts = []
                    parts.append("+10(放置位置2)")
                    rospy.loginfo(f"[sim] 🟡 放置位置2到达：{' '.join(parts)}，总分 {out['total_score']}")                    

                if out["success_triggered"]:
                    parts = []
                    parts.append(f"+{out['time_score_added']}(时间)")
                    rospy.loginfo(f"[sim] ✅ 任务完成：{' '.join(parts)}，用时 {out['elapsed_sec']:.2f}s，总分 {out['total_score']}")

                # 持续发布当前分数
                self.score = out["total_score"]
                rate.sleep()

            # 6) 收到 reset → 清理并退出 (让 deploy.py 重启新一轮)
            rospy.loginfo("[sim] 收到 reset/shutdown，开始清理并退出")
            self._cleanup()
            sys.exit(0)

        except KeyboardInterrupt:
            rospy.loginfo("[sim] 用户中断")
            self._cleanup()
            sys.exit(0)
        except Exception as e:
            rospy.logerr(f"[sim] 程序执行出错: {e}")
            traceback.print_exc()
            self._cleanup()
            sys.exit(1)

    def _cleanup(self):
        try:
            if self.traj_ctrl:
                self.traj_ctrl.stop()
        except Exception:
            pass
        if self.started:   # self.started 在 _on_start_service 里置 True
            try:
                os.makedirs(os.path.dirname(self.score_file), exist_ok=True)
                with open(self.score_file, "w") as f:
                    f.write(f"{int(self.score)}\n")
                rospy.loginfo(f"[sim] 本轮最终得分已写入: {self.score_file}（得分={int(self.score)}）")
            except Exception as e:
                rospy.logwarn(f"[sim] 写入最终得分失败: {e}")

            try:
                base, _ = os.path.splitext(self.score_file)
                detail_json_path = base + ".json"
                components = {}

                if self.comp_item1_pos: components["item1_pos"]=25
                if self.comp_item2_pos: components["item2_pos"]=25
                if self.comp_robot_pos1: components["robot_pos1"]=10
                if self.comp_robot_pos2: components["robot_pos2"]=10
                if self.comp_robot_pos3: components["robot_pos3"]=10
                if self.comp_robot_pos4: components["robot_pos4"]=10

                # 时间得分：如果累计不到，可以按需要从总分反推；这里优先用累计值
                if self.comp_time_score and self.comp_time_score > 0:
                    components["time"] = round(float(self.comp_time_score), 6)

                detail_payload = {
                    "total": int(self.score),
                    "components": components,
                }
                with open(detail_json_path, "w") as jf:
                    json.dump(detail_payload, jf, ensure_ascii=False, indent=2)
                rospy.loginfo(f"[sim] 本轮分项明细已写入: {detail_json_path}")
            except Exception as e:
                rospy.logwarn(f"[sim] 写入分项明细失败: {e}")  
        else:
            rospy.loginfo("[sim] 本轮未触发 start，不写入得分文件。")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed",type = int)
    args = parser.parse_args()
    seed = args.seed
    task = SimulatorTask4(seed)
    task.run()
