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
from utils.evaluator import ScoringEvaluator3, ScoringConfig3
import rospy
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import numpy as np
import json

class SimulatorTask3():
    def __init__(self,seed):
        rospy.init_node('simulator_task3', anonymous=False)

        self.init_service = rospy.ServiceProxy('/simulator/init', Trigger)
        self.pub_success = rospy.Publisher('/simulator/success',Bool, queue_size=10)
        # self.pub_score   = rospy.Publisher('/simulator/score', Int32, queue_size=10, latch=True)
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
        self.obj_pos = None
        self.score = 0
        default_score_file = "/tmp/simulator_score_last.txt"
        self.score_file = os.environ.get("SCORE_FILE", default_score_file)
        # 成功状态
        self.started = False
        self.already_reported_success = False
        self.intermediate_awarded = False

        self.seed = seed
        self.obj_pos = ObjectPose()
        self.obj_pos_set = ObjectRandomizer()

        self.marker1_pos = self.obj_pos.wait_for_position("marker1", timeout=5.0)

        self.target_region = [
        (self.marker1_pos[0]-0.18, self.marker1_pos[0]+0.18),   # x 范围
        (self.marker1_pos[1]-0.18, self.marker1_pos[1]+0.18),   # y 范围
        (0.685, 0.693)  # z 范围
        ]
        # self.target_region = [
        # (0.16, 0.50),
        # (0.21, 0.58),
        # (0.6, 1.00),
        # ]

        # 记录分项是否达成
        self.comp_back_obj1_pos = False
        self.comp_back_obj1_ori = False
        self.comp_back_obj2_pos = False
        self.comp_back_obj2_ori = False
        self.comp_front_obj1_pos = False
        self.comp_front_obj1_ori = False
        self.comp_time_score = 0


        self.evaluator = ScoringEvaluator3(
            ScoringConfig3(
                target_region=self.target_region,
                body_front_axis='-y',
                front_world_dir='z',
                tol_deg=5.0,
                time_full=10,
                time_threshold_sec=30,
                time_penalty_per_sec=1,
            ),
            is_in_region_fn=lambda pos, region: Utils.is_in_target_region(pos, region),
            is_front_facing_fn=lambda quat_xyzw, body_front_axis, front_world_dir, tol_deg:
                Utils.is_front_facing(quat_xyzw=quat_xyzw,
                                    body_front_axis=body_front_axis,
                                    front_world_dir=front_world_dir,
                                    tol_deg=tol_deg)
        )
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
    
    def _randomize_objects_with_seed(self,seed: int, regions: dict, FRONT: list, BACK: list):
        """
        使用固定seed随机化多个物体的位置和朝向
        regions: dict
        FRONT/BACK: 四元数 [w, x, y, z]
        """
        rng = np.random.default_rng(int(seed))
        all_objects = list(regions.keys())

        # 随机选一个front，其余是back
        front_objects = rng.choice(all_objects, size=1, replace=False)
        back_objects = [obj for obj in all_objects if obj not in front_objects]

        object_configs = []
        for name, region in regions.items():
            x, y, z = self._sample_position_with_seed(seed, region)
            if name in front_objects:
                qw, qx, qy, qz = FRONT
            else:
                qw, qx, qy, qz = BACK

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

        return {
            "front": list(front_objects),
            "back": back_objects
        }
    
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
                "shampoo1": {"x": (0.13, 0.35), "y": (-0.2, -0.1), "z": (0.69, 0.69)},  # 区域 A
                "shampoo2": {"x": (0.13, 0.35), "y": (-0.35, -0.25), "z": (0.69, 0.69)},  # 区域 B
                "shampoo3": {"x": (0.13, 0.35), "y": (-0.48, -0.40), "z": (0.69, 0.69)},  # 区域 C
            }

            FRONT = [0.5, -0.5, 0.5, -0.5]  # (w, x, y, z)
            BACK = [0.5, 0.5, 0.5, 0.5]

            # 随机化物体位置
            result = self._randomize_objects_with_seed(seed=self.seed, regions=REGIONS, FRONT=FRONT, BACK=BACK)
            obj1, obj2 = result["back"]
            obj3 = result["front"][0]
            print("\033[92mBACK,BACK,FRONT\033[0m",obj1,obj2,obj3)

            # 2) 预抓位
            num = 30
            q1_target1 = [60, 0, 0, -100, 0, 0, 0,   70, 0, 0, -135, 70, 40, 0]
            q1_list1 = Utils.interpolate_joint_trajectory(q1_target1, num = num) 
                
            q1_target2 = [30, 0, 0, -140, 70, 0, 0,   10, 5, 0, -130, 90, 90, 0]
            q1_list2 = Utils.interpolate_joint_trajectory(q1_target2, q1_target1, num = num)

            for q in q1_list1 :
                self.robot.control_arm_joint_positions(q)
                time.sleep(0.02)
            for q in q1_list2:
                self.robot.control_arm_joint_positions(q)
                time.sleep(0.02)
            
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
                    pos_back_obj1 = self.obj_pos.get_position(obj1)
                    ori_back_obj1 = self.obj_pos.get_orientation(obj1)

                    pos_back_obj2 = self.obj_pos.get_position(obj2)
                    ori_back_obj2 = self.obj_pos.get_orientation(obj2)

                    pos_front_obj1 = self.obj_pos.get_position(obj3)
                    ori_front_obj1 = self.obj_pos.get_orientation(obj3)
                except Exception as e:
                    rospy.logwarn(f"[sim] 获取位置/姿态出错：{e}")
                    pos_back_obj1, ori_back_obj1,pos_back_obj2,ori_back_obj2,pos_front_obj1,ori_front_obj1 = None, None, None, None, None, None

                if pos_back_obj1 is None or ori_back_obj1 is None or pos_back_obj2 is None or ori_back_obj2 is None or pos_front_obj1 is None or ori_front_obj1 is None:
                    # 取不到传感就当未成功
                    # 未成功阶段持续发 False
                    self.pub_success.publish(Bool(data=False))
                    # 持续发布分数
                    self.pub_score.publish(Int32(data=self.evaluator.score))
                    rate.sleep()
                    continue

                # 调用通用评估器
                out = self.evaluator.evaluate(pos_front_obj1,ori_front_obj1, pos_back_obj1, ori_back_obj1, pos_back_obj2, ori_back_obj2, now=time.time())

                if out.get("back_obj1_pos_added"): self.comp_back_obj1_pos = True
                if out.get("back_obj1_ori_added"): self.comp_back_obj1_ori = True
                if out.get("back_obj2_pos_added"): self.comp_back_obj2_pos = True
                if out.get("back_obj2_ori_added"): self.comp_back_obj2_ori = True
                if out.get("front_obj1_pos_added"): self.comp_front_obj1_pos = True
                if out.get("front_obj1_pos_added"): self.comp_front_obj1_ori = True

                ts_add = out.get("time_score_added")
                if isinstance(ts_add, (int, float)):
                    self.comp_time_score += float(ts_add)

                # 按评估器的“建议标志”做 ROS 行为
                if out["need_publish_success_true"]:
                    rospy.loginfo("[sim] ✅ 任务成功，发布 /simulator/success=True")
                    self.pub_success.publish(Bool(data=True))

                elif out["need_publish_success_false"]:
                    self.pub_success.publish(Bool(data=False))

                if out["back_obj1_pos_added"] or out["back_obj1_ori_added"]:
                    parts = []
                    if out["back_obj1_pos_added"]: parts.append("+10(位置)")
                    if out["back_obj1_ori_added"]: parts.append("+25(方向)")
                    rospy.loginfo(f"[sim] 🟡 反面物体1放置成功：{' '.join(parts)}，总分 {out['total_score']}")

                if out["back_obj2_pos_added"] or out["back_obj2_ori_added"]:
                    parts = []
                    if out["back_obj2_pos_added"]: parts.append("+10(位置)")
                    if out["back_obj2_ori_added"]: parts.append("+25(方向)")
                    rospy.loginfo(f"[sim] 🟡 反面物体2放置成功：{' '.join(parts)}，总分 {out['total_score']}")

                if out["front_obj1_pos_added"] or out["front_obj1_ori_added"]:
                    parts = []
                    if out["front_obj1_pos_added"]: parts.append("+10(位置)")
                    if out["front_obj1_ori_added"]: parts.append("+10(方向)")
                    rospy.loginfo(f"[sim] 🟡 正面物体放置成功：{' '.join(parts)}，总分 {out['total_score']}")

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

                if self.comp_back_obj1_pos: components["back_obj1_pos"]=10
                if self.comp_back_obj1_ori: components["back_obj1_ori"]=25
                if self.comp_back_obj2_pos: components["back_obj2_pos"]=10
                if self.comp_back_obj2_ori: components["back_obj2_ori"]=25
                if self.comp_front_obj1_pos: components["front_obj1_pos"]=10
                if self.comp_front_obj1_ori: components["front_obj1_pos"]=10

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
    task = SimulatorTask3(seed)
    task.run()
