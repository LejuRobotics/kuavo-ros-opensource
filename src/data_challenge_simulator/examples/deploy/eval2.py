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
from utils.evaluator import ScoringEvaluator2, ScoringConfig2
import rospy
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import numpy as np
import json
import random
YELLOW = "\033[93m"
GREEN = "\033[92m"
RESET = "\033[0m"
class SimulatorTask2():
    def __init__(self,seed):
        rospy.init_node('simulator_task2', anonymous=False)

        self.init_service = rospy.ServiceProxy('/simulator/init', Trigger)
        self.pub_success = rospy.Publisher('/simulator/success',Bool, queue_size=10)

        self.start_service = rospy.Service('/simulator/start', Trigger, self._on_start_service)
        self.reset_service = rospy.Service('/simulator/reset', Trigger, self._on_reset_service)

        self.start_evt = threading.Event()
        self.reset_evt = threading.Event()

        self.robot = None
        self.robot_state = None
        self.conveyor_ctrl = None
        self.gripper_ctrl = None
        self.traj_ctrl = None
        self.obj_pos = None
        self.score = 0
        default_score_file = "/tmp/simulator_score_last.txt"
        self.score_file = os.environ.get("SCORE_FILE", default_score_file)

        self.started = False
        self.already_reported_success = False
        self.intermediate_awarded = False

        self.seed = seed
        self.obj_pos = ObjectPose()
        self.rng = random.Random(self.seed)
        self.np_rng = np.random.default_rng(self.seed)
        self.select_object = self.rng.choice(["bag_place_root", "bag_drop_root"])
        self.conveyor_speed1 = 0.02
        self.conveyor_speed2 = -0.02
        self.intermediate_region = [
            [0.28,0.52],
            [-0.12,0.12],
            [0.74,0.82]
        ]
        self.target_region_place = [
            [-0.5,0.5],
            [0.4,0.8],
            [0.74,0.8]
        ]
        self.target_region_drop = [
            [0.12,0.40],
            [-0.02,0.25],
            [0.46,0.66]
        ]

        if self.select_object == "bag_place_root":
            self.target_region = self.target_region_place
        else:
            self.target_region = self.target_region_drop

        self.comp_intermediate_pos = False
        self.comp_final_pos = False
        self.comp_time_score = 0

        self.evaluator = ScoringEvaluator2(
            ScoringConfig2(
                target_region=self.target_region,
                intermediate_region=self.intermediate_region,
                time_full=20,
                time_threshold_sec=20,
                time_penalty_per_sec=1,
            ),
            is_in_region_fn=lambda pos, region: Utils.is_in_target_region(pos, region),
        )

    def _on_start_service(self, req):
        """等待外部代码发送 start 信号"""
        rospy.loginfo("[sim] Received external start signal, starting task execution")
        self.started = True
        self.start_evt.set()
        return TriggerResponse(success=True, message="Task started successfully")

    def _on_reset_service(self, req):
        """等待外部代码发送 reset 信号"""
        rospy.loginfo("[sim] Received external reset signal, preparing to reset task")
        self.reset_evt.set()
        return TriggerResponse(success=True, message="Task reset triggered")

    def _sample_position(self, position_ranges: dict):
        """
        使用固定种子采样位置，返回 (x, y, z)
        """
        x = float(self.np_rng.uniform(*position_ranges['x']))
        y = float(self.np_rng.uniform(*position_ranges['y']))
        z = float(self.np_rng.uniform(*position_ranges['z']))
        return x, y, z

    def run(self):
        try:
            self.reset_evt.clear()
            self.start_evt.clear()

            if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
                print("Init KuavoSDK failed, exit!")
                return

            self.robot = KuavoRobot()
            self.robot_state = KuavoRobotState()

            self.conveyor_ctrl = ConveyorController()
            self.obj_pos       = ObjectPose()

            obj_pos = ObjectRandomizer()
            x, y, z = self._sample_position(position_ranges={
                    'x': [0.25, 0.25],
                    'y': [-0.7, -0.5],  
                    'z': [0.9, 0.9]     
                })
            
            result = obj_pos.set_object_position(
                object_name=self.select_object,
                position = {"x":x, "y":y, "z":z},
                orientation = Utils.sample_orientation_jitter(max_yaw_deg=180.0, max_pitch_deg=0, max_roll_deg=0)
            )

            num = 30
            q_target1 = [90, -10, 0, -100, -90, 0, 0,   80, 10, 0, -120, 90, 0, 0]
            q_list1 = Utils.interpolate_joint_trajectory(q_target1, num=num) 
                
            q_target2 = [-10, 0, 0, -100, -90, -40, 0,   10,0, -35, -110, 90, 40, 0]
            q_list2 = Utils.interpolate_joint_trajectory(q_target2, q_target1, num=num)

            for q in q_list1 :
                self.robot.control_arm_joint_positions(q)
                time.sleep(0.02)
            for q in q_list2:
                self.robot.control_arm_joint_positions(q)
                time.sleep(0.02)

            rospy.wait_for_service('/simulator/init')
            try:
                rospy.loginfo("[sim] Published /simulator/init service completed")
                resp = self.init_service(TriggerRequest())
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

            rospy.loginfo("[sim] Waiting for external code to call /simulator/start service...")
            while not rospy.is_shutdown() and not self.reset_evt.is_set():
                if self.start_evt.wait(timeout=0.1):
                    break

            if self.reset_evt.is_set() or rospy.is_shutdown():
                rospy.logwarn("[sim] Received reset / shutdown during wait for start, exiting")
                self._cleanup()
                sys.exit(0)

            rospy.loginfo("[sim] Received external start signal, continuously reporting success")
            self.conveyor_ctrl.control_speed_both(self.conveyor_speed1,self.conveyor_speed2)

            rate = rospy.Rate(10)
            self.already_reported_success = False

            self.evaluator.reset()
            
            while not rospy.is_shutdown() and not self.reset_evt.is_set():
                try:
                    pos = self.obj_pos.get_position(self.select_object)
                except Exception as e:
                    rospy.logwarn(f"[sim] Failed to get position/orientation: {e}")
                    pos = None

                if pos is None:
                    self.pub_success.publish(Bool(data=False))
                    rate.sleep()
                    continue

                out = self.evaluator.evaluate(pos, now=time.time())

                if out.get("intermediate_pos_added"): self.comp_intermediate_pos = True
                if out.get("final_pos_added"):        self.comp_final_pos = True


                ts_add = out.get("time_score_added")
                if isinstance(ts_add, (int, float)):
                    self.comp_time_score += float(ts_add)

                if out["need_publish_success_true"]:
                    rospy.loginfo("[sim] ✅ Task success, published /simulator/success=True")
                    self.pub_success.publish(Bool(data=True))

                elif out["need_publish_success_false"]:
                    self.pub_success.publish(Bool(data=False))

                if out["intermediate_pos_added"]:
                    parts = []
                    parts.append("+40( Intermediate Point reached! )")
                    rospy.loginfo(f"{YELLOW} 🟡 Points Added: {' '.join(parts)} | Total Score {out['total_score']}{RESET}")

                if out["final_pos_added"]:
                    parts = []
                    parts.append("+40( Final Point reached! )")
                    rospy.loginfo(f"{YELLOW} 🟡 Points Added: {' '.join(parts)} | Total Score {out['total_score']}{RESET}")
                    parts = []
                    parts.append(f"+{out['time_score_added']}(Time)")
                    rospy.loginfo(f"{GREEN} ✅ Task Success: {' '.join(parts)} | Total Time: {out['elapsed_sec']:.2f}s | Total Score: {out['total_score']}{RESET}")

                    if out["need_stop_conveyor"]:
                        self.conveyor_ctrl.control_speed_both(0.0,0.0)

                self.score = out["total_score"]

                rate.sleep()

            rospy.loginfo("[sim] Received reset/shutdown, starting cleanup and exiting")
            self._cleanup()
            sys.exit(0)

        except KeyboardInterrupt:
            rospy.loginfo("[sim] User interrupted")
            self._cleanup()
            sys.exit(0)
        except Exception as e:
            rospy.logerr(f"[sim] Program execution error: {e}")
            traceback.print_exc()
            self._cleanup()
            sys.exit(1)

    def _cleanup(self):
        try:
            if self.conveyor_ctrl:
                self.conveyor_ctrl.control_speed_both(0.0,0.0)
        except Exception:
            pass
        try:
            if self.traj_ctrl:
                self.traj_ctrl.stop()
        except Exception:
            pass
        if self.started:
            try:
                os.makedirs(os.path.dirname(self.score_file), exist_ok=True)
                with open(self.score_file, "w") as f:
                    f.write(f"{int(self.score)}\n")
                rospy.loginfo(f"[sim] Final score written to: {self.score_file} (score={int(self.score)})")
            except Exception as e:
                rospy.logwarn(f"[sim] Failed to write final score: {e}")
            try:
                base, _ = os.path.splitext(self.score_file)
                detail_json_path = base + ".json"
                components = {}

                if self.comp_intermediate_pos: components["intermediate_pos"] = 40
                if self.comp_final_pos:        components["final_pos"] = 40

                if self.comp_time_score and self.comp_time_score > 0:
                    components["time"] = round(float(self.comp_time_score), 6)

                detail_payload = {
                    "total": int(self.score),
                    "components": components,
                }
                with open(detail_json_path, "w") as jf:
                    json.dump(detail_payload, jf, ensure_ascii=False, indent=2)
                rospy.loginfo(f"[sim] Detail written to: {detail_json_path}")
            except Exception as e:
                rospy.logwarn(f"[sim] Failed to write detail: {e}")

        else:
            rospy.loginfo("[sim] Task not started, not writing score file.")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed",type = int)
    args = parser.parse_args()
    seed = args.seed
    task = SimulatorTask2(seed)
    task.run()
