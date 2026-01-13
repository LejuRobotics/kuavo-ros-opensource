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
from utils.evaluator import ScoringEvaluator1, ScoringConfig1
import rospy
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import numpy as np
import json
import random
YELLOW = "\033[93m"
GREEN = "\033[92m"
RESET = "\033[0m"
class SimulatorTask1():
    def __init__(self,seed):
        rospy.init_node('simulator_task1', anonymous=False)

        self.init_service = rospy.ServiceProxy('/simulator/init', Trigger)
        self.pub_success = rospy.Publisher('/simulator/success',Bool, queue_size=10)

        self.start_service = rospy.Service('/simulator/start', Trigger, self._on_start_service)
        self.reset_service = rospy.Service('/simulator/reset', Trigger, self._on_reset_service)

        self.start_evt = threading.Event()
        self.reset_evt = threading.Event()

        self.robot = None
        self.robot_state = None
        self.gripper_ctrl = None
        self.traj_ctrl = None
        self.obj_pos = None
        self.score = 0
        default_score_file = "/tmp/simulator_score_last.txt"
        self.score_file = os.environ.get("SCORE_FILE", default_score_file)

        self.started = False
        self.already_reported_success = False

        self.seed = seed
        self.rng = random.Random(self.seed)
        self.np_rng = np.random.default_rng(self.seed)
        self.obj_pos = ObjectPose()
        self.obj_pos_set = ObjectRandomizer()
        GREEN = "\033[92m"
        RESET = "\033[0m"
        basket_pos = self.obj_pos.wait_for_position("basket1",timeout=5)
        print(f"{GREEN}table height is {basket_pos[2]}{RESET}")
        height = basket_pos[2]-0.005

        self.toy_box_region = [
            (0.31,0.49),
            (-0.15,0),
            (height,height+0.12)
        ]
        self.car_box_region = [
            (0.31,0.49),
            (0,0.15),
            (height,height+0.12)
        ]

        self.left_spawn_region = {
            "x": [0.23,0.43],
            "y": [0.22,0.45],
            "z": [height+0.01,height+0.01]
        }
        self.right_spawn_region = {
            "x": [0.23,0.43],
            "y": [-0.45,-0.22],
            "z": [height+0.01,height+0.01]
        }
        self.toy_obj = ["dino","dog","elephant","shark","sheep"]
        self.car_obj = ["fire","roller","school","boat","fire_engine"]
        self.toy = self.rng.choice(self.toy_obj)
        self.car = self.rng.choice(self.car_obj)

        self.comp_toy_pos = False
        self.comp_car_pos = False
        self.comp_time_score = 0

        self.evaluator = ScoringEvaluator1(
            ScoringConfig1(
                toy_box_region=self.toy_box_region,
                car_box_region=self.car_box_region,
                time_full=20,
                time_threshold_sec=15,
                time_penalty_per_sec=1,
            ),
            is_in_region_fn=lambda pos, region: Utils.is_in_target_region(pos, region)
        )

    def _on_start_service(self, req):
        """Wait for external code to send the start signal."""
        rospy.loginfo("[sim] Received external start signal, beginning task execution")
        self.started = True
        self.start_evt.set()
        return TriggerResponse(success=True, message="Task started successfully")

    def _on_reset_service(self, req):
        """Wait for external code to send the reset signal."""
        rospy.loginfo("[sim] Received external reset signal, preparing to reset task")
        self.reset_evt.set()
        return TriggerResponse(success=True, message="Task reset triggered")

    def _sample_position(self, region: dict):
        """
        Sample a single object's position.
        region: {"x": (xmin, xmax), "y": (ymin, ymax), "z": (zmin, zmax)}
        """
        x = float(self.np_rng.uniform(*region["x"]))
        y = float(self.np_rng.uniform(*region["y"]))
        z = float(self.np_rng.uniform(*region["z"]))
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
            coin = self.rng.choice([0,1])

            if coin == 0:
                for obj, region in zip([self.toy, self.car], [self.left_spawn_region, self.right_spawn_region]):
                    x, y, z = self._sample_position(region=region)
                    result = self.obj_pos_set.set_object_position(obj, position={"x":x, "y":y, "z":z})
                    print(f"{obj} spawned at {region} region, position: {result['final_position']}")
            else:
                for obj, region in zip([self.toy, self.car], [self.right_spawn_region, self.left_spawn_region]):
                    x, y, z = self._sample_position(region=region)
                    result = self.obj_pos_set.set_object_position(obj, position={"x":x, "y":y, "z":z})
                    print(f"{obj} spawned at {region} region, position: {result['final_position']}")

            num = 30
            q1_target1 = [90, -10, 0, -150, -90, 0, 0,   90, 10, 0, -150, 90, 0, 0]
            q1_list1 = Utils.interpolate_joint_trajectory(q1_target1, num = num) 
                
            q1_target2 = [30, 0, 0, -130, -90, -40, 0,   30, 0, 0, -130, 90, 40, 0]
            q1_list2 = Utils.interpolate_joint_trajectory(q1_target2, q1_target1, num = num)

            for q in q1_list1 :
                self.robot.control_arm_joint_positions(q)
                time.sleep(0.02)
            for q in q1_list2:
                self.robot.control_arm_joint_positions(q)
                time.sleep(0.02)
            
            rospy.wait_for_service('/simulator/init')
            try:
                rospy.loginfo("[sim] Published /simulator/init service completed")
                resp = self.init_service(TriggerRequest())
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")


            rospy.loginfo("[sim] Waiting for external code to call the /simulator/start service...")
            while not rospy.is_shutdown() and not self.reset_evt.is_set():
                if self.start_evt.wait(timeout=0.1):
                    break

            if self.reset_evt.is_set() or rospy.is_shutdown():
                rospy.logwarn("[sim] Received reset/shutdown while waiting for start, exiting")
                self._cleanup()
                sys.exit(0)

            rospy.loginfo("[sim] Received external start signal, continuously publishing /simulator/success")

            rate = rospy.Rate(10)
            self.already_reported_success = False
            self.evaluator.reset()
            
            while not rospy.is_shutdown() and not self.reset_evt.is_set():
                try:
                    pos_toy = self.obj_pos.get_position(self.toy)
                    pos_car = self.obj_pos.get_position(self.car)
                except Exception as e:
                    rospy.logwarn(f"[sim] Failed to get position/orientation: {e}")
                    pos_toy, pos_car = None, None

                if pos_toy is None or pos_car is None:
                    self.pub_success.publish(Bool(data=False))
                    rate.sleep()
                    continue

                out = self.evaluator.evaluate(pos_toy, pos_car, now=time.time())

                if out.get("toy_pos_added"): self.comp_toy_pos = True
                if out.get("car_pos_added"): self.comp_car_pos = True

                ts_add = out.get("time_score_added")
                if isinstance(ts_add, (int, float)):
                    self.comp_time_score += float(ts_add)

                if out["need_publish_success_true"]:
                    rospy.loginfo("[sim] ✅ Task success, published /simulator/success=True")
                    self.pub_success.publish(Bool(data=True))

                elif out["need_publish_success_false"]:
                    self.pub_success.publish(Bool(data=False))

                if out["toy_pos_added"]:
                    parts = []
                    parts.append("+40( Toy in the box!)")
                    rospy.loginfo(f"{YELLOW} 🟡 Points Added:{''.join(parts)} | Total score: {out['total_score']}{RESET}")

                if out["car_pos_added"]:
                    parts = []
                    parts.append("+40( Car in the box!)")
                    rospy.loginfo(f"{YELLOW} 🟡 Points Added:{' '.join(parts)}| Total score: {out['total_score']}{RESET}")

                if out["success_triggered"]:
                    parts = []
                    parts.append(f"+{out['time_score_added']}( Time! )")
                    rospy.loginfo(f"{GREEN} ✅ Task Success: {' '.join(parts)} | Total Time: {out['elapsed_sec']:.2f}s | Total Score: {out['total_score']}{RESET}")

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

                if self.comp_toy_pos: components["toy_pos"]=40
                if self.comp_car_pos: components["car_pos"]=40

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
    task = SimulatorTask1(seed)
    task.run()
