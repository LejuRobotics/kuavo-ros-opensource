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
import random

YELLOW = "\033[93m"
GREEN = "\033[92m"
RESET = "\033[0m"

class SimulatorTask3():
    def __init__(self,seed):
        rospy.init_node('simulator_task3', anonymous=False)

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

        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)
        self.obj_pos = ObjectPose()

        self.random_pos = ObjectRandomizer()
        self.conveyor_ctrl = ConveyorController()

        self.red_bin_region = [
        (0.155,0.365),  
        (-0.095,0.115),  
        (0.46,0.55) 
        ]
        self.black_bin_region = [
        (0.155,0.365),  
        (-0.305,-0.095),   
        (0.46,0.55)  
        ]

        self.comp_box_red1_pos = False
        self.comp_box_black1_pos = False
        self.comp_box_red2_pos = False
        self.comp_box_black2_pos = False
        self.comp_bonus = False
        self.comp_time_score = 0


        self.evaluator = ScoringEvaluator3(
            ScoringConfig3(
                red_bin_region=self.red_bin_region,
                black_bin_region=self.black_bin_region,
                time_full=10,
                time_threshold_sec=55,
                time_penalty_per_sec=1,
            ),
            is_in_region_fn=lambda pos, region: Utils.is_in_target_region(pos, region),
        )

        self.belt_speed = 0.012
        self.objects_total = ['box_red1','box_black1','box_red2', 'box_black2']
        self.rng.shuffle(self.objects_total)
        self.objects1 = self.objects_total[0]
        self.objects = self.objects_total[1:]
        self.object_index = [0]
        self.spawn_interval_range = [10.0, 14.0]
        self.timer_handle = {'timer': None}

    def _on_start_service(self, req):
        rospy.loginfo("[sim] Received external start signal, starting task execution")
        self.started = True
        self.start_evt.set()
        return TriggerResponse(success=True, message="Task started successfully")

    def _on_reset_service(self, req):
        rospy.loginfo("[sim] Received external reset signal, preparing to reset task")
        self.reset_evt.set()
        return TriggerResponse(success=True, message="Task reset triggered")

    def _euler_to_quat(self,roll, pitch, yaw):
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return {'x': qx, 'y': qy, 'z': qz, 'w': qw}

    def _sample_orientation_jitter(self, max_yaw_deg=20.0, max_pitch_deg=0.0, max_roll_deg=0.0):
        yaw = math.radians(self.np_rng.uniform(-max_yaw_deg, max_yaw_deg))
        pitch = math.radians(self.np_rng.uniform(-max_pitch_deg, max_pitch_deg))
        roll = math.radians(self.np_rng.uniform(-max_roll_deg, max_roll_deg))
        return self._euler_to_quat(roll, pitch, yaw)

    def _sample_spawn_position(self):
        return {
            'x': 0.4 + self.np_rng.uniform(-0.02, 0.10),
            'y': -0.37 + self.np_rng.uniform(0, 0),
            'z': 0.85
        }
    def _generate_objects_callback(self, event):
        current_idx = self.object_index[0]
        obj_name = self.objects[current_idx]
        
        orientation = self._sample_orientation_jitter(max_yaw_deg=20, max_pitch_deg=0, max_roll_deg=0)
        position = self._sample_spawn_position()
        result = self.random_pos.set_object_position(obj_name, position=position, orientation=orientation)
        if result['success']:
            rospy.loginfo(f"Successfully generated {obj_name} at position {position} with orientation {orientation}")
        else:
            rospy.logwarn(f"Failed to generate {obj_name}: {result['message']}")

        self.object_index[0] += 1
        
        if self.object_index[0] >= len(self.objects):
            rospy.loginfo("All objects generated once. Stopping timer.")
            if self.timer_handle['timer']:
                self.timer_handle['timer'].shutdown()
        else:

            next_interval = self.np_rng.uniform(self.spawn_interval_range[0], self.spawn_interval_range[1])
            rospy.loginfo(f"Next object will spawn in {next_interval:.2f} seconds")
            self.timer_handle['timer'] = rospy.Timer(rospy.Duration(next_interval), self._generate_objects_callback, oneshot=True)

    def run(self):
        try:
            self.reset_evt.clear()
            self.start_evt.clear()

            if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
                print("Init KuavoSDK failed, exit!")
                return 

            self.robot = KuavoRobot()
            self.robot_state = KuavoRobotState()

            num = 30
            q_target1 = [0, 0, 0, 0, 0, 0, 0,   80, -30, 0, -125, 45, 0, 0]
            q_list1 = Utils.interpolate_joint_trajectory(q_target1, num=num)

            q_target2 = [0, 0, 0, 0, 0, 0, 0,   18, 0, 0, -110, 90, 0, 0]

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

            self.random_pos.set_object_position(self.objects1, position=self._sample_spawn_position(), orientation=self._sample_orientation_jitter(max_yaw_deg=20, max_pitch_deg=0, max_roll_deg=0))

            self.conveyor_ctrl.control_speed(self.belt_speed,belt_id=1)

            first_interval = self.np_rng.uniform(self.spawn_interval_range[0], self.spawn_interval_range[1])
            rospy.loginfo(f"Object generation timer started. First object will spawn in {first_interval:.2f} seconds (range: {self.spawn_interval_range[0]}-{self.spawn_interval_range[1]}s)")
            self.timer_handle['timer'] = rospy.Timer(rospy.Duration(first_interval), self._generate_objects_callback, oneshot=True)

            rate = rospy.Rate(10)
            self.already_reported_success = False

            self.evaluator.reset()

            def ros_spin_thread():
                rospy.spin()
            
            spin_thread = threading.Thread(target=ros_spin_thread, daemon=True)
            spin_thread.start()
            
            while not rospy.is_shutdown() and not self.reset_evt.is_set():
                try:
                    box_red1_pos = self.obj_pos.get_position("box_red1")
                    box_black1_pos = self.obj_pos.get_position("box_black1")
                    box_red2_pos = self.obj_pos.get_position("box_red2")
                    box_black2_pos = self.obj_pos.get_position("box_black2")
                except Exception as e:
                    rospy.logwarn(f"[sim] Error getting position/orientation: {e}")
                    box_red1_pos, box_black1_pos, box_red2_pos, box_black2_pos = None, None, None, None

                if box_red1_pos is None or box_black1_pos is None or box_red2_pos is None or box_black2_pos is None:

                    self.pub_success.publish(Bool(data=False))

                    rate.sleep()
                    continue

                out = self.evaluator.evaluate(box_red1_pos, box_black1_pos, box_red2_pos, box_black2_pos, now=time.time())

                if out.get("box_red1_pos_added"): self.comp_box_red1_pos = True
                if out.get("box_black1_pos_added"): self.comp_box_black1_pos = True
                if out.get("box_red2_pos_added"): self.comp_box_red2_pos = True
                if out.get("box_black2_pos_added"): self.comp_box_black2_pos = True
                if out.get("bonus_added"): self.comp_bonus = True

                ts_add = out.get("time_score_added")
                if isinstance(ts_add, (int, float)):
                    self.comp_time_score += float(ts_add)

                if out["need_publish_success_true"]:
                    rospy.loginfo("[sim] ✅ Task success, published /simulator/success=True")
                    self.pub_success.publish(Bool(data=True))

                elif out["need_publish_success_false"]:
                    self.pub_success.publish(Bool(data=False))

                if out["box_red1_pos_added"]:
                    parts = []
                    if out["box_red1_pos_added"]: parts.append("+20 ( White Box in the Box! )")
                    rospy.loginfo(f"{YELLOW} 🟡 Points Added: {' '.join(parts)} | Total Score: {out['total_score']}{RESET}")

                if out["box_red2_pos_added"]:
                    parts = []
                    if out["box_red2_pos_added"]: parts.append("+20 ( White Box in the Box! )")
                    rospy.loginfo(f"{YELLOW} 🟡 Points Added: {' '.join(parts)} | Total Score: {out['total_score']}{RESET}")

                if out["box_black1_pos_added"]:
                    parts = []
                    if out["box_black1_pos_added"]: parts.append("+20 ( Black Box in the Box! )")
                    rospy.loginfo(f"{YELLOW} 🟡 Points Added: {' '.join(parts)} | Total Score: {out['total_score']}{RESET}")

                if out["box_black2_pos_added"]:
                    parts = []
                    if out["box_black2_pos_added"]: parts.append("+20 ( Black Box in the Box! )")
                    rospy.loginfo(f"{YELLOW} 🟡 Points Added: {' '.join(parts)} | Total Score: {out['total_score']}{RESET}")
                
                if out["success_triggered"]:
                    parts = []
                    parts.append(f"+{out['time_score_added']}(Time!)")
                    parts.append("+10 ( Bonus! )")
                    rospy.loginfo(f"{GREEN} ✅ Success Triggered: {' '.join(parts)} | Total Time: {out['elapsed_sec']:.2f}s | Total Score: {out['total_score']}{RESET}")

                    if out["need_stop_conveyor"]:
                        self.conveyor_ctrl.control_speed(0.0)

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
                self.conveyor_ctrl.control_speed(0.0)
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
                rospy.loginfo(f"[sim] Final score for this round written to: {self.score_file} (score={int(self.score)})")
            except Exception as e:
                rospy.logwarn(f"[sim] Failed to write final score: {e}")

            try:
                base, _ = os.path.splitext(self.score_file)
                detail_json_path = base + ".json"
                components = {}

                if self.comp_box_red1_pos: components["box_red1_pos"] = 20
                if self.comp_box_black1_pos: components["box_black1_pos"] = 20
                if self.comp_box_red2_pos: components["box_red2_pos"] = 20
                if self.comp_box_black2_pos: components["box_black2_pos"] = 20
                if self.comp_bonus: components["bonus"] = 10

                if self.comp_time_score and self.comp_time_score > 0:
                    components["time"] = round(float(self.comp_time_score), 6)

                detail_payload = {
                    "total": int(self.score),
                    "components": components,
                }
                with open(detail_json_path, "w") as jf:
                    json.dump(detail_payload, jf, ensure_ascii=False, indent=2)
                rospy.loginfo(f"[sim] Details for this round written to: {detail_json_path}")
            except Exception as e:
                rospy.logwarn(f"[sim] Failed to write details: {e}")

        else:
            rospy.loginfo("[sim] No start triggered, not writing score file.")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed",type = int)
    args = parser.parse_args()
    seed = args.seed
    task = SimulatorTask3(seed)
    task.run()
