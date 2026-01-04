import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))               
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))           
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import time
import signal
import subprocess
from datetime import datetime
from pathlib import Path
import argparse
from utils.xml_random import randomize_mjcf
import json
import random
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


GREEN = "\033[92m"
CYAN  = "\033[96m"
YELLOW= "\033[93m"
RESET = "\033[0m"

def generate_cfg(seed):


    rng = random.Random(seed)
    height = rng.uniform(0.75, 0.85)
    table_height = rng.uniform(0.7,0.8)
    cfg_conveyor = {
        "rules": [
            {
                "select": ".//body[@name='belt_plate1']",
                "attributes": {
                    "pos": {
                        "per_dim": [
                            {"set": 0.5},
                            {"set": 0},
                            {"set": height}
                        ]
                    }
                }
            },

            {
                "select": ".//body[@name='belt_front']",
                "attributes": {
                    "pos": {
                        "per_dim": [
                            {"set": 0.5},
                            {"set": 0},
                            {"set": height}
                        ]
                    }
                }
            },
            {
                "select": ".//body[@name='belt_back']",
                "attributes": {
                    "pos": {
                        "per_dim": [
                            {"set": -3},
                            {"set": 0},
                            {"set": height}
                        ]
                    }
                }
            },
            {
                "select": ".//body[@name='belt_left']",
                "attributes": {
                    "pos": {
                        "per_dim": [
                            {"set": -1.25},
                            {"set": 2.7},
                            {"set": height}
                        ]
                    }
                }
            },  
            {
                "select": ".//body[@name='belt_right']",
                "attributes": {
                    "pos": {
                        "per_dim": [
                            {"set": -1.25},
                            {"set": -2.7},
                            {"set": height}
                        ]
                    }
                }
            },  
            {
                "select": ".//body[@name='circle_left_front']",
                "attributes": {
                    "pos": {
                        "per_dim": [
                            {"set": 0.3},
                            {"set": 2.5},
                            {"set": height+0.001}
                        ]
                    }
                }
            }, 
            {
                "select": ".//body[@name='circle_left_back']",
                "attributes": {
                    "pos": {
                        "per_dim": [
                            {"set": -2.8},
                            {"set": 2.5},
                            {"set": height+0.001}
                        ]
                    }
                }
            }, 
            {
                "select": ".//body[@name='circle_right_front']",
                "attributes": {
                    "pos": {
                        "per_dim": [
                            {"set": 0.3},
                            {"set": -2.5},
                            {"set": height+0.001}
                        ]
                    }
                }
            },  
            {
                "select": ".//body[@name='circle_right_back']",
                "attributes": {
                    "pos": {
                        "per_dim": [
                            {"set": -2.8},
                            {"set": -2.5},
                            {"set": height+0.001}
                        ]
                    }
                }
            },
            {
                "select": ".//body[@name='box_left_front']",
                "attributes": {
                    "pos": {
                        "per_dim": [
                            {"set": 0.1},
                            {"set": 2.3}, 
                            {"set": height-0.425+0.015}         
                        ]
                    }
                }
            },
            {
                "select": ".//body[@name='box_right_front']",
                "attributes": {
                    "pos": {  
                        "per_dim": [
                            {"set": 0.1},   
                            {"set": -2.3},
                            {"set": height-0.425+0.015}    
                        ]
                    }
                }
            },
            {
                "select": ".//body[@name='box_left_back']",
                "attributes": {
                    "pos": { 
                        "per_dim": [
                            {"set": -2.6},   
                            {"set": 2.3},
                            {"set": height-0.425+0.015}           
                        ]
                    }
                }
            },
            {
                "select": ".//body[@name='box_right_back']",
                "attributes": {
                    "pos": {  
                        "per_dim": [
                            {"set": -2.6},    
                            {"set": -2.3},  
                            {"set": height-0.425+0.015}               
                        ]
                    }
                }
            },
        ]
    }

    cfg_table = {
    "rules": [
        {
            "select": ".//body[@name='table_front']",
            "attributes": {
                "pos": { 
                    "per_dim": [
                        {"set": 0.55},   
                        {"set": 0}, 
                        {"set": table_height}               
                    ]
                }
            }
        },
    ]
}
    cfg_basket1 = {
    "rules": [
        {
            "select": ".//body[@name='basket1']",
            "attributes": {
                "pos": { 
                    "per_dim": [
                        {"set": 0.4},  
                        {"set": 0.08},  
                        {"set": table_height+0.005}            
                    ]
                }
            }
        },
    ]
}

    cfg_basket2 = {
    "rules": [
        {
            "select": ".//body[@name='basket2']",
            "attributes": {
                "pos": {  
                    "per_dim": [
                        {"set": 0.4},   
                        {"set": -0.08},  
                        {"set": table_height+0.005}            
                    ]
                }
            }
        },
    ]
}
    return  cfg_conveyor, cfg_table, cfg_basket1, cfg_basket2
def read_score_from_file(score_file: str) -> int:
    try:
        with open(score_file, "r") as f:
            line = f.readline().strip()
            return int(line)
    except Exception:
        return None

def pre_run_cleanup(scores_dir: str, score_file: str):
    os.makedirs(scores_dir, exist_ok=True)
    targets = [
        os.path.join(scores_dir, "score.json"),
        score_file,
        os.path.splitext(score_file)[0] + ".json",
    ]
    for p in targets:
        try:
            os.remove(p)
        except FileNotFoundError:
            pass
        except Exception:
            pass



def write_score_json(path, task_id, scores, comp_sum):
    import os, json

    valid_scores = [s for s in scores if isinstance(s, (int, float))]
    valid_cycles = len(valid_scores)
    avg_total = (sum(valid_scores) / valid_cycles) if valid_cycles else 0.0

    keys = set(comp_sum.keys())
    process_keys, time_key = [], "time"
    try:
        catalog_path = os.path.join(os.path.dirname(path), "components_catalog.json")
        if os.path.exists(catalog_path):
            with open(catalog_path, "r") as cf:
                catalog = json.load(cf)
            expected = catalog.get(str(task_id)) or catalog.get(task_id) or []
            keys |= set(expected)
            process_keys = [k for k in expected if k.lower() != "time"]
            if "time" in expected:
                time_key = "time"
    except Exception:
        pass

    denom = valid_cycles if valid_cycles else 1
    components_avg = {}
    for k in sorted(keys):
        total_for_k = float(comp_sum.get(k, 0.0))
        components_avg[k] = total_for_k / denom

    process_avg = sum(components_avg.get(k, 0.0) for k in process_keys)
    for k in process_keys:
        components_avg.pop(k, None)
    components_avg["process"] = process_avg

    payload = {
        "task_id": task_id,
        "valid_cycles": valid_cycles,
        "avg_total_score": avg_total,
        "components_avg": components_avg
    }
    with open(path, "w") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)




def try_read_detail_json(score_file):
    try:
        base, _ = os.path.splitext(score_file)
        detail_path = base + ".json"
        if os.path.exists(detail_path):
            with open(detail_path, "r") as f:
                return json.load(f)
    except Exception:
        pass
    return None


def append_history(history_file: str, cycle_idx: int, score: int):
    try:
        os.makedirs(os.path.dirname(history_file), exist_ok=True)
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(history_file, "a") as f:
            f.write(f"{ts}\tcycle={cycle_idx}\tscore={score}\n")
    except Exception:
        pass

def run_task(task_id: int, headless: bool, time_wait: float = 5):

    print("[INFO] Starting roscore...")
    roscore_process = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(1)

    if headless:
        task_env = os.environ.copy()
        task_env["MUJOCO_HEADLESS"] = "1"
    else:
        task_env = os.environ.copy()

    task_script = os.path.join(SCRIPT_DIR, f"eval{task_id}.py")
    launch_file = f"load_kuavo_mujoco_sim{task_id}.launch"

    if not os.path.exists(task_script):
        print(f"[ERROR] TASK {task_id} SCRIPT NOT FOUND：{task_script}")
        sys.exit(1)

    scores_dir = os.path.join(SCRIPT_DIR, "scores")
    score_file = os.path.join(scores_dir, f"score_task{task_id}_last.txt")

    pre_run_cleanup(scores_dir, score_file)
    scores = []

    components_sum = {} 
    score_json_path = os.path.join(scores_dir, "score.json")
    os.makedirs(scores_dir, exist_ok=True)

    cycle_idx = 1
    while True:
        seed = 42 + cycle_idx
        cfg_conveyor, cfg_table, cfg_basket1, cfg_basket2 = generate_cfg(seed)

        if task_id == 1:
            randomize_mjcf(
                in_path="/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/scene1.xml",
                out_path="/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/scene1.xml",
                config=cfg_table,
            )
            randomize_mjcf(
                in_path="/root/kuavo_ws/src/data_challenge_simulator/models/assets/basket1/model.xml",
                out_path="/root/kuavo_ws/src/data_challenge_simulator/models/assets/basket1/model.xml",
                config=cfg_basket1,
            )
            randomize_mjcf(
                in_path="/root/kuavo_ws/src/data_challenge_simulator/models/assets/basket2/model.xml",
                out_path="/root/kuavo_ws/src/data_challenge_simulator/models/assets/basket2/model.xml",
                config=cfg_basket2,
            )

        if task_id == 3:
            randomize_mjcf(
                in_path="/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/scene3.xml",
                out_path="/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/scene3.xml",
                config=cfg_conveyor,
                seed = seed
            )


        print(f"\n[INFO] === Round {cycle_idx}: Launching simulation environment: {launch_file} ===")
        launch_process = subprocess.Popen(
            ['roslaunch', 'data_challenge_simulator', launch_file],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
            env=task_env
        )

        try:
            time.sleep(time_wait)

            print(f"[INFO] Running task script: {task_script}")
            env = os.environ.copy()

            env['SCORE_FILE'] = score_file

            EXAMPLES_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
            PKG_ROOT = os.path.dirname(EXAMPLES_DIR)
            env['PYTHONPATH'] = PKG_ROOT + os.pathsep + env.get('PYTHONPATH', '')

            try:
                if os.path.exists(score_file):
                    os.remove(score_file)
            except Exception:
                pass
            
            cmd = ['python3', task_script,'--seed', str(seed),]
            task_process = subprocess.Popen(cmd, env=env)

            task_process.wait()
            print("[INFO] Task script exited, reading final score for this round...")

            score = read_score_from_file(score_file)
            if score is None:
                valid_cnt = len(scores)
                avg = (sum(scores) / valid_cnt) if valid_cnt > 0 else 0.0
                print(
                    f"{CYAN}[RESULT]{RESET} "
                    f"Round {YELLOW}{cycle_idx}{RESET}: {YELLOW}No score (reset round){RESET} | "
                    f"Average score: {GREEN}{avg:.2f}{RESET} (valid {valid_cnt} rounds)"
                )

                detail = try_read_detail_json(score_file)
                if detail and isinstance(detail.get("components"), dict):
                    comps = detail["components"]
                    for k, v in comps.items():
                        if isinstance(v, (int, float)):
                            components_sum[k] = components_sum.get(k, 0.0) + float(v)

                write_score_json(score_json_path, task_id, scores, components_sum)

            else:
                scores.append(score)
                valid_cnt = len(scores)
                avg = sum(scores) / valid_cnt if valid_cnt > 0 else 0.0

                detail = try_read_detail_json(score_file)
                if detail and isinstance(detail.get("components"), dict):
                    comps = detail["components"]
                    for k, v in comps.items():
                        if isinstance(v, (int, float)):
                            components_sum[k] = components_sum.get(k, 0.0) + float(v)

                write_score_json(score_json_path, task_id, scores, components_sum)

                print(
                    f"{CYAN}[RESULT]{RESET} "
                    f"Round {YELLOW}{cycle_idx}{RESET}: Score: {GREEN}{score}{RESET} | "
                    f"Average score: {GREEN}{avg:.2f}{RESET} (valid {valid_cnt} rounds)"
                )

        except KeyboardInterrupt:
            print("\n[INFO] Received interrupt signal, exiting loop...")
            break

        finally:
            try:
                write_score_json(score_json_path, task_id, scores, components_sum)

                os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            time.sleep(1)
            print("[INFO] Simulation environment closed.")

        cycle_idx += 1
    os.killpg(os.getpgid(roscore_process.pid), signal.SIGTERM)

def main(headless,task_id,time_wait):
    if task_id is None:
        while task_id not in [1, 2, 3]:
            print("========== Model Inference ==========")
            print("Please select the task number (1-3):")
            print("1: Task 1 —— Toy arrangement")
            print("2: Task 2 —— Express parcel weighing")
            print("3: Task 3 —— Conveyor belt sorting")
            try:
                task_id = int(input("Please enter the task number (1-3): ").strip())
            except Exception:
                task_id = None

    run_task(task_id,headless,time_wait)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", default=False, help="Use headless mode")
    parser.add_argument("--task_id", type=int, choices=[1, 2, 3], help="Task number (1-3)")
    parser.add_argument("--time_wait", type=float, default=5, help="Time to wait for the simulation environment to start")
    args = parser.parse_args()
    headless = args.headless
    task_id = args.task_id
    time_wait = args.time_wait
    main(headless,task_id,time_wait)
