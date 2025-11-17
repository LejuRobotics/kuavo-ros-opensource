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

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


GREEN = "\033[92m"
CYAN  = "\033[96m"
YELLOW= "\033[93m"
RESET = "\033[0m"

cfg1 = {
    "rules": [
        # ---- marker1：位置 + 颜色 ----
        {
            "select": ".//body[@name='marker1']",
            "attributes": {
                "pos": {  # 基于 (0.52, -0.50, 0.8721)
                    "per_dim": [
                        {"uniform": [0.36, 0.52]},    # x
                        {"uniform": [-0.05, 0.08]},  # y
                        {"set": 0.8721}               # z 固定（也可用小范围 uniform）
                    ]
                }
            }
        },
        {
            "select": ".//body[@name='marker1']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}  # 随机 RGB，alpha=1
            }
        },

        # ---- marker2：位置 + 颜色 ----
        {
            "select": ".//body[@name='marker2']",
            "attributes": {
                "pos": {  # 基于 (0.42, -0.03, 0.8721)
                    "per_dim": [
                        {"uniform": [0.35, 0.54]},    # x
                        {"uniform": [-0.56, -0.36]},   # y
                        {"set": 0.8721}               # z 固定
                    ]
                }
            }
        },
        {
            "select": ".//body[@name='marker2']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- box_grab 两个几何的颜色 ----
        {
            "select": ".//geom[@name='box_on_belt']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },
        {
            "select": ".//geom[@name='box_base_disk']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- 光照与视角 ----
        {
            "select": ".//visual/headlight",
            "attributes": {
                "diffuse": {"uniform": [0.2, 0.6], "ndim": 3},
                "ambient": {"per_dim": [
                {"uniform": [0.0, 0.3]},
                {"uniform": [0.0, 0.3]},
                {"uniform": [0.0, 0.3]}
                ]},
                "specular": {"uniform": [0.0, 0.2], "ndim": 3}
            }
        }
    ]
}

cfg2 = {
    "rules": [
        # ---- marker1：位置 + 颜色 ----
        {
            "select": ".//body[@name='marker1']",
            "attributes": {
                "pos": {  # 基于 (0.52, -0.50, 0.8721)
                    "per_dim": [
                        {"uniform": [0.30, 0.50]},    # x
                        {"uniform": [0.05, 0.20]},  # y
                        {"set": 0.89}               # z 固定
                    ]
                }
            }
        },
        {
            "select": ".//body[@name='marker1']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}  # 随机 RGB，alpha=1
            }
        },

        # ---- marker2：位置 + 颜色 ----
        {
            "select": ".//body[@name='marker2']",
            "attributes": {
                "pos": {  # 基于 (0.42, -0.03, 0.8721)
                    "per_dim": [
                        {"uniform": [0.3, 0.50]},    # x
                        {"uniform": [0.4, 0.65]},   # y
                        {"set": 0.871}               # z 固定
                    ]
                }
            }
        },
        {
            "select": ".//body[@name='marker2']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- box_grab 两个几何的颜色 ----
        {
            "select": ".//geom[@name='box_on_belt']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },
        {
            "select": ".//geom[@name='box_base_disk']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- 光照与视角 ----
        {
            "select": ".//visual/headlight",
            "attributes": {
                "diffuse": {"uniform": [0.2, 0.6], "ndim": 3},
                "ambient": {"per_dim": [
                {"uniform": [0.0, 0.3]},
                {"uniform": [0.0, 0.3]},
                {"uniform": [0.0, 0.3]}
                ]},
                "specular": {"uniform": [0.0, 0.2], "ndim": 3}
            }
        }
    ]
}

cfg3 = {
    "rules": [
        {
            "select": ".//body[@name='table']/geom[@name='table_top']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        {
            "select": ".//body[@name='marker1']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- 光照与视角 ----
        {
            "select": ".//visual/headlight",
            "attributes": {
                "diffuse": {"uniform": [0.1, 0.8], "ndim": 3},
                "ambient": {"per_dim": [
                {"uniform": [0.0, 0.5]},
                {"uniform": [0.0, 0.5]},
                {"uniform": [0.0, 0.5]}
                ]},
                "specular": {"uniform": [0.0, 0.4], "ndim": 3}
            }
        }
    ]
}

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

    # 1) 总分平均（按有效轮次）
    valid_scores = [s for s in scores if isinstance(s, (int, float))]
    valid_cycles = len(valid_scores)
    avg_total = (sum(valid_scores) / valid_cycles) if valid_cycles else 0.0

    # 2) 获取 keys + process_keys
    keys = set(comp_sum.keys())
    process_keys, time_key = [], "time"
    try:
        catalog_path = os.path.join(os.path.dirname(path), "components_catalog.json")
        if os.path.exists(catalog_path):
            with open(catalog_path, "r") as cf:
                catalog = json.load(cf)
            expected = catalog.get(str(task_id)) or catalog.get(task_id) or []
            keys |= set(expected)
            # 从 catalog 里分离出 time 和 process 部分
            process_keys = [k for k in expected if k.lower() != "time"]
            if "time" in expected:
                time_key = "time"
    except Exception:
        pass

    # 3) 原子项平均
    denom = valid_cycles if valid_cycles else 1
    components_avg = {}
    for k in sorted(keys):
        total_for_k = float(comp_sum.get(k, 0.0))
        components_avg[k] = total_for_k / denom

    # 4) 聚合 process
    process_avg = sum(components_avg.get(k, 0.0) for k in process_keys)
    for k in process_keys:  # 删除原子项
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

    # === 无限循环，直到 Ctrl+C 结束 ===
def run_task(task_id: int, headless: bool):

    print("[INFO] 启动 roscore...")
    roscore_process = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(1)

        # 配置虚拟屏幕参数
    if headless:
        # # 设置 DISPLAY 环境变量
        task_env = os.environ.copy()
        # task_env["DISPLAY"] = display_num
        task_env["MUJOCO_HEADLESS"] = "1"
    else:
        task_env = os.environ.copy()  # 不改变 DISPLAY

    task_script = os.path.join(SCRIPT_DIR, f"eval{task_id}.py")
    launch_file = f"load_kuavo_mujoco_sim{task_id}.launch"

    if not os.path.exists(task_script):
        print(f"[ERROR] 任务脚本不存在：{task_script}")
        sys.exit(1)

    # 分数文件与历史文件
    scores_dir = os.path.join(SCRIPT_DIR, "scores")
    score_file = os.path.join(scores_dir, f"score_task{task_id}_last.txt")

    pre_run_cleanup(scores_dir, score_file)
    # 统计所有轮次的分数
    scores = []

    components_sum = {}        # 动态累计不同键的总和
    score_json_path = os.path.join(scores_dir, "score.json")
    os.makedirs(scores_dir, exist_ok=True)




    scene_map = {
        1: "scene1.xml",
        2: "scene2.xml",
        3: "scene3.xml",
        4: "scene4.xml",
    }

    scene_file = scene_map[task_id]

    scene_path = f"/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/{scene_file}"

    # === 无限循环，直到 Ctrl+C 结束 ===
    cycle_idx = 1
    while True:
        seed = 123123 + cycle_idx

        if task_id==1 or task_id==2 or task_id==3:

            cfg_map = {
            1: cfg1,
            2: cfg2,
            3: cfg3,
            # 4: cfg4,
            }

            cfg = cfg_map[task_id]

            n_changed = randomize_mjcf(
                in_path=scene_path,
                out_path=scene_path,
                config=cfg,
                seed = seed
            )

        print(f"\n[INFO] === 第 {cycle_idx} 轮：启动仿真环境：{launch_file} ===")
        launch_process = subprocess.Popen(
            ['roslaunch', 'data_challenge_simulator', launch_file],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,  # 独立进程组，便于整体 SIGTERM
            env=task_env
        )

        try:
            # 等仿真起来
            time.sleep(2)

            print(f"[INFO] 运行任务脚本：{task_script}")
            env = os.environ.copy()
            # 传递分数文件路径给任务脚本
            env['SCORE_FILE'] = score_file

            EXAMPLES_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
            PKG_ROOT = os.path.dirname(EXAMPLES_DIR)
            env['PYTHONPATH'] = PKG_ROOT + os.pathsep + env.get('PYTHONPATH', '')

            # 先清空旧分数文件，避免误读上轮
            try:
                if os.path.exists(score_file):
                    os.remove(score_file)
            except Exception:
                pass
            
            cmd = ['python3', task_script,'--seed', str(seed),]
            task_process = subprocess.Popen(cmd, env=env)

            # 等到任务脚本退出（收到 /simulator/reset 后会自行退出）
            task_process.wait()
            print("[INFO] 任务脚本退出，读取本轮最终得分...")

            # ✅ 从 txt 读取分数
            score = read_score_from_file(score_file)
            if score is None:
                # 没有成绩（可能是 reset 轮），不计入平均
                valid_cnt = len(scores)
                avg = (sum(scores) / valid_cnt) if valid_cnt > 0 else 0.0
                print(
                    f"{CYAN}[RESULT]{RESET} "
                    f"第 {YELLOW}{cycle_idx}{RESET} 轮：{YELLOW}无成绩（reset 轮）{RESET} | "
                    f"当前平均分: {GREEN}{avg:.2f}{RESET} （有效 {valid_cnt} 轮）"
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
                    f"第 {YELLOW}{cycle_idx}{RESET} 轮得分: {GREEN}{score}{RESET} | "
                    f"当前平均分: {GREEN}{avg:.2f}{RESET} （有效 {valid_cnt} 轮）"
                )

        except KeyboardInterrupt:
            print("\n[INFO] 收到中断信号，正在退出循环...")
            break

        finally:
            # 关闭仿真环境（roslaunch 进程组）
            try:
                write_score_json(score_json_path, task_id, scores, components_sum)

                os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            time.sleep(1)
            print("[INFO] 仿真环境已关闭。")

        cycle_idx += 1
    os.killpg(os.getpgid(roscore_process.pid), signal.SIGTERM)

def main(headless,task_id):
    if task_id is None:
        while task_id not in [1, 2, 3, 4]:
            print("========== 模型推理 ==========")
            print("请选择任务编号（1-4）：")
            print("1: 任务1 —— 传送带物品分拣")
            print("2: 任务2 —— 传送带物品称重")
            print("3: 任务3 —— 物品翻面")
            print("4: 任务4 —— 货架物品运送")
            try:
                task_id = int(input("请输入任务编号 (1-4): ").strip())
            except Exception:
                task_id = None

    run_task(task_id,headless)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", default=False, help="是否使用无头模式")
    parser.add_argument("--task_id", type=int, choices=[1, 2, 3, 4], help="任务编号 (1-4)")
    args = parser.parse_args()
    headless = args.headless
    task_id = args.task_id
    main(headless,task_id)
