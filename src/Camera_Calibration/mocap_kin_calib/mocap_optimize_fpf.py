#!/usr/bin/env python3
"""Interactive capture -> automatic CTB selection -> fit -> candidate validation."""
from __future__ import annotations

import argparse
import copy
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

import yaml

import mocap_optimize_1010 as optimizer
from mocap_stability import analyze_capture
from robot_variants import default_config_name


SCRIPT_DIR = Path(__file__).resolve().parent
RECORDER = SCRIPT_DIR / "record_joint_poses.py"


class Cancelled(Exception):
    pass


def ask(prompt, choices):
    while True:
        try:
            answer = input(prompt).strip().lower()
        except (EOFError, KeyboardInterrupt):
            raise Cancelled() from None
        if answer in ("", "n", "no"):
            raise Cancelled()
        if answer in choices:
            return answer
        print("请输入 " + "/".join(choices) + "，或 n 退出。")


def select_mocap_source(cfg):
    answer = ask("选择采集设备 [1=青瞳 / 2=动捕 OptiTrack / n=退出，默认 n]: ",
                 ("1", "q", "qingtong", "2", "o", "optitrack", "motive"))
    source = "qingtong" if answer in ("1", "q", "qingtong") else "motive"
    effective = copy.deepcopy(cfg)
    names = ("l_hand", "r_hand", "torso", "l_shoulder")
    for body in effective.get("mocap_bodies", []):
        if body.get("name") in names:
            if source == "qingtong":
                body["topic"] = f"/vrpn_client_node/{body['name']}/pose"
                body["msg_type"] = "PoseStamped"
            print(f"  {body['name']}: {body['topic']}")
    return effective, source


def run_command(command, log_path):
    print("执行: " + shlex.join(str(arg) for arg in command), flush=True)
    with log_path.open("w", encoding="utf-8") as log:
        with subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, bufsize=1) as process:
            try:
                for line in process.stdout:
                    print(line, end="", flush=True)
                    log.write(line)
                    log.flush()
                code = process.wait()
            except BaseException:
                process.terminate()
                process.wait()
                raise
    if code:
        raise RuntimeError(f"步骤失败（退出码 {code}），日志: {log_path}")


def write_yaml(path, cfg):
    path.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")


def recording_config(cfg, stage):
    """Select known fit/validation routes without editing the source YAML."""
    result = copy.deepcopy(cfg)
    calib = result["calibration"]
    counts = []
    for arm in ("left", "right"):
        path = SCRIPT_DIR / "config" / f"teach_{arm}_joint_{stage}.json"
        samples = json.loads(path.read_text(encoding="utf-8"))["samples"]
        if not isinstance(samples, list) or len(samples) < 3:
            raise ValueError(f"teach 文件缺少轨迹点: {path}")
        for sample in samples:
            values = sample.get(f"{arm}_arm_joints")
            if (not isinstance(values, list) or len(values) != 7 or
                    not all(math.isfinite(float(value)) for value in values)):
                raise ValueError(f"teach 文件存在无效的七关节轨迹点: {path}")
        counts.append(len(samples))
        calib[f"teach_{arm}"] = str(path)
    if counts[0] != counts[1]:
        raise ValueError("左右 teach 轨迹点数不一致")
    expected = list(range(1, counts[0] - 1))
    if stage == "fit" and len(expected) != 30:
        raise ValueError("拟合 teach 必须含 30 个采集点及首尾过渡点")
    calib["skip_first_last"] = True
    calib["urdf"] = result["kinematics"]["source_urdf"]
    # The recorder resolves these paths relative to its own script, not YAML.
    for key in ("urdf", "teach_head"):
        if calib.get(key):
            calib[key] = str((RECORDER.parent / Path(calib[key]).expanduser()).resolve())
    return result, expected


def check_capture(path, cfg, *, fit, expected_indices=None):
    capture = json.loads(path.read_text(encoding="utf-8"))
    effective = optimizer._config_for_capture(cfg, capture.get("meta", {}))
    data = optimizer.load_dataset(path, effective, validation_only=not fit)
    indices = [sample["index"] for sample in capture["samples"]]
    if fit:
        expected_indices = effective["kinematics"]["fit_indices"] + effective["kinematics"]["validation_indices"]
    if expected_indices is not None and (len(indices) != len(expected_indices) or
                                         set(indices) != set(expected_indices)):
        raise ValueError(f"{path}: 采集不完整；期望索引 {sorted(expected_indices)}，实际 {indices}")
    print(f"数据检查通过: {path}（{len(indices)} 点）")
    return effective, data


def capture_stage(label, destination, config_path, cfg, expected, run_dir, *, fit, mocap_source):
    print(f"{label} teach: {cfg['calibration']['teach_left']}")
    print(f"{label} teach: {cfg['calibration']['teach_right']}")
    print(f"{label}数据: {destination}；本次采集将记录 {len(expected)} 点。")
    action = ask(f"开始{label}采点？[y=采集 / r=使用已有文件 / n=退出，默认 n]: ",
                 ("y", "yes", "r"))
    if action == "r":
        check_capture(destination, cfg, fit=fit, expected_indices=expected)
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    # A fresh path prevents an interrupted recorder from reusing stale data.
    # Keep incomplete recordings in this run's folder for diagnosis.
    fresh = run_dir / ("capture_fit.new.json" if fit else "capture_vali.new.json")
    run_command([sys.executable, "-u", str(RECORDER), "--config", str(config_path),
                 "--robot-version", str(cfg["robot"]["robot_version"]),
                 "--urdf", cfg["calibration"]["urdf"],
                 "--fixture", "new", "--mocap-source", mocap_source, "--output", str(fresh)],
                run_dir / ("capture_fit.log" if fit else "capture_vali.log"))
    check_capture(fresh, cfg, fit=fit, expected_indices=expected)
    if fit:
        stability = cfg.get("workflow", {}).get("shoulder_stability", {})
        analyze_capture(fresh, **stability)
    # Stage on the destination filesystem, so replacement also works with an
    # external --output-dir. Existing captures survive partial/failed recording.
    with tempfile.NamedTemporaryFile(dir=destination.parent, prefix=".capture_", delete=False) as tmp:
        temporary_path = Path(tmp.name)
        tmp.write(fresh.read_bytes())
    try:
        os.replace(temporary_path, destination)
    finally:
        temporary_path.unlink(missing_ok=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description="一键采集、自动选择 CTB 模式、拟合及验证（默认 ROBOT_VERSION=62）")
    parser.add_argument("--config", type=Path, help="采集话题、CTB/ETK、验收阈值及自动模式门限配置")
    parser.add_argument("--fit-capture", type=Path, default=SCRIPT_DIR / "capture_fit.json")
    parser.add_argument("--validation-capture", type=Path, default=SCRIPT_DIR / "capture_vali.json")
    parser.add_argument("--output-dir", type=Path, help="覆盖候选输出目录；默认使用 YAML output_dir")
    args = parser.parse_args(argv)
    version = os.environ.setdefault("ROBOT_VERSION", "62")
    try:
        default_name = default_config_name(version)
    except ValueError as exc:
        parser.error(str(exc))
    config_path = (args.config or SCRIPT_DIR / "config" / default_name).expanduser().resolve()
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    cfg, mocap_source = select_mocap_source(cfg)
    cfg = optimizer._config_for_capture(cfg, {})
    output_dir = (args.output_dir.expanduser().resolve() if args.output_dir else
                  optimizer._resolve_config_path(config_path, cfg["kinematics"]["output_dir"]))
    cfg["kinematics"]["output_dir"] = str(output_dir)
    threshold = cfg["kinematics"].get("validation", {}).get("max_position_error_mm")
    if threshold is None or not math.isfinite(float(threshold)) or float(threshold) <= 0:
        raise ValueError("一键流程需要有限正数 validation.max_position_error_mm 作为验收阈值")
    fit_path = args.fit_capture.expanduser().resolve()
    validation_path = args.validation_capture.expanduser().resolve()
    if fit_path == validation_path:
        raise ValueError("拟合和独立验证必须使用两个不同的采集文件")
    fit_cfg, fit_expected = recording_config(cfg, "fit")
    validation_cfg, validation_expected = recording_config(cfg, "validation")
    if set(fit_expected) != set(cfg["kinematics"]["fit_indices"] + cfg["kinematics"]["validation_indices"]):
        raise ValueError("拟合 teach 索引与配置中的 25/5 点分组不匹配")
    target = Path(cfg["kinematics"]["source_urdf"])
    source = optimizer.write_urdf.resolve_fit_source(target)
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    output_dir.mkdir(parents=True, exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("fpf_%Y%m%d_%H%M%S_"), dir=output_dir))
    print(f"ROBOT_VERSION={version}；配置: {config_path}\n"
          f"拟合基准: {source}\n运行模型替换目标: {target}\n本次记录目录: {run_dir}")
    fit_config = run_dir / "fit.yaml"
    validation_config = run_dir / "validation.yaml"
    write_yaml(fit_config, fit_cfg)
    write_yaml(validation_config, validation_cfg)
    capture_stage("拟合", fit_path, fit_config, fit_cfg, fit_expected, run_dir,
                  fit=True, mocap_source=mocap_source)
    stability = analyze_capture(fit_path, **cfg.get("workflow", {}).get("shoulder_stability", {}))
    qingtong = stability["mode"] == "qingtong"
    limits = stability["thresholds"]
    print(f"自动模式门限: 最大点对差 > {limits['max_translation_mm']:g} mm "
          f"或 > {limits['max_rotation_deg']:g} deg")
    for arm, metrics in stability["arms"].items():
        print(f"torso—shoulder {arm}: 点对差均值 {metrics['mean_translation_mm']:.3f} mm / "
              f"{metrics['mean_rotation_deg']:.3f} deg；最大 {metrics['max_translation_mm']:.3f} mm / "
              f"{metrics['max_rotation_deg']:.3f} deg")
    for stage_cfg in (fit_cfg, validation_cfg):
        stage_cfg["kinematics"]["ctb"]["mode"] = "per_sample" if qingtong else "shared"
        stage_cfg["kinematics"].setdefault("validation", {})["qingtong"] = qingtong
    write_yaml(fit_config, fit_cfg)
    write_yaml(validation_config, validation_cfg)
    (run_dir / "stability.json").write_text(json.dumps(stability, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    angle_bound = fit_cfg["kinematics"]["ctb"].get("rotation_bound_deg", 3.0)
    print(f"自动选择: {'qingtong，逐点 CTB 相对 I ≤ ' + str(angle_bound) + '°' if qingtong else '共享 CTB 旋转'}；拟合和独立验证使用相同模式。")
    capture_stage("验证", validation_path, validation_config, validation_cfg,
                  validation_expected, run_dir, fit=False, mocap_source=mocap_source)
    if fit_path.read_bytes() == validation_path.read_bytes():
        raise ValueError("验证文件与拟合文件完全相同，请采集独立验证姿态")
    candidate = output_dir / cfg["kinematics"]["urdf_final_name"]
    ask(f"开始拟合并导出候选？（将更新 {candidate} 及偏置 JSON）[y/n，默认 n]: ", ("y", "yes"))
    run_command(["bash", str(SCRIPT_DIR / "mocap_optimize.sh"), str(fit_path),
                 "--config", str(fit_config), "--urdf", str(source),
                 "--output-urdf", "--force-output", "--no-install"], run_dir / "fit.log")
    if not candidate.is_file():
        raise RuntimeError(f"拟合未生成候选: {candidate}")
    ask("开始验证候选 URDF？[y/n，默认 n]: ", ("y", "yes"))
    # The one-click workflow always refits reference angles, leaving geometry
    # frozen. qingtong in the same effective config selects bounded per-pose CTB.
    run_command(["bash", str(SCRIPT_DIR / "mocap_optimize.sh"), str(validation_path),
                 "--config", str(validation_config), "--validate-urdf", str(candidate),
                 "--no-freeze-frame-angles"], run_dir / "validation.log")
    offsets = json.loads((output_dir / "kinematic_offsets.json").read_text(encoding="utf-8"))
    if (hashlib.sha256(source.read_bytes()).hexdigest() != source_hash or
            offsets["source_urdf_sha256"] != source_hash):
        raise RuntimeError("拟合基准发生变化，结果已保留，停止运行模型替换")
    print(f"拟合和验证数据验收均通过。候选: {candidate}\n日志及自动模式判定: {run_dir}")
    optimizer._confirm_apply_urdf(source, candidate, target)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (Cancelled, KeyboardInterrupt):
        print("已停止一键流程；已完成的数据和候选结果保留。")
        raise SystemExit(130)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
