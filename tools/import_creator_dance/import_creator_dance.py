#!/usr/bin/env python3
"""Import Leju Creator exported dance packages into humanoid_controllers."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import sys
import unicodedata
import zipfile
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
HUMANOID_CONTROLLERS = REPO_ROOT / "src/humanoid-control/humanoid_controllers"
CONTROLLERS_CONFIG_ROOT = HUMANOID_CONTROLLERS / "config"
NETWORK_MODEL_DIR = HUMANOID_CONTROLLERS / "model/networks"


class CreatorYamlLoader(yaml.SafeLoader):
    """Safe-ish loader for Isaac Lab YAML tags used in creator exports."""


def _construct_python_tuple(loader: yaml.Loader, node: yaml.Node) -> tuple[Any, ...]:
    return tuple(loader.construct_sequence(node))


def _construct_python_slice(loader: yaml.Loader, node: yaml.Node) -> slice:
    values = loader.construct_sequence(node)
    return slice(*values)


CreatorYamlLoader.add_constructor("tag:yaml.org,2002:python/tuple", _construct_python_tuple)
CreatorYamlLoader.add_constructor(
    "tag:yaml.org,2002:python/object/apply:builtins.slice", _construct_python_slice
)


class ImportErrorWithHint(RuntimeError):
    pass


def normalize_stem(stem: str) -> str:
    """统一 zip 去后缀名(=展示名/映射表 key)的字节形态。

    macOS 打的 zip 中文是 NFD 分解形, 前端 json 里通常是 NFC, 不归一化会让
    handler 写的 map key 与 joy 查表的 dance_name 字节不同而静默 miss。
    """
    return unicodedata.normalize("NFC", stem).strip()


def make_controller_name(stem: str) -> str:
    """由(已 normalize 的)展示名派生纯 ASCII 控制器名。

    控制器名同时是 ROS 服务名段和 info/onnx/csv 文件前缀, 必须是合法标识符。
    `dance_` + ascii 净化片段(无则省略) + `_` + sha1 前 10 位:
      - 对同一展示名确定 -> 可重入覆盖;
      - 不同中文名净化后都塌成空, 靠 hash 区分, 不碰撞。
    """
    digest = hashlib.sha1(stem.encode("utf-8")).hexdigest()[:10]
    slug = re.sub(r"[^a-z0-9]+", "_", stem.lower()).strip("_")[:24]
    return f"dance_{slug}_{digest}" if slug else f"dance_{digest}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Import a Leju Creator dance zip into the current ROBOT_VERSION controller config.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("zip_path", type=Path, help="Path to the Leju Creator export zip.")
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite existing controller files / yaml entry when dance already imported.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and print actions without writing files.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Emit a single-line JSON result to stdout (for ws handler).",
    )
    return parser.parse_args()


def classify_error(message: str) -> str:
    msg = message.lower()
    if "joints" in msg and "creator export has" in msg:
        return "E_JOINT_MISMATCH"
    if "controller already exists" in msg or "already exists:" in msg:
        return "E_BINDING_EXISTS"
    if "zip" in msg and ("invalid" in msg or "missing required" in msg or "does not exist" in msg or "multiple" in msg):
        return "E_BAD_ZIP"
    if "trajectory" in msg and "csv" in msg:
        return "E_BAD_ZIP"
    return "E_GENERIC"


def fail(message: str) -> None:
    raise ImportErrorWithHint(message)


def is_zip_member_usable(name: str) -> bool:
    parts = [part for part in name.replace("\\", "/").split("/") if part]
    if not parts or "__MACOSX" in parts:
        return False
    return not any(part.startswith("._") or part.startswith(".") for part in parts)


def zip_member_basename(name: str) -> str:
    parts = [part for part in name.replace("\\", "/").split("/") if part]
    return parts[-1] if parts else ""


def normalized_zip_name(name: str) -> str:
    return "/".join(part for part in name.replace("\\", "/").split("/") if part)


def find_unique_by_ext(
    zip_file: zipfile.ZipFile, exts: tuple[str, ...], label: str, required: bool = True
) -> str | None:
    """按扩展名(而非固定文件名)定位唯一成员。

    Creator 导出里 yaml / onnx / csv 的文件名是 md5 随机串, 不能写死名字;
    约定 zip 内每类各只有一个, 所以按扩展名取唯一即可。
    """
    exts = tuple(e.lower() for e in exts)
    matches = [
        info.filename
        for info in zip_file.infolist()
        if not info.is_dir()
        and is_zip_member_usable(info.filename)
        and zip_member_basename(info.filename).lower().endswith(exts)
    ]
    if not matches:
        if required:
            fail(f"zip is missing required {label} file (expected one *{exts[0]})")
        return None
    if len(matches) > 1:
        fail(f"zip contains multiple {label} files, expected exactly one: {matches}")
    return matches[0]


def load_creator_env(zip_path: Path) -> tuple[dict[str, Any], dict[str, str]]:
    if not zip_path.is_file():
        fail(f"zip path does not exist: {zip_path}")
    try:
        with zipfile.ZipFile(zip_path) as zf:
            # 文件名不固定(md5 随机), 按扩展名各取唯一一个。
            # 注意: 即使 zip 里带 wav 也忽略, 音乐完全由前端独立通路管理。
            members = {
                "env": find_unique_by_ext(zf, (".yaml", ".yml"), "env yaml"),
                "model": find_unique_by_ext(zf, (".onnx",), "model onnx"),
                "trajectory": find_unique_by_ext(zf, (".csv",), "trajectory csv"),
            }
            env = yaml.load(zf.read(members["env"]), Loader=CreatorYamlLoader)
            if not isinstance(env, dict):
                fail("env yaml did not parse as a YAML mapping")
            return env, {key: value for key, value in members.items() if value}
    except zipfile.BadZipFile as exc:
        fail(f"invalid zip file: {exc}")
    except yaml.YAMLError as exc:
        fail(f"failed to parse env yaml: {exc}")


def get_robot_version() -> str:
    robot_version = os.environ.get("ROBOT_VERSION", "").strip()
    if not robot_version:
        fail("ROBOT_VERSION is not set; please export ROBOT_VERSION before importing")
    if not re.fullmatch(r"\d+", robot_version):
        fail(f"ROBOT_VERSION must be numeric, got: {robot_version}")
    return robot_version


def require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        fail(f"{label} must be a mapping")
    return value


def require_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        fail(f"{label} must be a list")
    return value


def get_nested(data: dict[str, Any], path: str) -> Any:
    cur: Any = data
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            fail(f"env.yaml is missing required field: {path}")
        cur = cur[part]
    return cur


def regex_value_for_joint(mapping: dict[str, Any], joint_name: str, field_name: str) -> float:
    if joint_name in mapping:
        return float(mapping[joint_name])
    for pattern, value in mapping.items():
        if re.fullmatch(str(pattern), joint_name):
            return float(value)
    fail(f"{field_name} has no value for joint: {joint_name}")


def optional_regex_values(mapping: Any, joint_names: list[str], field_name: str) -> list[float] | None:
    if mapping is None:
        return None
    mapping = require_mapping(mapping, field_name)
    return [regex_value_for_joint(mapping, joint, field_name) for joint in joint_names]


def default_torque_limit(joint_name: str) -> float:
    if joint_name == "waist_yaw_joint" or joint_name == "waist_joint":
        return 64.0
    if re.fullmatch(r"leg_[lr][12]_joint", joint_name):
        return 120.0
    if re.fullmatch(r"leg_[lr]3_joint", joint_name):
        return 56.0
    if re.fullmatch(r"leg_[lr]4_joint", joint_name):
        return 150.0
    if re.fullmatch(r"leg_[lr][56]_joint", joint_name):
        return 70.2
    if re.fullmatch(r"zarm_[lr]1_joint", joint_name):
        return 11.28
    if joint_name.startswith("zarm_"):
        return 29.6
    return 100.0


def collect_joint_config(env: dict[str, Any]) -> dict[str, Any]:
    action_cfg = require_mapping(get_nested(env, "actions.joint_pos"), "actions.joint_pos")
    action_joints = [str(item) for item in require_list(action_cfg.get("joint_names"), "actions.joint_pos.joint_names")]
    if not action_joints:
        fail("actions.joint_pos.joint_names is empty")

    policy_joint_names = get_nested(env, "observations.policy.joint_pos.params.asset_cfg.joint_names")
    policy_joints = [str(item) for item in require_list(policy_joint_names, "observations.policy joint_names")]
    if policy_joints != action_joints:
        fail("policy joint_names and action joint_names differ; refusing to guess joint order")

    init_joint_pos = require_mapping(get_nested(env, "scene.robot.init_state.joint_pos"), "scene.robot.init_state.joint_pos")
    actuators = require_mapping(get_nested(env, "scene.robot.actuators.motor"), "scene.robot.actuators.motor")
    stiffness = require_mapping(actuators.get("stiffness"), "scene.robot.actuators.motor.stiffness")
    damping = require_mapping(actuators.get("damping"), "scene.robot.actuators.motor.damping")
    action_scale = require_mapping(action_cfg.get("scale"), "actions.joint_pos.scale")

    torque_limits = optional_regex_values(actuators.get("effort_limit_sim"), action_joints, "effort_limit_sim")
    if torque_limits is None:
        torque_limits = optional_regex_values(actuators.get("effort_limit"), action_joints, "effort_limit")
    if torque_limits is None:
        torque_limits = [default_torque_limit(joint) for joint in action_joints]

    return {
        "joint_names": action_joints,
        "default_joint_pos": [float(init_joint_pos.get(joint, 0.0)) for joint in action_joints],
        "joint_kp": [regex_value_for_joint(stiffness, joint, "stiffness") for joint in action_joints],
        "joint_kd": [regex_value_for_joint(damping, joint, "damping") for joint in action_joints],
        "torque_limits": torque_limits,
        "action_scale": [regex_value_for_joint(action_scale, joint, "actions.joint_pos.scale") for joint in action_joints],
    }


def get_residual_action(env: dict[str, Any]) -> bool:
    action_cfg = require_mapping(get_nested(env, "actions.joint_pos"), "actions.joint_pos")
    class_type = str(action_cfg.get("class_type", ""))
    if "Residual" in class_type:
        return True
    if "residual" in action_cfg:
        return bool(action_cfg["residual"])
    return False


def matrix_block(name: str, values: list[float], comments: list[str] | None = None) -> str:
    lines = [name, "{"]
    for idx, value in enumerate(values):
        rendered = f"{value:.12g}"
        comment = f"   ; {comments[idx]}" if comments else ""
        lines.append(f"    ({idx},0)   {rendered}{comment}")
    lines.append("}")
    return "\n".join(lines)


def single_input_data_block(num_joints: int) -> str:
    entries = [
        ("motion_command", num_joints * 2),
        ("motion_target_pos", 1),
        ("motion_anchor_ori_b", 6),
        ("projected_gravity", 3),
        ("base_ang_vel", 3),
        ("joint_pos", num_joints),
        ("joint_vel", num_joints),
        ("actions", num_joints),
    ]
    lines = ["singleInputData", "{"]
    for name, count in entries:
        lines.extend(
            [
                f"    {name}",
                "    {",
                "        startIdx    0",
                f"        numIdx      {count}",
                "        obsScales   1.0",
                "    }",
            ]
        )
    lines.append("}")
    return "\n".join(lines)


def replace_scalar_value(text: str, key: str, value: str) -> str:
    pattern = re.compile(rf"(?m)^(\s*{re.escape(key)}\s+)(\S+)(.*)$")

    def repl(match: re.Match[str]) -> str:
        return f"{match.group(1)}{value}{match.group(3)}"

    new_text, count = pattern.subn(repl, text, count=1)
    if count == 0:
        fail(f"template info is missing scalar field: {key}")
    return new_text


def replace_matrix_block(text: str, key: str, block: str) -> str:
    pattern = re.compile(rf"(?ms)^{re.escape(key)}\s*\{{.*?^\}}")
    new_text, count = pattern.subn(block, text, count=1)
    if count == 0:
        fail(f"template info is missing matrix block: {key}")
    return new_text


def build_info(env: dict[str, Any], dance_name: str, model_file: str, csv_file: str, template_info: Path) -> str:
    joint_cfg = collect_joint_config(env)
    joint_names = joint_cfg["joint_names"]
    num_joints = len(joint_names)
    sim_dt = float(get_nested(env, "sim.dt"))
    decimation = int(env.get("decimation", 1))
    trajectory_dt = sim_dt * decimation
    num_single_obs = num_joints * 5 + 13

    text = template_info.read_text(encoding="utf-8")
    text = replace_scalar_value(text, "trajectoryCSVFile", csv_file)
    text = replace_scalar_value(text, "trajectoryTimeStep", f"{trajectory_dt:.12g}")
    text = replace_scalar_value(text, "holdFrameIndex", "-1")
    text = replace_scalar_value(text, "networkModelFile", f"/{model_file}")
    text = replace_scalar_value(text, "numSingleObs", str(num_single_obs))
    text = replace_scalar_value(text, "residualAction", str(get_residual_action(env)).lower())

    # Deployment base state and CSP/Ruiwo flags stay inherited from the
    # target robot template. Policy gains/scales come from env.yaml.
    text = replace_matrix_block(text, "JointControlMode", matrix_block("JointControlMode", [0.0] * num_joints, joint_names))
    text = replace_matrix_block(text, "jointKp", matrix_block("jointKp", joint_cfg["joint_kp"], joint_names))
    text = replace_matrix_block(text, "jointKd", matrix_block("jointKd", joint_cfg["joint_kd"], joint_names))
    text = replace_matrix_block(text, "actionScaleTest", matrix_block("actionScaleTest", joint_cfg["action_scale"], joint_names))
    text = replace_matrix_block(text, "singleInputData", single_input_data_block(num_joints))
    return text


def count_info_matrix_entries(info_path: Path, matrix_name: str) -> int | None:
    if not info_path.is_file():
        return None
    text = info_path.read_text(encoding="utf-8", errors="ignore")
    match = re.search(rf"(?m)^\s*{re.escape(matrix_name)}\s*\{{(?P<body>.*?)^\s*\}}", text, re.S)
    if not match:
        return None
    return len(re.findall(r"^\s*\(\s*\d+\s*,\s*0\s*\)", match.group("body"), re.M))


def validate_target_joint_count(version_dir: Path, imported_count: int) -> None:
    template = version_dir / "rl/dance_param.info"
    existing_count = count_info_matrix_entries(template, "defaultJointState")
    if existing_count is not None and existing_count != imported_count:
        fail(
            f"target {template} has {existing_count} joints, but creator export has {imported_count}; "
            "check ROBOT_VERSION or use a matching creator package"
        )


def read_yaml_file(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        fail(f"failed to parse YAML {path}: {exc}")
    if not isinstance(data, dict):
        fail(f"YAML file must contain a mapping: {path}")
    return data


def controller_exists(controllers_yaml: Path, dance_name: str) -> tuple[bool, dict[str, Any]]:
    data = read_yaml_file(controllers_yaml)
    controllers = data.setdefault("controllers", [])
    if not isinstance(controllers, list):
        fail(f"{controllers_yaml} field 'controllers' must be a list")
    return any(isinstance(item, dict) and item.get("name") == dance_name for item in controllers), data


def append_controller(controllers_yaml: Path, dance_name: str, info_name: str, dry_run: bool) -> None:
    block = "\n" + controller_block(dance_name, info_name)
    if dry_run:
        return
    with controllers_yaml.open("a", encoding="utf-8") as f:
        if controllers_yaml.stat().st_size and not controllers_yaml.read_text(encoding="utf-8").endswith("\n"):
            f.write("\n")
        f.write(block)


def controller_block(dance_name: str, info_name: str) -> str:
    return (
        f"  - name: \"{dance_name}\"\n"
        "    class: \"DANCE_CONTROLLER\"\n"
        "    type: \"DANCE_CONTROLLER\"\n"
        f"    config_file: \"rl/{info_name}\"\n"
        "    enabled: true\n"
    )


def rewrite_controller(data: dict[str, Any], controllers_yaml: Path, dance_name: str, info_name: str, dry_run: bool) -> None:
    if dry_run:
        return
    text = controllers_yaml.read_text(encoding="utf-8")
    name_pattern = re.escape(dance_name)
    pattern = re.compile(
        rf"(?ms)^  - name:\s*[\"']?{name_pattern}[\"']?.*?(?=^  - name:|\Z)"
    )
    replacement = controller_block(dance_name, info_name)
    new_text, count = pattern.subn(replacement, text, count=1)
    if count == 0:
        append_controller(controllers_yaml, dance_name, info_name, dry_run=False)
        return
    controllers_yaml.write_text(new_text, encoding="utf-8")


def check_file_conflict(path: Path, force: bool, label: str) -> None:
    if path.exists() and not force:
        fail(f"{label} already exists: {path}; pass --force to overwrite")


def safe_copy_from_zip(zip_path: Path, member: str, destination: Path, dry_run: bool) -> None:
    if dry_run:
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path) as zf, zf.open(member) as src, destination.open("wb") as dst:
        shutil.copyfileobj(src, dst)


def validate_trajectory_shape(zip_path: Path, member: str, num_joints: int) -> bool:
    expected = {num_joints * 2, 3 + 4 + num_joints * 2, 4 + num_joints * 2}
    saw_header = False
    with zipfile.ZipFile(zip_path) as zf, zf.open(member) as src:
        for raw_line in src:
            line = raw_line.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            delimiter = "," if "," in line else None
            parts = line.split(delimiter) if delimiter else line.split()
            parts = [part for part in parts if part]
            try:
                [float(part) for part in parts]
            except ValueError:
                if saw_header:
                    fail("trajectory.csv contains more than one non-numeric row")
                saw_header = True
                continue
            if len(parts) not in expected:
                fail(
                    f"trajectory.csv has {len(parts)} columns; expected one of {sorted(expected)} for {num_joints} joints"
                )
            return saw_header
    fail("trajectory.csv is empty")


def main() -> int:
    args = parse_args()
    try:
        # display_name = zip 去后缀名(可中文, 作展示名 / 映射表 key);
        # controller_name = 由它派生的纯 ASCII 控制器名(ROS 服务名段 + 文件前缀)。
        # 中英文映射关系由 ws handler 写入 dance_name_map.json, 脚本只负责落盘控制器文件。
        display_name = normalize_stem(args.zip_path.stem)
        if not display_name:
            fail("zip filename has no stem; cannot derive dance_name")
        controller_name = make_controller_name(display_name)

        robot_version = get_robot_version()
        version_dir = CONTROLLERS_CONFIG_ROOT / f"kuavo_v{robot_version}"
        if not version_dir.is_dir():
            fail(f"target robot config directory does not exist: {version_dir}")

        controllers_yaml = version_dir / "rl_controllers.yaml"
        if not controllers_yaml.is_file():
            fail(f"target rl_controllers.yaml does not exist: {controllers_yaml}")

        env, members = load_creator_env(args.zip_path)
        joint_count = len(collect_joint_config(env)["joint_names"])
        validate_target_joint_count(version_dir, joint_count)
        validate_trajectory_shape(args.zip_path, members["trajectory"], joint_count)

        info_name = f"dance_param_{controller_name}.info"
        onnx_name = f"{controller_name}.onnx"
        csv_name = f"{controller_name}.csv"

        info_path = version_dir / "rl" / info_name
        onnx_path = NETWORK_MODEL_DIR / onnx_name
        csv_path = version_dir / "rl" / csv_name

        check_file_conflict(info_path, args.force, "info file")
        check_file_conflict(onnx_path, args.force, "onnx file")
        check_file_conflict(csv_path, args.force, "csv file")

        exists, controllers_data = controller_exists(controllers_yaml, controller_name)
        if exists and not args.force:
            fail(f"controller already exists in {controllers_yaml}: {controller_name}; pass --force to update it")

        template_info = version_dir / "rl/dance_param.info"
        if not template_info.is_file():
            fail(f"template dance info does not exist: {template_info}")
        info_text = build_info(env, controller_name, onnx_name, csv_name, template_info)

        if not args.dry_run:
            info_path.parent.mkdir(parents=True, exist_ok=True)
            info_path.write_text(info_text, encoding="utf-8")
            safe_copy_from_zip(args.zip_path, members["model"], onnx_path, args.dry_run)
            safe_copy_from_zip(args.zip_path, members["trajectory"], csv_path, args.dry_run)

        if exists:
            rewrite_controller(controllers_data, controllers_yaml, controller_name, info_name, args.dry_run)
        else:
            append_controller(controllers_yaml, controller_name, info_name, args.dry_run)

        if args.json:
            result = {
                "code": 0,
                "dry_run": bool(args.dry_run),
                "robot_version": robot_version,
                "dance_name": display_name,
                "controller": controller_name,
                "info_path": str(info_path),
                "onnx_path": str(onnx_path),
                "csv_path": str(csv_path),
                "controller_existed": bool(exists),
            }
            print(json.dumps(result, ensure_ascii=False))
            return 0
        print("Import validation passed." if args.dry_run else "Import complete.")
        print(f"  ROBOT_VERSION: {robot_version}")
        print(f"  Dance name:  {display_name}")
        print(f"  Controller:  {controller_name}")
        print(f"  Info: {info_path}")
        print(f"  ONNX: {onnx_path}")
        print(f"  CSV: {csv_path}")
        return 0
    except ImportErrorWithHint as exc:
        if args.json:
            print(
                json.dumps(
                    {"code": 2, "error": str(exc), "error_kind": classify_error(str(exc))},
                    ensure_ascii=False,
                )
            )
        else:
            print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
