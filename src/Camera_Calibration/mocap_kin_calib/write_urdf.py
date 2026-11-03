#!/usr/bin/env python3
"""Back up and atomically install a verified full-body URDF candidate.

The default action is a dry-run.  ``--apply`` is required for replacement.
The candidate must have the same full robot structure as the target and may
change origins only on the fourteen arm joints.  This prevents an arm-only
URDF, another robot version, or a second unrelated edit from replacing the
runtime model.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import stat
import tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path


ARM_JOINTS = {
    *(f"zarm_l{i}_joint" for i in range(1, 8)),
    *(f"zarm_r{i}_joint" for i in range(1, 8)),
}
CALIBRATION_COMMENT_PREFIX = "mocap_kin_calib:"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _tree(path: Path):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True, insert_pis=True))
    tree = ET.parse(path, parser=parser)
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError(f"{path} root tag is {root.tag!r}, expected 'robot'")
    return tree, root


def nominal_backup_path(target: Path) -> Path:
    return target.with_name(target.stem + ".nominal.urdf")


def ensure_nominal_source(source: Path) -> Path:
    """Do not silently fit new corrections on top of a calibrated model."""
    _, root = _tree(source)
    if any(child.tag is ET.Comment and
           (child.text or "").strip().startswith(CALIBRATION_COMMENT_PREFIX)
           for child in root):
        raise ValueError(
            f"拟合基准已包含 mocap 标定结果: {source}；"
            "请用 --urdf 指定替换前的原始 URDF 备份。"
        )
    return source


def resolve_fit_source(target: Path) -> Path:
    """Keep fitting against the original model after installing a candidate."""
    nominal = nominal_backup_path(target)
    if nominal.exists():
        ensure_nominal_source(nominal)
        _, original_root = _tree(nominal)
        _, target_root = _tree(target)
        _check_candidate_matches_target(original_root, target_root)
        return nominal
    return ensure_nominal_source(target)


def _elements(root, tag: str):
    result = {}
    for element in root.findall(tag):
        name = element.get("name")
        if not name or name in result:
            raise ValueError(f"duplicate or missing {tag} name in URDF: {name!r}")
        result[name] = element
    return result


def _canonical(element, *, ignore_origin: bool = False) -> bytes:
    """Stable comparison for two URDF elements from the same generator."""
    copy = ET.fromstring(ET.tostring(element, encoding="unicode"))
    if ignore_origin:
        origin = copy.find("origin")
        if origin is not None:
            copy.remove(origin)
    return ET.tostring(copy, encoding="utf-8")


def _canonical_robot(root) -> bytes:
    copy = ET.fromstring(ET.tostring(root, encoding="unicode"))
    for joint in copy.findall("joint"):
        if joint.get("name") in ARM_JOINTS:
            origin = joint.find("origin")
            if origin is not None:
                joint.remove(origin)
    return ET.tostring(copy, encoding="utf-8")


def _check_candidate_matches_target(target_root, candidate_root):
    if target_root.get("name") != candidate_root.get("name"):
        raise ValueError(
            f"robot name differs: target={target_root.get('name')!r}, "
            f"candidate={candidate_root.get('name')!r}"
        )
    if _canonical_robot(target_root) != _canonical_robot(candidate_root):
        raise ValueError(
            "target/candidate differ outside the fourteen calibrated arm joint origins"
        )
    target_links = _elements(target_root, "link")
    candidate_links = _elements(candidate_root, "link")
    if set(target_links) != set(candidate_links):
        raise ValueError("target/candidate link sets differ; candidate must be full-body URDF")
    for name in target_links:
        if _canonical(target_links[name]) != _canonical(candidate_links[name]):
            raise ValueError(f"link differs outside calibrated joint origins: {name}")

    target_joints = _elements(target_root, "joint")
    candidate_joints = _elements(candidate_root, "joint")
    if set(target_joints) != set(candidate_joints):
        raise ValueError("target/candidate joint sets differ; candidate must be full-body URDF")
    missing_arm = ARM_JOINTS - set(candidate_joints)
    if missing_arm:
        raise ValueError(f"candidate is missing arm joints: {sorted(missing_arm)}")
    for name in target_joints:
        ignore_origin = name in ARM_JOINTS
        if _canonical(target_joints[name], ignore_origin=ignore_origin) != _canonical(
            candidate_joints[name], ignore_origin=ignore_origin
        ):
            scope = "arm joint origin" if ignore_origin else "non-calibrated joint"
            raise ValueError(f"target/candidate differ outside allowed {scope}: {name}")


def install_urdf(target: Path, candidate: Path, *, apply: bool = False,
                 nominal_source: Path = None) -> int:
    target = target.expanduser().resolve()
    candidate = candidate.expanduser().resolve()
    if not target.is_file() or not candidate.is_file():
        raise SystemExit("target/candidate URDF 必须存在")
    if target == candidate:
        raise SystemExit("target 和 candidate 必须是不同文件")

    _, target_root = _tree(target)
    _, candidate_root = _tree(candidate)
    _check_candidate_matches_target(target_root, candidate_root)
    nominal = nominal_backup_path(target)
    if nominal_source is not None:
        nominal_source = ensure_nominal_source(nominal_source.expanduser().resolve())
        _, source_root = _tree(nominal_source)
        _check_candidate_matches_target(source_root, target_root)
        if nominal.exists() and sha256(nominal) != sha256(nominal_source):
            raise ValueError(f"原始拟合基准与已有备份不一致，拒绝覆盖: {nominal}")
    before = sha256(target)
    candidate_hash = sha256(candidate)
    if before == candidate_hash:
        print("target 与 candidate 内容相同，无需重复写入。")
        return 0
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    backup = target.with_name(target.name + ".bak." + stamp)
    print(f"target:    {target}")
    print(f"candidate: {candidate}")
    print(f"target sha256:    {before}")
    print(f"candidate sha256: {candidate_hash}")
    print(f"backup:    {backup}")
    if nominal_source is not None:
        print(f"nominal:   {nominal}")
    if not apply:
        print("dry-run：未写入。确认 target/candidate/备份路径后，再追加 --apply。")
        return 0

    if backup.exists():
        raise SystemExit(f"备份文件已存在，拒绝覆盖: {backup}")
    shutil.copy2(target, backup)
    if sha256(backup) != before:
        raise RuntimeError("备份期间 target 内容发生变化，未替换主 URDF")
    if nominal_source is not None and not nominal.exists():
        # Exclusive creation preserves the first nominal model on later installs.
        with nominal.open("xb") as stream:
            try:
                stream.write(nominal_source.read_bytes())
                stream.flush()
                os.fsync(stream.fileno())
                os.fchmod(stream.fileno(), stat.S_IMODE(nominal_source.stat().st_mode))
            except BaseException:
                nominal.unlink()
                raise
    temporary = None
    try:
        with tempfile.NamedTemporaryFile("wb", dir=str(target.parent),
                                         prefix=".urdf_replace_", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(candidate.read_bytes())
            os.chmod(temporary, stat.S_IMODE(target.stat().st_mode))
            stream.flush()
            os.fsync(stream.fileno())
        if sha256(temporary) != candidate_hash or sha256(target) != before:
            raise RuntimeError("替换前 target/candidate 内容发生变化，未替换主 URDF")
        os.replace(temporary, target)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    after = sha256(target)
    if after != candidate_hash:
        raise RuntimeError("替换后 SHA256 与 candidate 不一致")
    print(f"已写入，target sha256: {after}")
    print("请停止并重新启动运控，使新的 URDF 被运行时重新加载；此脚本不控制机器人。")
    return 0


def install_urdf_directory(target: Path, candidate: Path, *, apply: bool = False,
                           nominal_source: Path = None) -> int:
    """Install the S63 model family, rolling back earlier writes on failure."""
    from urdf_model_sync import build_model_updates

    target = target.expanduser().resolve()
    candidate = candidate.expanduser().resolve()
    if target.name != "biped_s63.urdf" or target == candidate:
        raise ValueError("目录同步要求 target 为 biped_s63.urdf，且 candidate 与其不同")
    source = (resolve_fit_source(target) if nominal_source is None
              else ensure_nominal_source(nominal_source.expanduser().resolve()))
    _, target_root = _tree(target)
    _, source_root = _tree(source)
    _, candidate_root = _tree(candidate)
    _check_candidate_matches_target(source_root, target_root)
    _check_candidate_matches_target(target_root, candidate_root)
    nominal = nominal_backup_path(target)
    source_bytes = source.read_bytes()
    if nominal.exists() and nominal.read_bytes() != source_bytes:
        raise ValueError(f"原始拟合基准与已有备份不一致，拒绝覆盖: {nominal}")

    updates = build_model_updates(target, candidate)
    before = {path: path.read_bytes() for path in updates}
    changed = {path: value for path, value in updates.items() if value != before[path]}
    print(f"同步目录: {target.parent}")
    for path in updates:
        action = "更新" if path in changed else "内容已一致/通过 include 生效"
        print(f"  {action}: {path.relative_to(target.parent)}")
    print(f"原始拟合基准: {nominal}")
    if not apply:
        print("dry-run：目录检查通过，未替换文件。")
        return 0
    if not changed and nominal.exists():
        print("全部模型已与候选一致，无需重复写入。")
        return 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    backup = target.parent.with_name(target.parent.name + ".bak." + stamp)
    shutil.copytree(target.parent, backup, symlinks=True)
    print(f"目录备份: {backup}")
    for path, original in before.items():
        if (backup / path.relative_to(target.parent)).read_bytes() != original:
            raise RuntimeError(f"备份期间文件内容发生变化，未替换: {path}")

    replaced = []
    nominal_created = False
    with tempfile.TemporaryDirectory(prefix=".mocap_install_", dir=str(target.parent.parent)) as temp:
        staging = Path(temp)
        for index, (path, value) in enumerate(changed.items()):
            staged = staging / str(index)
            with staged.open("wb") as stream:
                stream.write(value)
                os.fchmod(stream.fileno(), stat.S_IMODE(path.stat().st_mode))
                stream.flush()
                os.fsync(stream.fileno())
        try:
            if any(path.read_bytes() != original for path, original in before.items()):
                raise RuntimeError("确认后模型文件内容发生变化，未替换任何模型")
            if not nominal.exists():
                with nominal.open("xb") as stream:
                    nominal_created = True
                    stream.write(source_bytes)
                    stream.flush()
                    os.fsync(stream.fileno())
                    os.fchmod(stream.fileno(), stat.S_IMODE(source.stat().st_mode))
            elif nominal.read_bytes() != source_bytes:
                raise RuntimeError("原始拟合基准内容发生变化，未替换任何模型")
            for index, path in enumerate(changed):
                os.replace(staging / str(index), path)
                replaced.append(path)
            if any(path.read_bytes() != value for path, value in updates.items()):
                raise RuntimeError("替换后的模型内容校验失败")
        except BaseException as error:
            failures = []
            for index, path in enumerate(reversed(replaced)):
                try:
                    restored = staging / f"restore_{index}"
                    shutil.copy2(backup / path.relative_to(target.parent), restored)
                    os.replace(restored, path)
                except OSError as exc:
                    failures.append(f"{path}: {exc}")
            if nominal_created:
                try:
                    nominal.unlink()
                except OSError as exc:
                    failures.append(f"{nominal}: {exc}")
            if failures:
                raise RuntimeError(f"目录更新失败且部分回滚失败，请从 {backup} 恢复: "
                                   + "; ".join(failures)) from error
            raise RuntimeError(f"目录更新失败，已回滚本次修改；备份保留在 {backup}: {error}") from error
    print(f"已同步 {len(changed)} 个模型文件；Gazebo xacro 使用同步后的 include。")
    print("后续拟合自动使用原始 nominal 备份；相关 IK/MPC 进程需重新加载模型后生效。")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-urdf", type=Path, required=True,
                        help="正在被运控加载的完整运行时 URDF")
    parser.add_argument("--candidate-urdf", type=Path, required=True,
                        help="mocap_optimize_1010.py 生成的完整 urdf_final.urdf")
    parser.add_argument("--apply", action="store_true",
                        help="实际写入；默认只检查并打印计划")
    parser.add_argument("--all-models", action="store_true",
                        help="同步 S63 urdf 目录内全部模型，包括 Drake、Gazebo 和重建的左右单臂")
    args = parser.parse_args()
    if args.all_models:
        return install_urdf_directory(args.target_urdf, args.candidate_urdf, apply=args.apply)
    return install_urdf(args.target_urdf, args.candidate_urdf, apply=args.apply)


if __name__ == "__main__":
    raise SystemExit(main())
