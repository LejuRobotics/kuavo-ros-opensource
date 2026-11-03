"""Build a consistent S63 model set without writing to the asset directory.

The candidate contains final, absolute arm joint origins.  Copy those origins;
never apply its Ei corrections to another model's (possibly older) geometry.
Drake upper-body/full models call the local torso frame ``base_link``.  That
frame is explicitly treated as the main model's ``waist_yaw_link`` here.  This
does not change or equate the full model's upstream leg/waist kinematics.
"""

import copy
import json
import math
from pathlib import Path
import re
from typing import Dict
import xml.etree.ElementTree as ET
from xml.parsers import expat


_MAIN = Path("biped_s63.urdf")
_VARIANTS = {
    Path("biped_s63_gazebo.urdf"): "waist_yaw_link",
    Path("drake/biped_v3_arm.urdf"): "waist_yaw_link",
    Path("drake/biped_v3_upper_body.urdf"): "base_link",
    Path("drake/biped_v3_full.urdf"): "base_link",
}
_SINGLE_ARMS = {Path("drake/Larm.urdf"): "l", Path("drake/Rarm.urdf"): "r"}
_XACRO = Path("biped_s63_gazebo.xacro")
_KNOWN_MODELS = {_MAIN, _XACRO, *_VARIANTS, *_SINGLE_ARMS}
_JOINT_NAMES = tuple("zarm_%s%d_joint" % (side, i) for side in "lr" for i in range(1, 8))
_AXES = ((0, 1, 0), (1, 0, 0), (0, 0, 1), (0, 1, 0),
         (0, 0, 1), (1, 0, 0), (0, 1, 0))
_METADATA = re.compile(rb"<!--\s*mocap_kin_calib:.*?-->", re.DOTALL)


def _parse(data, label):
    try:
        root = ET.fromstring(data, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    except ET.ParseError as exc:
        raise ValueError("%s: invalid XML: %s" % (label, exc)) from exc
    if root.tag != "robot":
        raise ValueError("%s: expected a robot root" % label)
    return root


def _named(root, tag, label):
    result = {}
    for element in root.findall(tag):
        name = element.get("name")
        if not name or name in result:
            raise ValueError("%s: missing or duplicate %s name %r" % (label, tag, name))
        result[name] = element
    return result


def _validate_tree(root, label):
    """Require a complete, connected URDF tree (not a dangling arm fragment)."""
    links = _named(root, "link", label)
    joints = _named(root, "joint", label)
    if not links:
        raise ValueError("%s: no links" % label)
    children = {}
    edges = {name: [] for name in links}
    for name, joint in joints.items():
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            raise ValueError("%s: %s lacks a parent or child" % (label, name))
        parent_name, child_name = parent.get("link"), child.get("link")
        if parent_name not in links or child_name not in links:
            raise ValueError("%s: %s references a missing link" % (label, name))
        if child_name in children:
            raise ValueError("%s: link %s has multiple parent joints" % (label, child_name))
        children[child_name] = name
        edges[parent_name].append(child_name)
    roots = set(links) - set(children)
    if len(roots) != 1:
        raise ValueError("%s: expected one root link, found %s" % (label, sorted(roots)))
    visited, pending = set(), list(roots)
    while pending:
        current = pending.pop()
        if current in visited:
            raise ValueError("%s: cyclic link tree at %s" % (label, current))
        visited.add(current)
        pending.extend(edges[current])
    if visited != set(links):
        raise ValueError("%s: disconnected or cyclic links %s" % (label, sorted(set(links) - visited)))
    return links, joints


def _vector(element, attribute, label):
    try:
        values = tuple(float(value) for value in element.attrib[attribute].split())
    except (AttributeError, KeyError, ValueError) as exc:
        raise ValueError("%s: missing or invalid %s" % (label, attribute)) from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("%s: %s must contain three finite numbers" % (label, attribute))
    return values


def _check_arms(joints, torso, label):
    for side in "lr":
        for i, axis in enumerate(_AXES, 1):
            name = "zarm_%s%d_joint" % (side, i)
            joint = joints.get(name)
            if joint is None:
                raise ValueError("%s: missing arm joint %s" % (label, name))
            expected_parent = torso if i == 1 else "zarm_%s%d_link" % (side, i - 1)
            expected_child = "zarm_%s%d_link" % (side, i)
            if (joint.get("type") != "revolute"
                    or joint.find("parent").get("link") != expected_parent
                    or joint.find("child").get("link") != expected_child):
                raise ValueError("%s: incompatible type or parent/child for %s" % (label, name))
            actual_axis = _vector(joint.find("axis"), "xyz", "%s: %s axis" % (label, name))
            if any(abs(actual - expected) > 1e-12 for actual, expected in zip(actual_axis, axis)):
                raise ValueError("%s: incompatible axis for %s: %s" % (label, name, actual_axis))
            origins = joint.findall("origin")
            if len(origins) != 1 or len(origins[0]) or (origins[0].text or "").strip():
                raise ValueError("%s: %s must have exactly one empty origin" % (label, name))
            _vector(origins[0], "xyz", "%s: %s origin" % (label, name))
            _vector(origins[0], "rpy", "%s: %s origin" % (label, name))


def _metadata_comment(data, label):
    matches = list(_METADATA.finditer(data))
    if len(matches) != 1:
        raise ValueError("%s: expected one mocap_kin_calib metadata comment" % label)
    comment = matches[0].group()
    payload = comment.split(b"mocap_kin_calib:", 1)[1][:-3].strip()
    try:
        metadata = json.loads(payload)
    except (ValueError, UnicodeDecodeError) as exc:
        raise ValueError("%s: invalid mocap calibration metadata" % label) from exc
    if not isinstance(metadata, dict) or metadata.get("schema_version") != 1:
        raise ValueError("%s: unsupported mocap calibration metadata" % label)
    return comment


def _tag_end(data, start):
    quote = None
    for i in range(start, len(data)):
        char = data[i:i + 1]
        if quote:
            if char == quote:
                quote = None
        elif char in (b'"', b"'"):
            quote = char
        elif char == b">":
            return i + 1
    raise ValueError("unterminated XML tag")


def _origin_spans(data):
    """Locate top-level joint origins without reserializing unrelated XML."""
    parser = expat.ParserCreate()
    depth, current_joint, open_origin = 0, None, None
    spans = {}

    def start(name, attributes):
        nonlocal depth, current_joint, open_origin
        depth += 1
        if depth == 2 and name == "joint":
            current_joint = attributes.get("name")
        elif depth == 3 and name == "origin" and current_joint in _JOINT_NAMES:
            begin = parser.CurrentByteIndex
            end = _tag_end(data, begin)
            if data[begin:end].rstrip().endswith(b"/>"):
                spans[current_joint] = (begin, end)
            else:
                open_origin = begin

    def end(name):
        nonlocal depth, current_joint, open_origin
        if depth == 3 and name == "origin" and open_origin is not None:
            spans[current_joint] = (open_origin, _tag_end(data, parser.CurrentByteIndex))
            open_origin = None
        if depth == 2 and name == "joint":
            current_joint = None
        depth -= 1

    parser.StartElementHandler, parser.EndElementHandler = start, end
    parser.Parse(data, True)
    if set(spans) != set(_JOINT_NAMES):
        raise ValueError("cannot locate all 14 top-level arm joint origins")
    return spans


def _copy_origins(data, candidate_joints, comment):
    spans = _origin_spans(data)
    # Reverse byte offsets keep all other XML, including meshes and inertias, intact.
    replacements = [(start, end, ET.tostring(candidate_joints[name].find("origin"),
                                             encoding="utf-8").strip())
                    for name, (start, end) in spans.items()]
    for start, end, origin in sorted(replacements, reverse=True):
        data = data[:start] + origin + data[end:]
    existing = list(_METADATA.finditer(data))
    if len(existing) > 1:
        raise ValueError("destination contains duplicate mocap calibration metadata")
    if existing:
        old = existing[0]
        return data[:old.start()] + comment + data[old.end():]
    robot_start = re.search(rb"<robot\b", data)
    if robot_start is None:
        raise ValueError("missing robot root")
    end = _tag_end(data, robot_start.start())
    return data[:end] + b"\n  " + comment + data[end:]


def _single_arm(candidate_links, candidate_joints, side, comment):
    root = ET.Element("robot", {"name": "S63_%sarm" % side})
    root.append(ET.Comment(comment[4:-3].decode("utf-8")))
    ET.SubElement(root, "link", {"name": "waist_yaw_link"})
    for i in range(1, 8):
        root.append(copy.deepcopy(candidate_links["zarm_%s%d_link" % (side, i)]))
        root.append(copy.deepcopy(candidate_joints["zarm_%s%d_joint" % (side, i)]))
    _validate_tree(root, "%s single arm" % side)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"


def build_model_updates(target: Path, candidate: Path, *, rebuild_legacy_arms: bool = True) -> Dict[Path, bytes]:
    """Return validated bytes for every existing S63 runtime URDF/Xacro.

    ``target`` is the S63 ``urdf`` directory or its ``biped_s63.urdf`` file.
    The stable ``biped_s63.nominal.urdf`` is a fitting input and never updated.
    Legacy Larm/Rarm are rebuilt as complete standard S63 seven-joint models;
    explicitly disabling that option returns those two original fragments as-is.
    No directory or file is created, changed, or deleted by this function.
    """
    target = Path(target)
    directory = (target if target.is_dir() else target.parent).resolve()
    if not target.is_dir() and target.name != _MAIN.name:
        raise ValueError("target must be the S63 urdf directory or biped_s63.urdf")
    discovered = {path.relative_to(directory) for path in directory.rglob("*")
                  if path.is_file() and path.suffix.lower() in (".urdf", ".xacro")}
    discovered.discard(Path("biped_s63.nominal.urdf"))
    unexpected, missing = discovered - _KNOWN_MODELS, _KNOWN_MODELS - discovered
    if unexpected or missing:
        raise ValueError("S63 model set mismatch; unexpected=%s; missing=%s" %
                         (sorted(map(str, unexpected)), sorted(map(str, missing))))
    for relative in _KNOWN_MODELS:
        path = directory / relative
        if path.is_symlink() or path.resolve().parent not in (directory, directory / "drake"):
            raise ValueError("model must be a regular file inside the S63 urdf directory: %s" % path)

    candidate = Path(candidate)
    source_data = candidate.read_bytes()
    source_root = _parse(source_data, candidate)
    source_links, source_joints = _validate_tree(source_root, candidate)
    _check_arms(source_joints, "waist_yaw_link", candidate)
    comment = _metadata_comment(source_data, candidate)
    updates = {directory / _MAIN: source_data}
    for relative, torso in _VARIANTS.items():
        path = directory / relative
        data = path.read_bytes()
        root = _parse(data, path)
        _, joints = _validate_tree(root, path)
        _check_arms(joints, torso, path)
        updates[path] = _copy_origins(data, source_joints, comment)
        updated_root = _parse(updates[path], path)
        _, updated_joints = _validate_tree(updated_root, path)
        _check_arms(updated_joints, torso, path)
    for relative, side in _SINGLE_ARMS.items():
        path = directory / relative
        updates[path] = (_single_arm(source_links, source_joints, side, comment)
                         if rebuild_legacy_arms else path.read_bytes())

    xacro = directory / _XACRO
    data = xacro.read_bytes()
    root = _parse(data, xacro)
    includes = [node.get("filename", "") for node in root.findall("{http://ros.org/wiki/xacro}include")]
    if (not any(name.endswith("/biped_s63_gazebo.urdf") for name in includes)
            or any(node.get("name") in _JOINT_NAMES for node in root.iter("joint"))):
        raise ValueError("%s: expected a wrapper including the updated Gazebo URDF, without arm joints" % xacro)
    updates[xacro] = data
    return updates
