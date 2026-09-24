"""Canonical SG100 gripper postures.

Both grippers use one normalized joint-space coordinate:
``closure=0`` is fully open and ``closure=1`` is fully closed.  Intermediate
values stay on the same straight line in joint space.
"""

JOINT_SUFFIXES = (
    "thumb_j1", "thumb_j2", "thumb_j3",
    "index_j1", "index_j2", "index_j3",
    "middle_j1", "middle_j2",
    "little_j1", "little_j2", "little_j3",
)


def _validate(side, closure):
    if side not in ("l", "r"):
        raise ValueError("side must be 'l' or 'r'")
    closure = float(closure)
    if not 0.0 <= closure <= 1.0:
        raise ValueError("closure must be in [0, 1]")
    return closure


def joint_names(side):
    """Return the controller/model joint order for one hand."""
    _validate(side, 0.0)
    return tuple("{}_{}".format(side, suffix) for suffix in JOINT_SUFFIXES)


def single_gripper_pose(closure, side):
    """Return the canonical thumb/index gripper as a named radian pose."""
    closure = _validate(side, closure)
    bare = {
        "thumb_j1": 1.5,
        "thumb_j2": -2.5 + 1.5 * closure,
        "thumb_j3": 1.5 * (1.0 - closure),
        "index_j1": 0.0,
        "index_j2": 1.5 * closure,
        "index_j3": 1.5 * (1.0 - closure),
        "middle_j1": 0.0,
        "middle_j2": 0.0,
        "little_j1": 0.0,
        "little_j2": 0.0,
        "little_j3": 0.0,
    }
    return {
        "{}_{}".format(side, suffix): bare[suffix]
        for suffix in JOINT_SUFFIXES
    }


def double_gripper_pose(closure, side):
    """Return the two-pair gripper: single gripper plus middle/little."""
    closure = _validate(side, closure)
    pose = single_gripper_pose(closure, side)
    additions = {
        "middle_j1": 1.5 * closure,
        "middle_j2": 1.5 * (1.0 - closure),
        # little_j1 forms the second opposing finger and then stays frozen.
        "little_j1": 3.0,
        "little_j2": 1.5 * closure,
        "little_j3": 1.5 * (1.0 - closure),
    }
    pose.update({
        "{}_{}".format(side, suffix): value
        for suffix, value in additions.items()
    })
    return pose


def box_gripper_pose(closure, side):
    """Return the dedicated whole-box gripper pose.

    The shaped hand keeps all flexion joints fixed around the box. Closure
    couples only the index abduction joint and thumb opposition joint:
    index_j1 moves -1.0 -> 0.0 while thumb_j1 moves 1.0 -> 1.8.
    """
    closure = _validate(side, closure)
    bare = {
        "thumb_j1": 1.0 + 0.8 * closure,
        "thumb_j2": -1.0,
        "thumb_j3": 0.0,
        "index_j1": -1.0 + closure,
        "index_j2": 1.0,
        "index_j3": 0.0,
        "middle_j1": 1.0,
        "middle_j2": 0.0,
        "little_j1": 3.0,
        "little_j2": 1.0,
        "little_j3": 0.0,
    }
    return {
        "{}_{}".format(side, suffix): bare[suffix]
        for suffix in JOINT_SUFFIXES
    }


def internal_independent_expansion_pose(
        index_expansion, middle_expansion, little_expansion, side,
        little_j3_compact=0.0, little_j3_expanded=0.0):
    """Return a Task 3 pose with one expansion coordinate per finger.

    Each normalized coordinate is independent so a latched finger can keep
    its last target while the other fingers continue searching.  The default
    four-joint mode keeps ``little_j3`` at zero.  A scanner may explicitly
    supply a compact/expanded range for that already-authorized fifth joint.
    """
    index_expansion = _validate(side, index_expansion)
    middle_expansion = _validate(side, middle_expansion)
    little_expansion = _validate(side, little_expansion)
    little_j3_compact = float(little_j3_compact)
    little_j3_expanded = float(little_j3_expanded)
    bare = {
        "thumb_j1": 0.0,
        "thumb_j2": 0.0,
        "thumb_j3": 0.0,
        "index_j1": 0.0,
        "index_j2": 1.55 - 0.55 * index_expansion,
        "index_j3": 0.0,
        "middle_j1": 1.65 - 0.55 * middle_expansion,
        "middle_j2": 0.0,
        "little_j1": 3.0,
        "little_j2": 1.8 - 0.8 * little_expansion,
        "little_j3": (
            little_j3_compact
            + (little_j3_expanded - little_j3_compact) * little_expansion),
    }
    return {
        "{}_{}".format(side, suffix): bare[suffix]
        for suffix in JOINT_SUFFIXES
    }


def internal_expansion_pose(expansion, side):
    """Return the compatibility Task 3 pose on a shared expansion path.

    New Task 3 code should use :func:`internal_independent_expansion_pose`.
    Unlike the superseded trajectory, the default path does not move the
    optional fifth joint ``little_j3``.
    """
    expansion = _validate(side, expansion)
    return internal_independent_expansion_pose(
        expansion, expansion, expansion, side)


def advance_internal_expansions(expansions, latch_mask, step, limits=None):
    """Advance only SEARCHING fingers and return an immutable coordinate tuple.

    Mask bits 0/1/2 represent index/middle/little.  A set bit freezes that
    finger at its input coordinate.  ``limits`` are per-finger normalized
    search limits and default to the theoretical endpoint 1.0.
    """
    if len(expansions) != 3:
        raise ValueError("expansions must contain index, middle, and little")
    if limits is None:
        limits = (1.0, 1.0, 1.0)
    if len(limits) != 3:
        raise ValueError("limits must contain index, middle, and little")
    step = float(step)
    if step <= 0.0:
        raise ValueError("step must be positive")
    latch_mask = int(latch_mask)
    if latch_mask < 0 or latch_mask > 0b111:
        raise ValueError("latch_mask must use only the lowest three bits")
    result = []
    for index, (value, limit) in enumerate(zip(expansions, limits)):
        value = float(value)
        limit = float(limit)
        if not 0.0 <= value <= 1.0 or not 0.0 <= limit <= 1.0:
            raise ValueError("expansion coordinates and limits must be in [0, 1]")
        if value > limit:
            raise ValueError("an expansion coordinate exceeds its search limit")
        result.append(value if latch_mask & (1 << index)
                      else min(limit, value + step))
    return tuple(result)


def lever_hook_pose(side):
    """Return the fixed inverted-L hand used to hook a horizontal lever.

    The thumb points straight forward with all three joints at zero.  Index,
    middle, and little keep their proximal joints straight while only their
    distal joints bend down to form one three-pad hook edge.
    """
    _validate(side, 0.0)
    bent_distal_joints = {
        "index_j3": 1.5,
        "middle_j2": 1.5,
        "little_j3": 1.5,
    }
    return {
        "{}_{}".format(side, suffix): bent_distal_joints.get(suffix, 0.0)
        for suffix in JOINT_SUFFIXES
    }


def _ordered_targets(pose, side):
    return [pose[name] for name in joint_names(side)]


def single_gripper_targets(side, closure):
    """Return the canonical single-gripper pose in controller joint order."""
    return _ordered_targets(single_gripper_pose(closure, side), side)


def double_gripper_targets(side, closure):
    """Return the canonical double-gripper pose in controller joint order."""
    return _ordered_targets(double_gripper_pose(closure, side), side)


def box_gripper_targets(side, closure):
    """Return the whole-box gripper pose in controller joint order."""
    return _ordered_targets(box_gripper_pose(closure, side), side)


def internal_expansion_targets(side, expansion):
    """Return the Task 3 internal-expansion pose in controller order."""
    return _ordered_targets(internal_expansion_pose(expansion, side), side)


def internal_independent_expansion_targets(
        side, index_expansion, middle_expansion, little_expansion,
        little_j3_compact=0.0, little_j3_expanded=0.0):
    """Return independent Task 3 finger targets in controller order."""
    return _ordered_targets(
        internal_independent_expansion_pose(
            index_expansion, middle_expansion, little_expansion, side,
            little_j3_compact=little_j3_compact,
            little_j3_expanded=little_j3_expanded),
        side)


def lever_hook_targets(side):
    """Return the fixed lever-hook pose in controller joint order."""
    return _ordered_targets(lever_hook_pose(side), side)
