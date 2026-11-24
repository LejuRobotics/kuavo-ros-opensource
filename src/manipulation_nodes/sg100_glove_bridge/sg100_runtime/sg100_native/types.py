"""Finger geometry and numerical solver settings."""

from dataclasses import dataclass, field


@dataclass
class FingerSpec:
    name: str
    joints: list[str]
    tip_frame: str
    tip_offset_xyz: tuple[float, float, float]
    pos_weight: float = 1.0
    rot_weight: float = 0.04


@dataclass
class SolverConfig:
    max_iters: int = 40
    step_eta: float = 1.0
    damping_lambda0: float = 0.001
    pos_tol: float = 1e-4
    max_step: float = 0.2
    bounds_eps: float = 1e-4
    posture_weight: float = 0.001
    smooth_weight: float = 0.01
    finger_weights: dict = field(default_factory=dict)
    mcp_aa_direct: dict = field(default_factory=dict)
    side_orientation_ik: dict = field(default_factory=dict)
    four_finger_flexion_orientation_ik: dict = field(default_factory=dict)
    root_flexion_weight: dict = field(default_factory=dict)
    tip_reference_world_wxyz: object = None
    tip_reference_world_wxyz_per_finger: dict = field(default_factory=dict)
