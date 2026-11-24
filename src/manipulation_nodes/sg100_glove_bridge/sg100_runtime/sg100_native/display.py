"""Meshcat display models and per-hand visibility."""

import subprocess
import sys
from contextlib import redirect_stdout
from pathlib import Path

from .sdk import JOINT_ORDER


def display_models(hand, display_dir=None):
    import pinocchio as pin

    if hand not in ("right", "left"):
        raise ValueError("hand must be 'right' or 'left'")
    directory = (
        Path(display_dir)
        if display_dir is not None
        else (Path(__file__).resolve().parents[1] / "display" / hand)
    )
    return pin.buildModelsFromUrdf(
        str(directory / f"{hand}_sg100.urdf"), package_dirs=[str(directory)]
    )


class HandVisualizer:
    def __init__(self, hands, display_dir=None):
        self.models = {}
        self.viewer = None
        self._closed = False
        try:
            with redirect_stdout(sys.stderr):
                self._load(hands, display_dir)
        except BaseException:
            self.close()
            raise

    def _load(self, hands, display_dir):
        import numpy as np
        import pinocchio as pin
        from pinocchio.visualize import MeshcatVisualizer

        for hand in hands:
            directory = Path(display_dir) / hand if display_dir and len(hands) > 1 else display_dir
            model, collision, visual = display_models(hand, directory)
            viz = MeshcatVisualizer(model, collision, visual)
            viz.initViewer(viewer=self.viewer, open=False)
            if self.viewer is None:
                self.viewer = viz.viewer
            viz.loadViewerModel(rootNodeName=hand)
            q = pin.neutral(model)
            indices = [model.joints[model.getJointId(name)].idx_q for name in JOINT_ORDER]
            viz.display(q)
            if len(hands) > 1:
                transform = np.eye(4)
                transform[0, 3] = -0.13 if hand == "left" else 0.13
                self.viewer[hand].set_transform(transform)
            self.models[hand] = (viz, q, indices)
            self.hide(hand)
        distance = 0.28 if len(hands) > 1 else 0.20
        self.viewer["/Cameras/default/rotated/<object>"].set_property(
            "position", [distance, distance * 0.7, distance]
        )
        viz.setBackgroundColor(col_top=[0.97647, 0.98039, 0.98039])
        self.viewer["/Grid"].set_property("visible", False)
        self.viewer["/Axes"].set_property("visible", False)

    def preview(self, hand=None):
        """Show a neutral model without marking its data stream as online."""
        import pinocchio as pin

        for side in self.models if hand is None else (hand,):
            viz, q, _ = self.models[side]
            q[:] = pin.neutral(viz.model)
            viz.display(q)
            self.viewer[side].set_property("visible", True)

    def update(self, hand, angles):
        viz, q, indices = self.models[hand]
        q[indices] = angles
        viz.display(q)
        self.viewer[hand].set_property("visible", True)

    def hide(self, hand):
        self.viewer[hand].set_property("visible", False)

    def close(self):
        if self.viewer is None or self._closed:
            return
        self._closed = True
        window = self.viewer.window
        window.zmq_socket.close(linger=0)
        process = window.server_proc
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
