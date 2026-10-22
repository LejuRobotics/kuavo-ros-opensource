#!/usr/bin/env python3
"""Live, read-only URDF projection on the head camera; no catkin build needed.

Reuses Camera_Calibration's FK primitives, joint layout and CameraInfo reader.
Only subscribes to feedback/camera/static TF; never sends motion commands.
Real STL triangle faces form a continuous arm silhouette (without scene occlusion).
Run --help, or see README_cam_proj_validation.md for camera prerequisites.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import struct
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
CALIB_ROOT = HERE.parent
sys.path.insert(0, str(CALIB_ROOT))

# Projection-only correction: physical head yaw zero reads -3.7 degrees.
HEAD_YAW_OFFSET_DEG = 0.0
HEAD_PITCH_OFFSET_DEG = 0.0

# These imports only define functions; no recorder or zero-writing main is run.
from mocap_joint_calib.write_zero import build_joint_locations
from plot_board_error_from_csv import (
    find_chain_joint_names, load_urdf_joints, parse_axis, parse_origin, rodrigues,
)


class JointLayout:
    def __init__(self, version, kuavo_json=None):
        self.path = Path(kuavo_json) if kuavo_json else (
            CALIB_ROOT.parent / "kuavo_assets" / "config" / f"kuavo_v{version}" / "kuavo.json")
        cfg = json.loads(self.path.read_text())
        locations, _, _, _, _ = build_joint_locations(str(version), self.path)
        self.indices = {name: loc.motor_index for name, loc in locations.items()}
        self.count = int(cfg["NUM_JOINT"])
        waist_count = int(cfg.get("NUM_WAIST_JOINT", 0))
        if waist_count == 1:
            self.indices["waist_yaw_joint"] = (
                self.count - int(cfg["NUM_ARM_JOINT"]) - int(cfg["NUM_HEAD_JOINT"]) - 1)
        elif waist_count:
            raise ValueError("Only the single yaw waist layout is supported")

    def decode(self, values):
        q = np.asarray(values, dtype=float)
        if q.shape != (self.count,) or not np.isfinite(q).all():
            raise ValueError(f"joint_q must contain {self.count} finite radians; got {q.shape}")
        return {name: float(q[index]) for name, index in self.indices.items()}


def rigid_inverse(matrix):
    out = np.eye(4)
    out[:3, :3] = matrix[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ matrix[:3, 3]
    return out


def pose_matrix(values):
    """T_parent_child from x y z (metres), qx qy qz qw."""
    from scipy.spatial.transform import Rotation
    values = np.asarray(values, dtype=float)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError("Transform needs seven finite x y z qx qy qz qw values")
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    out[:3, 3] = values[:3]
    return out


class StaticTransforms:
    """Only /tf_static edges: never borrow old-model moving joints from /tf."""
    def __init__(self):
        self.edges = {}

    def update(self, parent, child, matrix):
        self.edges[child.lstrip("/")] = (parent.lstrip("/"), matrix)

    def lookup(self, parent, child):
        parent, child = parent.lstrip("/"), child.lstrip("/")
        todo = [(parent, np.eye(4))]
        visited = set()
        while todo:
            frame, transform = todo.pop()
            if frame == child:
                return transform
            if frame in visited:
                continue
            visited.add(frame)
            for edge_child, (edge_parent, matrix) in self.edges.items():
                if edge_parent == frame and edge_child not in visited:
                    todo.append((edge_child, transform @ matrix))
                if edge_child == frame and edge_parent not in visited:
                    todo.append((edge_parent, transform @ rigid_inverse(matrix)))
        raise ValueError(f"No fixed /tf_static path: {parent} -> {child}")


class CameraExtrinsics:
    """Join candidate-URDF fixed geometry to the existing camera-driver TF.

    Kuavo5's deployed camera bridge and the wheel-arm launch documentation
    identify head_camera_depth with the driver root. Apply that nominal
    convention locally for S56/S62/S63, without publishing TF or borrowing
    moving joints (or fixed geometry) from an old robot model.
    """
    def __init__(self, model, version, mount_frame="head_camera_depth", driver_frame="head_camera_link"):
        self.anchor = model.camera_link
        self.mount_frame, self.driver_frame = mount_frame, driver_frame
        self.version = str(version)
        self.fixed = StaticTransforms()
        for joint in model.joints.values():
            if joint.get("type") != "fixed":
                continue
            rotation, translation = parse_origin(joint.find("origin"))
            matrix = np.eye(4)
            matrix[:3, :3], matrix[:3, 3] = rotation, translation
            self.fixed.update(joint.find("parent").get("link"), joint.find("child").get("link"), matrix)

    def resolve(self, driver_tf, image_frame):
        # A candidate with its own optical chain is already self-contained.
        try:
            return self.fixed.lookup(self.anchor, image_frame), "candidate URDF optical chain"
        except ValueError:
            pass
        try:
            anchor_mount = self.fixed.lookup(self.anchor, self.mount_frame)
            driver_optical = driver_tf.lookup(self.driver_frame, image_frame)
        except ValueError:
            # Explicit complete fixed TF remains supported for other cameras.
            return driver_tf.lookup(self.anchor, image_frame), "complete /tf_static optical chain"
        try:
            mount_driver = driver_tf.lookup(self.mount_frame, self.driver_frame)
            source = "published mount TF"
        except ValueError:
            if self.version not in ("56", "62", "63") or self.mount_frame != "head_camera_depth":
                raise ValueError("No published camera mount TF; provide --optical-transform for this model")
            mount_driver = np.eye(4)
            source = f"S{self.version} nominal head_camera_depth = driver root (Kuavo camera bridge convention)"
        return anchor_mount @ mount_driver @ driver_optical, source


class JointHistory:
    """Bounded feedback history; select by image stamp, never by command angle."""
    def __init__(self, size=2000):
        self.samples = deque(maxlen=size)
        self.arrivals = deque(maxlen=size)

    def append(self, stamp, values, received=None):
        received = time.monotonic() if received is None else received
        if self.samples and stamp < self.samples[-1][0]:
            self.samples.clear()  # bag replay/time reset; do not mix epochs
            self.arrivals.clear()
        self.samples.append((stamp, values))
        self.arrivals.append((stamp, received, values))

    def nearest(self, stamp, max_skew):
        if not self.samples:
            raise ValueError("Waiting for joint feedback")
        sample = min(self.samples, key=lambda item: abs(item[0] - stamp))
        skew = sample[0] - stamp
        if abs(skew) > max_skew:
            raise ValueError(f"Image/joint stamp mismatch: {skew * 1000:+.1f} ms")
        return sample[1], sample[0], skew

    def match(self, image_stamp, image_received, max_skew, mode="header",
              stale_after=1.0, now=None):
        """Receipt matching is an explicit approximation for stationary poses.

        Both arrival clocks come from this process, before rendering or locks.
        They do not establish when an image was captured across the network.
        """
        if mode not in ("header", "receipt"):
            raise ValueError(f"Unknown sync mode: {mode}")
        if image_received is None:
            raise ValueError("Image reception time is required for freshness checks")
        now = time.monotonic() if now is None else now
        image_age = now - image_received
        if image_age > stale_after:
            raise ValueError("Waiting for fresh camera images")
        if not self.arrivals:
            raise ValueError("Waiting for joint feedback")
        if now - self.arrivals[-1][1] > stale_after:
            raise ValueError("Waiting for fresh joint feedback")
        index, target = (0, image_stamp) if mode == "header" else (1, image_received)
        joint_stamp, joint_received, values = min(
            self.arrivals, key=lambda item: abs(item[index] - target))
        header_skew, receipt_skew = joint_stamp - image_stamp, joint_received - image_received
        skew = header_skew if mode == "header" else receipt_skew
        if abs(skew) > max_skew:
            domain = "stamp" if mode == "header" else "receipt"
            raise ValueError(f"Image/joint {domain} mismatch: {skew * 1000:+.1f} ms")
        joint_age = now - joint_received
        if joint_age > stale_after:
            raise ValueError("Matched joint feedback is stale")
        timing = dict(sync_mode=mode, matching_skew_ms=skew * 1000,
                      header_skew_ms=header_skew * 1000, receipt_skew_ms=receipt_skew * 1000,
                      image_arrival_age_ms=image_age * 1000, joint_arrival_age_ms=joint_age * 1000,
                      header_stamps_within_tolerance=abs(header_skew) <= max_skew,
                      capture_time_alignment="header" if mode == "header" else "unverified_receipt")
        return values, joint_stamp, skew, timing


def resolve_mesh(uri, urdf_path):
    if uri.startswith("package://"):
        package, relative = uri[len("package://"):].split("/", 1)
        # Prefer this checkout, not an unrelated overlay from ROS_PACKAGE_PATH.
        candidate = CALIB_ROOT.parent / package / relative
        if candidate.is_file():
            return candidate
        import rospkg
        candidate = Path(rospkg.RosPack().get_path(package)) / relative
    elif uri.startswith("file://"):
        candidate = Path(uri[len("file://"):])
    else:
        candidate = urdf_path.parent / uri
    if not candidate.is_file():
        raise FileNotFoundError(f"Mesh not found: {uri} (resolved {candidate})")
    return candidate


def load_stl_mesh(path):
    """Read all STL triangles once, retaining their faces and shared vertices."""
    data = Path(path).read_bytes()
    count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * count:
        dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
                          ("attribute", "<u2")])
        points = np.frombuffer(data, dtype=dtype, offset=84, count=count)["vertices"].reshape(-1, 3)
    else:
        # Also accepts ASCII STL; other mesh formats fail explicitly.
        points = np.asarray([list(map(float, line.split()[1:]))
                             for line in data.decode("ascii").splitlines()
                             if line.strip().startswith("vertex ")], dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or len(points) % 3 or not np.isfinite(points).all():
        raise ValueError(f"Invalid/empty STL: {path}")
    vertices, indices = np.unique(points, axis=0, return_inverse=True)
    return vertices.astype(np.float64), indices.reshape(-1, 3).astype(np.int32)


def load_stl_points(path, limit):
    """Compatibility helper; rendering uses all faces rather than this sample."""
    points, _ = load_stl_mesh(path)
    if len(points) > limit:
        points = points[np.linspace(0, len(points) - 1, limit, dtype=int)]
    return points


def strip_s56_hand_mesh(path, vertices, faces):
    """Keep wrist CAD components from the verified S56 misnamed nohand assets.

    These two files contain a complete palm/fingers below the wrist flange.
    Preserve whole original connected components, rather than cutting or
    inventing wrist geometry. Other versions/assets are not inferred by name.
    """
    path = Path(path)
    hashes = {
        "l_hand_pitch_nohand.STL": "d2218b13cc6f70482de86a1d6b9c7c364baaa95cd1f8e117ec3747a0854e2ebc",
        "r_hand_pitch_nohand.STL": "28ee2cb04299525e62ee7f27f191027dbb0294e6f3d29df37a1fbfabc2ff7f5e",
    }
    if path.name not in hashes or path.parent.name != "meshes" or path.parent.parent.name != "biped_s56":
        return vertices, faces, None
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != hashes[path.name]:
        raise ValueError(f"Unverified S56 wrist mesh: {path}; inspect its hand geometry before projection")
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    graph = coo_matrix((np.ones(len(edges), dtype=np.uint8), (edges[:, 0], edges[:, 1])),
                       shape=(len(vertices), len(vertices)))
    count, labels = connected_components(graph, directed=False)
    component_min_z = np.full(count, np.inf)
    np.minimum.at(component_min_z, labels, vertices[:, 2])
    # The verified CAD wrist ends at ~-69.38 mm; the palm starts below it.
    keep = component_min_z[labels[faces[:, 0]]] >= -0.0696
    used, remap = np.unique(faces[keep], return_inverse=True)
    if not len(used) or keep.all():
        raise ValueError(f"S56 wrist/hand component separation failed: {path}")
    info = dict(method="verified_s56_wrist_components", source_sha256=digest,
                component_min_z_threshold_m=-0.0696, source_triangles=len(faces),
                kept_triangles=int(keep.sum()), removed_triangles=int((~keep).sum()))
    return vertices[used], remap.reshape(-1, 3).astype(np.int32), info


def clip_triangles_near(triangles, near=1e-6):
    """Clip crossing faces instead of dropping them or projecting negative Z."""
    triangles = np.asarray(triangles, dtype=np.float64).reshape(-1, 3, 3)
    front = triangles[:, :, 2] >= near
    whole = triangles[front.all(axis=1)]
    pieces = []
    for triangle in triangles[front.any(axis=1) & ~front.all(axis=1)]:
        polygon = []
        for previous, current in zip(np.roll(triangle, 1, axis=0), triangle):
            prev_in, curr_in = previous[2] >= near, current[2] >= near
            if prev_in != curr_in:
                ratio = (near - previous[2]) / (current[2] - previous[2])
                intersection = previous + ratio * (current - previous)
                intersection[2] = near
                polygon.append(intersection)
            if curr_in:
                polygon.append(current)
        for index in range(1, len(polygon) - 1):
            pieces.append([polygon[0], polygon[index], polygon[index + 1]])
    if pieces:
        return np.concatenate((whole, np.asarray(pieces)))
    return whole


def triangle_union_mask(triangles, width, height):
    """Rasterize the union at pixel centres with batched triangle scanlines.

    Positive coverage is a union, unlike fillPoly's even/odd rule, which erases
    overlapping faces. Concavities and actual holes remain intact.
    """
    triangles = np.asarray(triangles, dtype=np.float64).reshape(-1, 3, 2)
    mask = np.zeros((height, width), np.uint8)
    triangles = triangles[np.isfinite(triangles).all(axis=(1, 2))]
    if not len(triangles):
        return mask
    lo = np.minimum(np.minimum(triangles[:, 0], triangles[:, 1]), triangles[:, 2])
    hi = np.maximum(np.maximum(triangles[:, 0], triangles[:, 1]), triangles[:, 2])
    triangles = triangles[(hi[:, 0] >= 0) & (lo[:, 0] <= width - 1)
                          & (hi[:, 1] >= 0) & (lo[:, 1] <= height - 1)]
    if not len(triangles):
        return mask
    order = np.argsort(triangles[:, :, 1], axis=1)
    ordered = np.take_along_axis(triangles, order[:, :, None], axis=1)
    starts = np.ceil(np.clip(ordered[:, 0, 1], 0, height - 1)).astype(np.int32)
    stops = np.floor(np.clip(ordered[:, 2, 1], 0, height - 1)).astype(np.int32)
    counts = np.maximum(0, stops - starts + 1)
    cumulative = np.cumsum(counts)
    first = 0
    while first < len(ordered):
        # Bound temporary arrays even if a camera intersects dense geometry.
        offset = int(cumulative[first - 1]) if first else 0
        last = max(first + 1, int(np.searchsorted(cumulative, offset + 1_000_000, side="right")))
        batch, lengths = ordered[first:last], counts[first:last]
        total = int(lengths.sum())
        if total:
            indices = np.repeat(np.arange(len(batch)), lengths)
            row = (np.arange(total) - np.repeat(np.cumsum(lengths) - lengths, lengths)
                   + np.repeat(starts[first:last], lengths))
            a, b, c = batch[:, 0], batch[:, 1], batch[:, 2]
            def slope(p, q):
                dy = q[:, 1] - p[:, 1]
                return np.divide(q[:, 0] - p[:, 0], dy, out=np.zeros(len(dy)), where=dy != 0)
            long_x = a[indices, 0] + (row - a[indices, 1]) * slope(a, c)[indices]
            top = row < b[indices, 1]
            short_x = np.where(top,
                               a[indices, 0] + (row - a[indices, 1]) * slope(a, b)[indices],
                               b[indices, 0] + (row - b[indices, 1]) * slope(b, c)[indices])
            lower, upper = np.minimum(long_x, short_x), np.maximum(long_x, short_x)
            horizontal = (c[:, 1] == a[:, 1])[indices]
            if horizontal.any():
                flat_x = batch[indices[horizontal], :, 0]
                lower[horizontal] = flat_x.min(axis=1)
                upper[horizontal] = flat_x.max(axis=1)
            left = np.ceil(np.clip(lower, 0, width)).astype(np.int32)
            right = np.floor(np.clip(upper, -1, width - 1)).astype(np.int32)
            valid = left <= right
            row, left, right = row[valid], left[valid], right[valid]
            size = height * (width + 1)
            delta = (np.bincount(row * (width + 1) + left, minlength=size)
                     - np.bincount(row * (width + 1) + right + 1, minlength=size))
            covered = np.cumsum(delta.reshape(height, width + 1), axis=1)[:, :width] > 0
            mask[covered] = 255
        first = last
    return mask


def undistort_points_compat(pixels, K, D):
    """Use the native iterative inverse in both OpenCV 4 and OpenCV 5."""
    import cv2
    criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 1e-9)
    undistort_iter = getattr(cv2, "undistortPointsIter", None)
    if callable(undistort_iter):
        rays = undistort_iter(pixels, K, D, None, None, criteria)
    else:
        # OpenCV 5 merged the iterative overload into undistortPoints.
        rays = cv2.undistortPoints(pixels, K, D, criteria=criteria)
    return rays.reshape(-1, 2)


class RasterCamera:
    """Cache inverse distortion so distant vertices cannot fold into view.

    Triangles fill a pinhole image, remapped to raw pixels. Distorted curved
    triangle edges need no straight-chord approximation or surface subdivision.
    """
    def __init__(self, K, D, model, width, height):
        import cv2
        self.K, self.D, self.model = np.asarray(K, float), np.asarray(D, float), model
        self.width, self.height = width, height
        self.key = (width, height, model, self.K.tobytes(), self.D.tobytes())
        self.offset = np.zeros(2)
        self.canvas_width, self.canvas_height = width, height
        self.map_x = self.map_y = None
        if model != "equidistant" and not np.any(self.D):
            return
        yy, xx = np.indices((height, width), dtype=np.float32)
        pixels = np.stack((xx, yy), axis=-1).reshape(-1, 1, 2).astype(np.float64)
        if model == "equidistant":
            rays = cv2.fisheye.undistortPoints(pixels, self.K, self.D).reshape(-1, 2)
        else:
            rays = undistort_points_compat(pixels, self.K, self.D)
        ideal = rays * [self.K[0, 0], self.K[1, 1]] + [self.K[0, 2], self.K[1, 2]]
        back = self.distort(np.column_stack((rays, np.ones(len(rays)))))
        valid = np.isfinite(ideal).all(axis=1) & (np.linalg.norm(back - pixels[:, 0], axis=1) < 0.05)
        if not valid.any():
            raise ValueError("Camera distortion has no valid inverse in this image")
        low = np.floor(ideal[valid].min(axis=0)) - 1
        high = np.ceil(ideal[valid].max(axis=0)) + 1
        dimensions = high - low + 1
        if np.prod(dimensions) > 16 * width * height:
            raise ValueError("Camera distortion requires an excessive field; verify CameraInfo")
        self.offset = low
        self.canvas_width, self.canvas_height = map(int, dimensions)
        ideal -= low
        ideal[~valid] = -1
        self.map_x = ideal[:, 0].reshape(height, width).astype(np.float32)
        self.map_y = ideal[:, 1].reshape(height, width).astype(np.float32)

    def distort(self, xyz):
        import cv2
        xyz = np.asarray(xyz, np.float64).reshape(-1, 1, 3)
        if self.model == "equidistant":
            uv = cv2.fisheye.projectPoints(xyz, np.zeros(3), np.zeros(3), self.K, self.D)[0]
        else:
            uv = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), self.K, self.D)[0]
        return uv.reshape(-1, 2)

    def ideal_pixels(self, xyz):
        return (xyz[..., :2] / xyz[..., 2:3] * [self.K[0, 0], self.K[1, 1]]
                + [self.K[0, 2], self.K[1, 2]] - self.offset)

    def raw_mask(self, mask):
        import cv2
        if self.map_x is None:
            return mask
        mapped = cv2.remap(mask, self.map_x, self.map_y, cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        return (mapped >= 128).astype(np.uint8) * 255


class ProjectionModel:
    def __init__(self, urdf_path, camera_link="head_camera_base", arms="both", points_per_link=2500,
                 root_link="base_link"):
        self.path = Path(urdf_path).expanduser().resolve()
        self.sha256 = hashlib.sha256(self.path.read_bytes()).hexdigest()
        self.camera_link = camera_link
        self.root_link = root_link
        self.joints = load_urdf_joints(self.path)
        xml = ET.parse(self.path).getroot()
        links = {link.get("name"): link for link in xml.findall("link")}
        if camera_link not in links:
            raise ValueError(f"Camera anchor {camera_link!r} is absent from {self.path}")
        if root_link not in links:
            raise ValueError(f"Projection root {root_link!r} is absent from {self.path}")
        sides = ("l", "r") if arms == "both" else (("l",) if arms == "left" else ("r",))
        # This validator ends at J7: do not follow attached hands/tools or the
        # nominal end-effector frame 17 cm beyond the wrist.
        selected = {f"zarm_{side}{i}_link" for side in sides for i in range(1, 8)}
        self.tips = [f"zarm_{side}7_link" for side in sides]
        self.clouds = []
        self.meshes = []
        self.raster_camera = None
        self.mesh_paths = []
        self.mesh_filters = []
        for name in sorted(selected):
            if name not in links:
                raise ValueError(f"Arm link {name!r} missing from URDF")
            for visual in links[name].findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    raise ValueError(f"{name}: this lightweight renderer supports STL visuals only")
                path = resolve_mesh(mesh.get("filename", ""), self.path)
                if path.suffix.lower() != ".stl":
                    raise ValueError(f"Only STL visuals supported: {path}")
                vertices, faces = load_stl_mesh(path)
                if name in self.tips:
                    vertices, faces, mesh_filter = strip_s56_hand_mesh(path, vertices, faces)
                    if mesh_filter is not None:
                        self.mesh_filters.append(dict(link=name, **mesh_filter))
                scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                if scale.shape != (3,) or not np.isfinite(scale).all():
                    raise ValueError(f"Invalid mesh scale on {name}")
                rotation, translation = parse_origin(visual.find("origin"))
                vertices = (vertices * scale) @ rotation.T + translation
                self.meshes.append((name, vertices, faces))
                points = vertices
                if len(points) > points_per_link:
                    points = points[np.linspace(0, len(points) - 1, points_per_link, dtype=int)]
                self.clouds.append((name, points))
                self.mesh_paths.append(str(path))
        if not self.clouds:
            raise ValueError("No arm visual meshes found")
        targets = [camera_link] + [name for name, _ in self.clouds] + self.tips
        order = []
        for target in targets:
            for name in find_chain_joint_names(self.joints, self.root_link, target):
                if name not in order:
                    order.append(name)
        self.steps = []
        self.required_joints = set()
        for name in order:
            j = self.joints[name]
            kind = j.get("type", "fixed")
            if kind not in ("fixed", "revolute", "continuous") or j.find("mimic") is not None:
                raise ValueError(f"Unsupported joint on projection chain: {name} ({kind})")
            rotation, translation = parse_origin(j.find("origin"))
            origin = np.eye(4)
            origin[:3, :3], origin[:3, 3] = rotation, translation
            self.steps.append((name, j.find("parent").get("link"), j.find("child").get("link"),
                               kind, origin, parse_axis(j)))
            if kind != "fixed":
                self.required_joints.add(name)

    def fk(self, q):
        missing = self.required_joints - q.keys()
        if missing:
            raise ValueError(f"Missing measured joints: {sorted(missing)}")
        poses = {self.root_link: np.eye(4)}
        for name, parent, child, kind, origin, axis in self.steps:
            transform = origin
            if kind != "fixed":
                motion = np.eye(4)
                motion[:3, :3] = rodrigues(axis, q[name])
                transform = origin @ motion
            poses[child] = poses[parent] @ transform
        return poses

    def render(self, frame, q, optical_transform, K, D, distortion_model="plumb_bob",
               alpha=0.55, pixel_rotation=0, mask_color_rgb=(255, 190, 0)):
        import cv2
        if pixel_rotation not in (0, 180):
            raise ValueError("pixel_rotation must be 0 or 180 degrees")
        poses = self.fk(q)
        camera_pose = poses[self.camera_link] @ optical_transform
        to_camera = rigid_inverse(camera_pose)
        height, width = frame.shape[:2]
        key = (width, height, distortion_model, np.asarray(K, float).tobytes(),
               np.asarray(D, float).tobytes())
        if self.raster_camera is None or self.raster_camera.key != key:
            self.raster_camera = RasterCamera(K, D, distortion_model, width, height)
        camera = self.raster_camera
        triangles, projected = [], 0
        for name, vertices, faces in self.meshes:
            transform = to_camera @ poses[name]
            xyz = vertices @ transform[:3, :3].T + transform[:3, 3]
            front = xyz[:, 2] >= 1e-6
            projected += int(np.count_nonzero(front))
            good = front[faces]
            if good.all():
                triangles.append(camera.ideal_pixels(xyz)[faces])
            elif good.any():
                clipped = clip_triangles_near(xyz[faces])
                if len(clipped):
                    triangles.append(camera.ideal_pixels(clipped))
        faces_2d = np.concatenate(triangles) if triangles else np.empty((0, 3, 2))
        mask = triangle_union_mask(faces_2d, camera.canvas_width, camera.canvas_height)
        mask = camera.raw_mask(mask)
        if pixel_rotation == 180:
            mask = cv2.rotate(mask, cv2.ROTATE_180)
        # All arm faces share one union mask: no internal triangle/link contours.
        # This is a silhouette, without depth testing against torso or real objects.
        tint = frame.copy()
        tint[mask != 0] = tuple(reversed(mask_color_rgb))  # RGB option -> OpenCV BGR
        output = cv2.addWeighted(frame, 1.0 - alpha, tint, alpha, 0.0)
        boundary = mask != cv2.erode(mask, np.ones((3, 3), np.uint8))
        output[boundary] = (0, 255, 255)
        for name in self.tips:
            point = (to_camera @ poses[name])[:3, 3].reshape(1, 3)
            if point[0, 2] <= 1e-6:
                continue
            ideal = camera.ideal_pixels(point)[0]
            if not (0 <= ideal[0] < camera.canvas_width and 0 <= ideal[1] < camera.canvas_height):
                continue
            pixels = project_points(point, K, D, distortion_model, width, height)
            if len(pixels):
                uv = tuple(int(v) for v in pixels[0])
                if pixel_rotation == 180:
                    uv = (width - 1 - uv[0], height - 1 - uv[1])
                cv2.drawMarker(output, uv, (255, 100, 0), cv2.MARKER_CROSS, 14, 2)
                cv2.putText(output, "L link7" if "_l7_" in name else "R link7", (uv[0] + 8, uv[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 100, 0), 1)
        stats = {"projected_points": projected, "projected_triangles": len(faces_2d),
                 "mask_pixels": int(cv2.countNonZero(mask)),
                 "render_mode": "triangle_union", "pixel_rotation": pixel_rotation,
                 "mask_color_rgb": list(mask_color_rgb), "alpha": float(alpha),
                 "fk_root": self.root_link, "T_root_optical": camera_pose.tolist()}
        if self.root_link == "base_link":
            stats["T_base_optical"] = camera_pose.tolist()
        return output, stats


def project_points(xyz, K, D, model, width, height):
    import cv2
    xyz = np.asarray(xyz, dtype=np.float64)
    # Reject points behind or arbitrarily close to the optical centre before division.
    xyz = xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 1e-6)]
    if not len(xyz):
        return np.empty((0, 2), dtype=np.int32)
    if model == "equidistant":
        uv = cv2.fisheye.projectPoints(xyz.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)[0]
    else:
        uv = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), K, D)[0]
    uv = uv.reshape(-1, 2)
    keep = (np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 0) & (uv[:, 0] < width - 0.5)
            & (uv[:, 1] >= 0) & (uv[:, 1] < height - 0.5))
    return np.rint(uv[keep]).astype(np.int32)


def camera_parameters(info, frame, image_frame):
    from chessboard_pose_logger.record_chessboard_pose_to_csv import _intrinsics_from_camera_info
    if info.header.frame_id.lstrip("/") != image_frame.lstrip("/"):
        raise ValueError("Image and CameraInfo frame_id differ; select matching topics")
    if (info.height, info.width) != frame.shape[:2]:
        raise ValueError("Image and CameraInfo sizes differ; do not resize one without its intrinsics")
    if (info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset
            or info.roi.width not in (0, info.width) or info.roi.height not in (0, info.height)):
        raise ValueError("Binned/cropped CameraInfo is not supported; use the full-resolution raw stream")
    model = info.distortion_model
    if model not in ("", "plumb_bob", "rational_polynomial", "equidistant"):
        raise ValueError(f"Unsupported distortion model: {model}")
    intrinsics = _intrinsics_from_camera_info(info)
    K, D = intrinsics.K, intrinsics.D
    if not np.isfinite(K).all() or not np.isfinite(D).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError("CameraInfo is uncalibrated or invalid")
    if model == "equidistant" and len(D) != 4:
        raise ValueError("equidistant CameraInfo requires four distortion coefficients")
    if model != "equidistant" and len(D) not in (0, 4, 5, 8, 12, 14):
        raise ValueError("Unsupported OpenCV distortion coefficient count")
    return K, D, model


class ProjectionViewer:
    def __init__(self, args):
        import cv2
        import rospy
        from cv_bridge import CvBridge
        from kuavo_msgs.msg import sensorsData
        from sensor_msgs.msg import CameraInfo, CompressedImage, Image
        from tf2_msgs.msg import TFMessage
        self.args, self.rospy, self.cv2 = args, rospy, cv2
        self.bridge = CvBridge()
        # Arm and camera poses share this wheel-arm waist frame. The chassis
        # chain cancels from their relative transform, so it needs no angles.
        root_link = "waist_yaw_link" if args.robot_version in ("62", "63") else "base_link"
        self.model = ProjectionModel(args.urdf, args.camera_link, args.arms, root_link=root_link)
        self.layout = JointLayout(args.robot_version, args.kuavo_json)
        self.extrinsics = CameraExtrinsics(self.model, args.robot_version, args.mount_frame, args.driver_frame)
        missing = self.model.required_joints - self.layout.indices.keys()
        if missing:
            raise ValueError(f"kuavo.json cannot map required joints: {sorted(missing)}")
        self.history, self.static = JointHistory(), StaticTransforms()
        self.lock = threading.Lock()
        self.latest_image, self.info = None, None
        self.optical_override = pose_matrix(args.optical_transform) if args.optical_transform else None
        self.pub = rospy.Publisher(args.output_topic, Image, queue_size=1)
        self.subs = [
            rospy.Subscriber(args.image_topic, CompressedImage if args.compressed else Image,
                             self.on_image, queue_size=1, buff_size=2**24, tcp_nodelay=True),
            rospy.Subscriber(args.camera_info_topic, CameraInfo, self.on_info, queue_size=1),
            rospy.Subscriber(args.joint_topic, sensorsData, self.on_joints, queue_size=1, tcp_nodelay=True),
            rospy.Subscriber("/tf_static", TFMessage, self.on_static, queue_size=10),
        ]
        cv2.setNumThreads(1)
        self.last_save = None
        rospy.loginfo("URDF: %s sha256=%s", self.model.path, self.model.sha256)
        rospy.loginfo("Projection FK root: %s", self.model.root_link)
        rospy.loginfo("Joint layout: %s (%d values); indices=%s", self.layout.path, self.layout.count, self.layout.indices)
        rospy.loginfo("Cached %d STL triangles; max display rate %.1f Hz. s=save, q/Esc=exit",
                      sum(len(faces) for _, _, faces in self.model.meshes), args.fps)
        rospy.loginfo("Uses existing camera topics; does not start camera drivers or command joints")
        if args.sync_mode == "receipt":
            rospy.logwarn("RECEIPT matching: use stationary poses only; source clocks/capture times remain unverified")

    def on_image(self, msg):
        received = time.monotonic()
        with self.lock:
            self.latest_image = (msg, received)

    def on_info(self, msg):
        with self.lock:
            self.info = msg

    def on_joints(self, msg):
        received = time.monotonic()
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0:
            self.rospy.logwarn_throttle(3, "Joint feedback has zero header stamp; no overlay")
            return
        # No FK or image work in the 1 kHz feedback callback.
        values = tuple(msg.joint_data.joint_q)
        with self.lock:
            self.history.append(stamp, values, received)

    def on_static(self, msg):
        for item in msg.transforms:
            t, q = item.transform.translation, item.transform.rotation
            try:
                matrix = pose_matrix([t.x, t.y, t.z, q.x, q.y, q.z, q.w])
                with self.lock:
                    self.static.update(item.header.frame_id, item.child_frame_id, matrix)
            except ValueError as exc:
                self.rospy.logwarn_throttle(3, str(exc))

    def image_rotation(self):
        """Optional pixel correction, never inferred from the driver's setting.

        On the tested Gemini335L (FW 1.4.60, SDK 1.10.22), ROS rgb8, K/D and TF
        stay in the same orientation for driver color_rotation=0 and 180.
        Thus the verified default is zero; blindly copying 180 is incorrect.
        """
        return int(self.args.image_rotation)

    def process(self, msg, received=None):
        started = time.perf_counter()
        frame = (self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") if self.args.compressed
                 else self.bridge.imgmsg_to_cv2(msg, "bgr8"))
        stamp, image_frame = msg.header.stamp.to_sec(), msg.header.frame_id
        status = ""
        try:
            if stamp <= 0 or not image_frame:
                raise ValueError("Image needs a nonzero stamp and optical frame_id")
            with self.lock:
                info = self.info
                if info is None:
                    raise ValueError("Waiting for CameraInfo")
                values, joint_stamp, skew, timing = self.history.match(
                    stamp, received, self.args.max_skew, self.args.sync_mode, self.args.stale_after)
                if self.optical_override is not None:
                    optical, extrinsic_source = self.optical_override.copy(), "explicit --optical-transform"
                else:
                    optical, extrinsic_source = self.extrinsics.resolve(self.static, image_frame)
            K, D, distortion_model = camera_parameters(info, frame, image_frame)
            q = self.layout.decode(values)
            q_feedback = q.copy()
            print(
                f"head Z(yaw)={np.degrees(q['zhead_1_joint']):+.3f} deg, "
                f"Y(pitch)={np.degrees(q['zhead_2_joint']):+.3f} deg",
                flush=True,
            )
            
            
            q["zhead_1_joint"] += np.deg2rad(HEAD_YAW_OFFSET_DEG)
            q["zhead_2_joint"] += np.deg2rad(HEAD_PITCH_OFFSET_DEG)
            
            pixel_rotation = self.image_rotation()
            output, stats = self.model.render(frame, q, optical, K, D, distortion_model, self.args.alpha,
                                              pixel_rotation=pixel_rotation, mask_color_rgb=self.args.mask_color)
            elapsed_ms = (time.perf_counter() - started) * 1000
            status = f"URDF FK | dt(q-image) {skew * 1000:+.1f} ms | render {elapsed_ms:.1f} ms | points {stats['projected_points']}"
            if self.args.sync_mode == "receipt":
                status = (f"RECEIPT / static poses | arrival {skew * 1000:+.1f} ms | "
                          f"header {timing['header_skew_ms'] / 1000:+.2f} s | "
                          f"render {elapsed_ms:.0f} ms | mask {stats['mask_pixels']} px")
            metadata = dict(stats, urdf=str(self.model.path), urdf_sha256=self.model.sha256,
                            kuavo_json=str(self.layout.path), joint_indices=self.layout.indices,
                            image_stamp=stamp, joint_stamp=joint_stamp, joint_q_rad=q,
                            joint_q_feedback_rad=q_feedback, head_yaw_offset_deg=HEAD_YAW_OFFSET_DEG,
                            image_topic=self.args.image_topic, joint_topic=self.args.joint_topic,
                            camera_info_topic=self.args.camera_info_topic, image_frame=image_frame,
                            camera_link=self.model.camera_link, T_camera_link_optical=optical.tolist(),
                            extrinsic_source=extrinsic_source, mount_frame=self.args.mount_frame,
                            driver_frame=self.args.driver_frame,
                            K=K.tolist(), D=D.tolist(), distortion_model=distortion_model,
                            pixel_rotation_deg=pixel_rotation,
                            rotation_source="--image-rotation (verified default: 0)",
                            width=frame.shape[1], height=frame.shape[0], mesh_paths=self.model.mesh_paths,
                            mesh_filters=self.model.mesh_filters, endpoint_frames=self.model.tips,
                            geometry_scope="arm links 1-7; verified S56 wrist components only",
                            approximate_mesh_points=False,
                            mesh_triangles=sum(len(faces) for _, _, faces in self.model.meshes))
            metadata.update(timing)
            # Keep just this frame for an optional snapshot, without extra copies.
            self.last_save = (frame, output, metadata)
        except (ValueError, RuntimeError) as exc:
            self.last_save = None
            output = frame.copy()
            status = "NO OVERLAY: " + str(exc)
            self.rospy.logwarn_throttle(3, status)
        self.cv2.putText(output, status, (8, 24), self.cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                         (0, 255, 255) if self.last_save else (0, 0, 255), 1, self.cv2.LINE_AA)
        self.publish_image(output, msg.header)
        return output

    def publish_image(self, output, header=None):
        if self.pub.get_num_connections():
            out_msg = self.bridge.cv2_to_imgmsg(output, "bgr8")
            if header is not None:
                out_msg.header = header
            self.pub.publish(out_msg)

    def save(self):
        if self.last_save is None:
            self.rospy.logwarn("No valid projection to save")
            return
        raw, overlay, metadata = self.last_save
        directory = self.args.save_dir / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        directory.mkdir(parents=True, exist_ok=False)
        for name, image in [("raw.png", raw), ("overlay.png", overlay)]:
            if not self.cv2.imwrite(str(directory / name), image):
                raise IOError(f"Failed to save {directory / name}")
        (directory / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        self.rospy.loginfo("Saved %s", directory)

    def check_inputs(self):
        """Read-only preflight, including partial results with no robot feedback."""
        deadline = time.monotonic() + self.args.check_timeout
        report = {}
        while not self.rospy.is_shutdown():
            time_check = "time_sync" if self.args.sync_mode == "header" else "receipt_match"
            checks = {key: "WAIT" for key in ("image", "camera_info", "camera_transform",
                                               "joint_feedback", time_check, "image_rotation")}
            report = {"urdf": str(self.model.path), "urdf_sha256": self.model.sha256,
                      "fk_root": self.model.root_link,
                      "joint_count": self.layout.count, "image_topic": self.args.image_topic,
                      "joint_topic": self.args.joint_topic, "sync_mode": self.args.sync_mode,
                      "checks": checks}
            if self.args.sync_mode == "receipt":
                report["capture_time_alignment"] = "unverified_receipt"
                report["usage"] = "Stationary poses only; arrival matching does not synchronize source clocks"
            with self.lock:
                latest, info = self.latest_image, self.info
                last_joint = self.history.samples[-1] if self.history.samples else None
            stamp, frame, image_frame = None, None, ""
            try:
                if latest is None or time.monotonic() - latest[1] > self.args.stale_after:
                    raise ValueError("Waiting for fresh camera images")
                msg = latest[0]
                image_frame = msg.header.frame_id
                candidate_stamp = msg.header.stamp.to_sec()
                if candidate_stamp <= 0 or not image_frame:
                    raise ValueError("Image needs a nonzero stamp and optical frame_id")
                frame = (self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") if self.args.compressed
                         else self.bridge.imgmsg_to_cv2(msg, "bgr8"))
                stamp = candidate_stamp
                checks["image"] = "OK"
                report["image_size"] = [int(frame.shape[1]), int(frame.shape[0])]
            except Exception as exc:
                checks["image"] = str(exc)
            if info is not None:
                checks["camera_info"] = "Received; waiting for image geometry"
                if frame is not None:
                    try:
                        camera_parameters(info, frame, image_frame)
                        checks["camera_info"] = "OK"
                    except ValueError as exc:
                        checks["camera_info"] = str(exc)
            target_frame = image_frame or (info.header.frame_id if info is not None else "")
            if target_frame:
                try:
                    with self.lock:
                        optical, source = ((self.optical_override.copy(), "explicit --optical-transform")
                                           if self.optical_override is not None else
                                           self.extrinsics.resolve(self.static, target_frame))
                    checks["camera_transform"] = "OK"
                    report["extrinsic_source"] = source
                    report["T_camera_link_optical"] = optical.tolist()
                except ValueError as exc:
                    checks["camera_transform"] = str(exc)
            if last_joint is not None:
                try:
                    q = self.layout.decode(last_joint[1])
                    checks["joint_feedback"] = "OK"
                    report["head_deg"] = [float(np.degrees(q[name])) for name in
                                          ("zhead_1_joint", "zhead_2_joint")]
                except ValueError as exc:
                    checks["joint_feedback"] = str(exc)
            if stamp is not None:
                try:
                    with self.lock:
                        values, joint_stamp, skew, timing = self.history.match(
                            stamp, latest[1], self.args.max_skew, self.args.sync_mode, self.args.stale_after)
                    self.layout.decode(values)
                    checks[time_check] = "OK"
                    report["joint_image_skew_ms"] = timing["header_skew_ms"]
                    report.update(timing)
                except ValueError as exc:
                    checks[time_check] = str(exc)
            try:
                report["pixel_rotation_deg"] = self.image_rotation()
                checks["image_rotation"] = "OK"
            except ValueError as exc:
                checks["image_rotation"] = str(exc)
            report["ready"] = all(value == "OK" for value in checks.values())
            if report["ready"] or time.monotonic() >= deadline:
                break
            time.sleep(0.05)
        print(json.dumps(report, indent=2, ensure_ascii=False))
        for sub in self.subs:
            sub.unregister()
        return bool(report.get("ready"))

    def run(self):
        window = "Kuavo camera projection (s: save, q: exit)"
        if not self.args.no_display:
            self.cv2.namedWindow(window, self.cv2.WINDOW_NORMAL)
        previous, next_frame = None, 0.0
        shown = None
        was_stale = False
        while not self.rospy.is_shutdown():
            now = time.monotonic()
            redraw = False
            with self.lock:
                latest = self.latest_image
            if latest is not None and latest[0] is not previous and now >= next_frame:
                msg, received = latest
                previous = msg
                try:
                    shown = self.process(msg, received)
                except Exception as exc:
                    self.last_save = None
                    shown = np.zeros((100, 720, 3), np.uint8)
                    self.cv2.putText(shown, "Cannot decode/process image; see terminal", (8, 40),
                                     self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    self.publish_image(shown, msg.header)
                    self.rospy.logwarn_throttle(3, f"Cannot process camera image: {exc}")
                redraw = True
                next_frame = now + 1.0 / self.args.fps
            stale = latest is None or now - latest[1] > self.args.stale_after
            if stale:
                self.last_save = None
                if not was_stale:
                    shown = np.zeros((100, 720, 3), np.uint8)
                    self.cv2.putText(shown, "Waiting for fresh camera images: " + self.args.image_topic,
                                     (8, 40), self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    self.publish_image(shown, latest[0].header if latest else None)
                    redraw = True
                self.rospy.logwarn_throttle(3, "Waiting for fresh camera images")
            was_stale = stale
            if not self.args.no_display:
                if shown is not None and redraw:
                    display = shown
                    if shown.shape[1] > self.args.display_width:
                        height = max(1, round(shown.shape[0] * self.args.display_width / shown.shape[1]))
                        display = self.cv2.resize(shown, (self.args.display_width, height), interpolation=self.cv2.INTER_AREA)
                    self.cv2.imshow(window, display)
                key = self.cv2.waitKey(1) & 0xff
                if key in (27, ord("q")):
                    break
                try:
                    visible = self.cv2.getWindowProperty(window, self.cv2.WND_PROP_VISIBLE)
                    # OpenCV 4.2/GTK on 79 returns -1 for unsupported VISIBLE.
                    # AUTOSIZE still reports 0/1 while that window exists.
                    closed = visible == 0 or (visible < 0 and self.cv2.getWindowProperty(
                        window, self.cv2.WND_PROP_AUTOSIZE) < 0)
                    if closed:
                        break
                except self.cv2.error:
                    break
                if key == ord("s"):
                    self.save()
            time.sleep(0.005)
        for sub in self.subs:
            sub.unregister()
        if not self.args.no_display:
            self.cv2.destroyAllWindows()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--urdf", type=Path,
                        help="Candidate URDF; default: biped_s63 for versions 62/63, output/urdf_final.urdf otherwise")
    parser.add_argument("--robot-version", default=os.environ.get("ROBOT_VERSION", "56"))
    parser.add_argument("--kuavo-json", type=Path, help="Override joint layout config; otherwise use this checkout")
    parser.add_argument("--image-topic", default="/head_camera/color/image_raw", help="Raw, distorted colour stream")
    parser.add_argument("--camera-info-topic", default="/head_camera/color/camera_info")
    parser.add_argument("--joint-topic", default="/sensors_data_raw", help="Actual kuavo_msgs/sensorsData feedback")
    parser.add_argument("--compressed", action="store_true", help="image-topic is sensor_msgs/CompressedImage")
    parser.add_argument("--image-rotation", choices=("0", "180"), default="0",
                        help="Additional projected-pixel rotation; verified Gemini335L default is 0, even with driver color_rotation=180")
    parser.add_argument("--camera-link", default="head_camera_base", help="Camera anchor link in the candidate URDF")
    parser.add_argument("--optical-transform", type=float, nargs=7, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
                        help="Explicit T_camera_link_image_optical, metres/xyzw; default: candidate mount + driver TF")
    parser.add_argument("--mount-frame", default="head_camera_depth", help="Candidate URDF camera mount anchor")
    parser.add_argument("--driver-frame", default="head_camera_link", help="Root frame of the existing camera driver")
    parser.add_argument("--arms", choices=("left", "right", "both"), default="both")
    parser.add_argument("--fps", type=float, default=20.0, help="Maximum processing rate; older images are discarded")
    parser.add_argument("--alpha", type=float, default=0.55)
    parser.add_argument("--mask-color", type=int, nargs=3, metavar=("R", "G", "B"),
                        default=(255, 190, 0), help="Translucent mask RGB colour; each channel 0-255")
    parser.add_argument("--sync-mode", choices=("header", "receipt"), default="header",
                        help="header: match source stamps; receipt: approximate by local arrival times for stationary poses only")
    parser.add_argument("--max-skew", type=float, default=0.1, help="Allowed image/feedback difference in the selected sync mode, seconds")
    parser.add_argument("--stale-after", type=float, default=1.0, help="Mark stopped image stream after this many seconds")
    parser.add_argument("--output-topic", default="/cam_proj_validation/image_overlay")
    parser.add_argument("--no-display", action="store_true", help="Only publish overlay when a subscriber is connected")
    parser.add_argument("--display-width", type=int, default=960, help="Max GUI image width to reduce SSH/X11 traffic; saved/published image stays full size")
    parser.add_argument("--check", action="store_true", help="Check camera, fixed transform and feedback; no GUI; exit 0 when ready, 2 otherwise")
    parser.add_argument("--check-timeout", type=float, default=8.0, help="Seconds to wait for inputs with --check")
    parser.add_argument("--save-dir", type=Path, default=HERE / "output/cam_proj_validation")
    args = parser.parse_args(argv)
    if args.urdf is None:
        args.urdf = (CALIB_ROOT.parent / "kuavo_assets/models/biped_s63/urdf/biped_s63.urdf"
                     if args.robot_version in ("62", "63") else HERE / "output/urdf_final.urdf")
    if any(channel < 0 or channel > 255 for channel in args.mask_color):
        parser.error("mask-color RGB channels must be integers in [0,255]")
    if not np.isfinite([args.fps, args.alpha, args.max_skew, args.stale_after]).all():
        parser.error("Rendering limits must be finite")
    if args.display_width < 1:
        parser.error("display-width must be positive")
    if not np.isfinite(args.check_timeout) or args.check_timeout <= 0:
        parser.error("check-timeout must be finite and positive")
    if args.fps <= 0 or not 0 <= args.alpha <= 1 or args.max_skew <= 0 or args.stale_after <= 0:
        parser.error("fps/max-skew/stale-after must be positive, alpha in [0,1]")
    return args


def main():
    # Let --help work before ROS/OpenCV have been installed or sourced.
    argv = [arg for arg in sys.argv[1:] if not arg.startswith("__")]
    args = parse_args(argv)
    try:
        import cv2
        import rospy
        from cv_bridge import CvBridge
        from kuavo_msgs.msg import sensorsData
    except ImportError as exc:
        raise SystemExit(f"Missing ROS/OpenCV dependency: {exc}\n"
                         "Use the existing camera-calibration Python environment; source devel/setup.bash first.")
    if not args.no_display and not args.check and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise SystemExit("No desktop display. Run on the desktop, or use --no-display and view the output ROS topic.")
    rospy.init_node("cam_proj_validation", anonymous=True)
    try:
        viewer = ProjectionViewer(args)
        if args.check:
            raise SystemExit(0 if viewer.check_inputs() else 2)
        viewer.run()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass


if __name__ == "__main__":
    main()
