#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

try:
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Float64
except Exception as e:  # pragma: no cover
    rospy = None
    CvBridge = None
    CameraInfo = None
    Image = None
    Float64 = None
    _ros_import_error = e


def _quat_from_R(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert 3x3 rotation matrix to quaternion (x,y,z,w)."""
    # Robust branchless-ish conversion
    t = float(np.trace(R))
    if t > 0.0:
        s = (t + 1.0) ** 0.5 * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = (1.0 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5 * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = (1.0 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5 * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = (1.0 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5 * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    # Normalize
    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if n < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / n, qy / n, qz / n, qw / n


def _board_object_points(nx: int, ny: int, square: float, origin_at_center: bool) -> np.ndarray:
    objp = np.zeros((nx * ny, 3), np.float32)
    objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2) * float(square)
    if origin_at_center:
        objp[:, 0] -= (nx - 1) * float(square) * 0.5
        objp[:, 1] -= (ny - 1) * float(square) * 0.5
    return objp


@dataclass
class Intrinsics:
    K: np.ndarray  # (3,3)
    D: np.ndarray  # (N,) where N in {0,4,5,8,...}


def _intrinsics_from_camera_info(msg: "CameraInfo") -> Intrinsics:
    K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
    D = np.array(list(msg.D), dtype=np.float64).reshape(-1)
    return Intrinsics(K=K, D=D)


def _intrinsics_from_yaml(path: Path) -> Intrinsics:
    try:
        import yaml  # type: ignore
    except Exception as e:
        raise RuntimeError("缺少 PyYAML：请先 `pip install pyyaml` 或改用 --intrinsics_source camera_info") from e

    data = yaml.safe_load(path.read_text(encoding="utf-8", errors="ignore"))
    if not isinstance(data, dict):
        raise RuntimeError(f"无法解析内参 YAML：{path}")

    # 支持 ROS camera_calibration YAML 常见格式
    # camera_matrix: {rows: 3, cols: 3, data: [...]}
    # distortion_coefficients: {data: [...]}
    cam_m = data.get("camera_matrix") or data.get("K") or data.get("cameraMatrix")
    dist = data.get("distortion_coefficients") or data.get("D") or data.get("distCoeffs")
    if isinstance(cam_m, dict):
        cam_m = cam_m.get("data")
    if isinstance(dist, dict):
        dist = dist.get("data")

    if cam_m is None:
        raise RuntimeError(f"YAML 内缺少 camera_matrix/K 字段：{path}")
    K = np.array(cam_m, dtype=np.float64).reshape(3, 3)

    if dist is None:
        D = np.zeros((0,), dtype=np.float64)
    else:
        D = np.array(dist, dtype=np.float64).reshape(-1)
    return Intrinsics(K=K, D=D)


class ChessboardPoseLogger:
    def __init__(
        self,
        image_topic: str,
        camera_info_topic: str,
        out_csv: Path,
        nx: int,
        ny: int,
        square: float,
        origin_at_center: bool,
        intrinsics_source: str,
        intrinsics_yaml: Optional[Path],
        write_every_n_frames: int,
        display: bool,
        keyframe_flag_topic: str,
        keyframe_max_age_sec: float,
    ):
        if rospy is None:  # pragma: no cover
            raise RuntimeError(f"ROS 依赖导入失败：{_ros_import_error}")

        self.bridge = CvBridge()
        self.image_topic = image_topic
        self.camera_info_topic = camera_info_topic
        self.out_csv = out_csv
        self.nx, self.ny = int(nx), int(ny)
        self.square = float(square)
        self.origin_at_center = bool(origin_at_center)
        self.objp = _board_object_points(self.nx, self.ny, self.square, self.origin_at_center)

        self.intrinsics_source = intrinsics_source
        self.intrinsics_yaml = intrinsics_yaml
        self._intrinsics: Optional[Intrinsics] = None

        self.write_every_n_frames = max(1, int(write_every_n_frames))
        self.display = bool(display)
        self._frame_idx = 0

        self.keyframe_flag_topic = (keyframe_flag_topic or "").strip()
        self.keyframe_max_age_sec = max(0.0, float(keyframe_max_age_sec))
        self._pending_keyframe = False
        self._pending_keyframe_index: Optional[int] = None
        self._latest_pose = None  # dict with tx,ty,tz,qx,qy,qz,qw,rvec,rmse,ros_time,unix_time,frame_id

        self._ensure_csv_header()

        self._sub_img = rospy.Subscriber(self.image_topic, Image, self._on_image, queue_size=1, buff_size=2**24)
        self._sub_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self._on_camera_info, queue_size=1)
        self._sub_kf = None
        if self.keyframe_flag_topic:
            self._sub_kf = rospy.Subscriber(self.keyframe_flag_topic, Float64, self._on_keyframe_flag, queue_size=10)

        rospy.loginfo(f"[board_pose] image_topic={self.image_topic}")
        rospy.loginfo(f"[board_pose] camera_info_topic={self.camera_info_topic}")
        rospy.loginfo(f"[board_pose] out_csv={self.out_csv}")
        rospy.loginfo(f"[board_pose] board: points_x={self.nx}, points_y={self.ny}, square={self.square}, origin_at_center={self.origin_at_center}")
        rospy.loginfo(f"[board_pose] intrinsics_source={self.intrinsics_source} intrinsics_yaml={self.intrinsics_yaml}")
        if self.keyframe_flag_topic:
            rospy.loginfo(f"[board_pose] keyframe_flag_topic={self.keyframe_flag_topic} (keyframe-only logging)")
        else:
            rospy.loginfo(f"[board_pose] keyframe_flag_topic=<disabled> (realtime logging)")

    def _on_keyframe_flag(self, msg: "Float64") -> None:
        # 关键帧触发：msg.data 由 demo 发布为关键帧编号（1-based）。
        # 尝试写入“最新一帧有效棋盘位姿”；若太旧则等待下一次检测到棋盘再写。
        try:
            kf = int(round(float(getattr(msg, "data", 0.0))))
            self._pending_keyframe_index = kf if kf >= 1 else None
        except Exception:
            self._pending_keyframe_index = None
        self._pending_keyframe = True
        self._maybe_write_latest_pose(force_latest=True)

    def _ensure_csv_header(self) -> None:
        self.out_csv.parent.mkdir(parents=True, exist_ok=True)
        if self.out_csv.exists() and self.out_csv.stat().st_size > 0:
            return
        with self.out_csv.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "unix_time",
                    "ros_time",
                    "keyframe_index",
                    "frame_id",
                    "points_x",
                    "points_y",
                    "square_size_m",
                    "origin_at_center",
                    "t_x",
                    "t_y",
                    "t_z",
                    "q_x",
                    "q_y",
                    "q_z",
                    "q_w",
                    "rvec_x",
                    "rvec_y",
                    "rvec_z",
                    "reproj_rmse_px",
                ]
            )

    def _on_camera_info(self, msg: "CameraInfo") -> None:
        if self.intrinsics_source != "camera_info":
            return
        self._intrinsics = _intrinsics_from_camera_info(msg)

    def _get_intrinsics(self) -> Intrinsics:
        if self.intrinsics_source == "yaml":
            if self._intrinsics is None:
                if not self.intrinsics_yaml:
                    raise RuntimeError("intrinsics_source=yaml 但未提供 --intrinsics_yaml")
                self._intrinsics = _intrinsics_from_yaml(Path(self.intrinsics_yaml))
            return self._intrinsics

        if self._intrinsics is None:
            raise RuntimeError("尚未收到 CameraInfo，无法计算位姿")
        return self._intrinsics

    def _on_image(self, msg: "Image") -> None:
        self._frame_idx += 1
        if not self.keyframe_flag_topic:
            if (self._frame_idx - 1) % self.write_every_n_frames != 0:
                return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[board_pose] CvBridge 转换失败: {e}")
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        pattern_size = (self.nx, self.ny)

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if not ok or corners is None:
            if self.display:
                cv2.imshow("board_pose", cv_img)
                cv2.waitKey(1)
            return

        # refine
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, winSize=(11, 11), zeroZone=(-1, -1), criteria=term)

        try:
            intr = self._get_intrinsics()
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[board_pose] 内参未就绪: {e}")
            return

        # solvePnP: returns rvec,tvec such that X_cam = R * X_obj + t
        ok, rvec, tvec = cv2.solvePnP(self.objp, corners2, intr.K, intr.D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return

        R, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = _quat_from_R(R)
        tx, ty, tz = float(tvec[0]), float(tvec[1]), float(tvec[2])

        # reprojection rmse
        proj, _ = cv2.projectPoints(self.objp, rvec, tvec, intr.K, intr.D)
        proj = proj.reshape(-1, 2)
        obs = corners2.reshape(-1, 2)
        err = proj - obs
        rmse = float(np.sqrt(np.mean(np.sum(err * err, axis=1))))

        unix_time = time.time()
        ros_time = msg.header.stamp.to_sec()
        frame_id = msg.header.frame_id or ""

        self._latest_pose = {
            "unix_time": unix_time,
            "ros_time": ros_time,
            "frame_id": frame_id,
            "tx": tx,
            "ty": ty,
            "tz": tz,
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
            "rvec_x": float(rvec[0]),
            "rvec_y": float(rvec[1]),
            "rvec_z": float(rvec[2]),
            "rmse": rmse,
        }

        if self.keyframe_flag_topic:
            self._maybe_write_latest_pose(force_latest=False)
        else:
            self._append_pose_row(self._latest_pose)

        if self.display:
            cv2.drawChessboardCorners(cv_img, pattern_size, corners2, True)
            txt = f"t=({tx:.3f},{ty:.3f},{tz:.3f})m rmse={rmse:.2f}px"
            cv2.putText(cv_img, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("board_pose", cv_img)
            cv2.waitKey(1)

    def _append_pose_row(self, pose: dict) -> None:
        with self.out_csv.open("a", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    f"{pose['unix_time']:.6f}",
                    f"{pose['ros_time']:.6f}",
                    str(int(pose.get("keyframe_index") or 0)),
                    pose["frame_id"],
                    self.nx,
                    self.ny,
                    f"{self.square:.6f}",
                    int(self.origin_at_center),
                    f"{pose['tx']:.6f}",
                    f"{pose['ty']:.6f}",
                    f"{pose['tz']:.6f}",
                    f"{pose['qx']:.8f}",
                    f"{pose['qy']:.8f}",
                    f"{pose['qz']:.8f}",
                    f"{pose['qw']:.8f}",
                    f"{pose['rvec_x']:.8f}",
                    f"{pose['rvec_y']:.8f}",
                    f"{pose['rvec_z']:.8f}",
                    f"{pose['rmse']:.4f}",
                ]
            )

    def _maybe_write_latest_pose(self, force_latest: bool) -> None:
        if not self._pending_keyframe:
            return
        if self._latest_pose is None:
            return
        if not force_latest and self.keyframe_max_age_sec > 1e-9:
            age = abs(rospy.Time.now().to_sec() - float(self._latest_pose["ros_time"]))
            if age > self.keyframe_max_age_sec:
                return
        self._latest_pose["keyframe_index"] = self._pending_keyframe_index
        self._append_pose_row(self._latest_pose)
        self._pending_keyframe = False
        self._pending_keyframe_index = None


def main() -> int:
    ap = argparse.ArgumentParser(description="订阅相机图像，识别棋盘并将棋盘相对相机的位姿写入 CSV（T_cam_board）")
    ap.add_argument("--image_topic", required=True)
    ap.add_argument("--camera_info_topic", required=True)
    ap.add_argument("--out_csv", required=True)
    ap.add_argument("--points_x", type=int, default=11)
    ap.add_argument("--points_y", type=int, default=8)
    ap.add_argument("--square_size", type=float, default=0.03)
    ap.add_argument("--origin_at_center", action="store_true")
    ap.add_argument("--intrinsics_source", choices=["camera_info", "yaml"], default="camera_info")
    ap.add_argument("--intrinsics_yaml", default=None)
    ap.add_argument("--write_every_n_frames", type=int, default=1, help="仅 realtime 模式有效：每 N 帧写一次")
    ap.add_argument("--display", action="store_true")
    ap.add_argument("--keyframe_flag_topic", default="", help="关键帧触发话题（std_msgs/Float64）。设置后仅在触发时写入一行 CSV。")
    ap.add_argument("--keyframe_max_age_sec", type=float, default=0.5, help="关键帧写入时允许使用的“最新检测”最大时间差（秒）。")
    ap.add_argument("--realtime", action="store_true", help="启用实时采集（不推荐）。未开启时必须提供 --keyframe_flag_topic。")
    args = ap.parse_args()

    if rospy is None:  # pragma: no cover
        sys.stderr.write(f"ROS 依赖导入失败：{_ros_import_error}\n")
        return 2

    out_csv = Path(args.out_csv).expanduser()
    intr_yaml = Path(args.intrinsics_yaml).expanduser() if args.intrinsics_yaml else None
    if intr_yaml is not None and not intr_yaml.is_file():
        raise SystemExit(f"内参 YAML 不存在: {intr_yaml}")

    rospy.init_node("chessboard_pose_logger", anonymous=True)
    if (not args.realtime) and (not (args.keyframe_flag_topic or "").strip()):
        raise SystemExit("默认使用关键帧采样：请提供 --keyframe_flag_topic；若确实要实时采集，请显式加 --realtime。")
    _ = ChessboardPoseLogger(
        image_topic=args.image_topic,
        camera_info_topic=args.camera_info_topic,
        out_csv=out_csv,
        nx=args.points_x,
        ny=args.points_y,
        square=args.square_size,
        origin_at_center=args.origin_at_center,
        intrinsics_source=args.intrinsics_source,
        intrinsics_yaml=intr_yaml,
        write_every_n_frames=args.write_every_n_frames if args.realtime else 1,
        display=args.display,
        keyframe_flag_topic=args.keyframe_flag_topic,
        keyframe_max_age_sec=args.keyframe_max_age_sec,
    )
    rospy.loginfo("[board_pose] running. Ctrl-C to stop.")
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

