
from dataclasses import dataclass
import time
from typing import Callable, Dict, Tuple, Any, Optional

@dataclass
class ScoringConfig1:
    # 区域与方向阈值
    toy_box_region: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]
    car_box_region: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]

    # # 计分规则
    time_full: int = 20                    # 时间满分
    time_threshold_sec: int = 20           # ≤10秒得满分
    time_penalty_per_sec: int = 1          # 超出每秒扣2分，最低0

class ScoringEvaluator1:
    def __init__(
        self,
        config: ScoringConfig1,
        is_in_region_fn: Callable[[Tuple[float, float, float], Any], bool],
    ):
        self.cfg = config
        self._is_in_region = is_in_region_fn

        self.score: int = 0
        self.already_reported_success: bool = False

        self.toy_pos_awarded: bool = False
        self.car_pos_awarded: bool = False

        self.start_time: float = time.time()

    def reset(self, start_time: Optional[float] = None):
        self.score = 0
        self.already_reported_success = False

        self.toy_pos_awarded = False
        self.car_pos_awarded = False

        self.start_time = time.time() if start_time is None else start_time

    def evaluate(
        self,
        pos_xyz_toy: Tuple[float, float, float],
        pos_xyz_car: Tuple[float, float, float],

        now: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        输入：当前位置/朝向，返回一次评估结果。
        只做纯逻辑，不做任何发布/IO。
        """
        now = time.time() if now is None else now
        result = {
            # 判定输出
            "in_region_toy": False,
            "in_region_car": False,

            "success_triggered": False,        # 本帧首次触发终点成功
            "need_publish_success_true": False,
            "need_publish_success_false": False,  # 在未成功阶段持续发 False

            # 计分输出
            "score_delta": 0,
            "total_score": self.score,
            "elapsed_sec": now - self.start_time,

            "toy_pos_added": False,
            "car_pos_added": False,
            "time_score_added": 0,    # 时间分
        }

        try:
            in_region_toy = self._is_in_region(pos_xyz_toy, self.cfg.toy_box_region)
            in_region_car = self._is_in_region(pos_xyz_car, self.cfg.car_box_region)
            
        except Exception:
            in_region_toy = False
            in_region_car = False

        result["in_region_toy"] = in_region_toy
        result["in_region_car"] = in_region_car

        # —— 反面物体加分 —— #
        if in_region_toy:
            # 位置10分（只加一次）
            if not self.toy_pos_awarded:
                self.toy_pos_awarded = True
                self.score += 40
                result["score_delta"] += 40
                result["toy_pos_added"] = True

        # —— 反面物体加分 —— #
        if in_region_car:
            # 位置10分（只加一次）
            if not self.car_pos_awarded:
                self.car_pos_awarded = True
                self.score += 40
                result["score_delta"] += 40
                result["car_pos_added"] = True

        # —— 终点成功 —— #
        if self.toy_pos_awarded and self.car_pos_awarded and (not self.already_reported_success):
            self.already_reported_success = True
            result["success_triggered"] = True
            result["need_publish_success_true"] = True
            result["need_stop_conveyor"] = True

            # 时间分
            elapsed = now - self.start_time
            if elapsed <= self.cfg.time_threshold_sec:
                time_score = self.cfg.time_full
            else:
                over = elapsed - self.cfg.time_threshold_sec
                time_score = max(0, self.cfg.time_full - over * self.cfg.time_penalty_per_sec)

            self.score += time_score
            result["score_delta"] += time_score
            result["elapsed_sec"] = elapsed
            result["time_score_added"] = int(time_score)
        else:
            # 未成功阶段建议持续发 False
            if not self.already_reported_success:
                result["need_publish_success_false"] = True

        result["total_score"] = self.score
        return result
@dataclass
class ScoringConfig2:
    # 区域与方向阈值
    target_region: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]
    intermediate_region: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]

    # # 计分规则
    time_full: int = 20                    # 时间满分
    time_threshold_sec: int = 30           # ≤12秒得满分
    time_penalty_per_sec: int = 2          # 超出每秒扣2分，最低0
class ScoringEvaluator2:
    def __init__(
        self,
        config: ScoringConfig2,
        is_in_region_fn: Callable[[Tuple[float, float, float], Any], bool],
    ):
        self.cfg = config
        self._is_in_region = is_in_region_fn

        self.score: int = 0
        self.already_reported_success: bool = False

        self.intermediate_pos_awarded: bool = False

        self.final_pos_awarded: bool = False
      
        self.start_time: float = time.time()

    def reset(self, start_time: Optional[float] = None):
        self.score = 0
        self.already_reported_success = False
        self.intermediate_pos_awarded = False
        self.final_pos_awarded = False
        self.start_time = time.time() if start_time is None else start_time

    def evaluate(
        self,
        pos_xyz: Tuple[float, float, float],
        now: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        输入：当前位置，返回一次评估结果。
        只做纯逻辑，不做任何发布/IO。
        """
        now = time.time() if now is None else now
        result = {
            # 判定输出
            "in_region": False,
            "in_intermediate": False,

            # 事件标志（用于上层决定ROS行为）      
            "need_publish_success_true": False,
            "need_publish_success_false": False,  # 在未成功阶段持续发 False

            # 设备控制
            "need_stop_conveyor": False,       # 终点成功后停传送带

            # 计分输出
            "score_delta": 0,
            "total_score": self.score,
            "elapsed_sec": now - self.start_time,
            "intermediate_pos_added": False,
            "final_pos_added": False,   # 30 或 0
            "time_score_added": 0,    # 时间分
        }

        try:
            in_region = self._is_in_region(pos_xyz, self.cfg.target_region)
            in_intermediate = self._is_in_region(pos_xyz, self.cfg.intermediate_region)

        except Exception:
            in_region = False
            in_intermediate = False

        result["in_region"] = in_region
        result["in_intermediate"] = in_intermediate


        # —— 中间点一次性加分 —— #
        if in_intermediate:
            # 位置40分（只加一次）
            if not self.intermediate_pos_awarded:
                self.intermediate_pos_awarded = True
                self.score += 40
                result["score_delta"] += 40
                result["intermediate_pos_added"] = True

        if in_region:
            # 位置40分（只加一次）
            if not self.final_pos_awarded:
                self.final_pos_awarded = True
                self.score += 40
                result["score_delta"] += 40
                result["final_pos_added"] = True


        # —— 终点成功 —— #
        if in_region and (not self.already_reported_success):
            self.already_reported_success = True
            result["need_publish_success_true"] = True
            result["need_stop_conveyor"] = True

            # # 时间分
            elapsed = now - self.start_time
            if elapsed <= self.cfg.time_threshold_sec:
                time_score = self.cfg.time_full
            else:
                over = elapsed - self.cfg.time_threshold_sec
                time_score = max(0, self.cfg.time_full - over * self.cfg.time_penalty_per_sec)

            self.score += time_score
            result["score_delta"] += time_score
            result["elapsed_sec"] = elapsed
            result["time_score_added"] = int(time_score)
        else:
            # 未成功阶段建议持续发 False
            if not self.already_reported_success:
                result["need_publish_success_false"] = True

        result["total_score"] = self.score
        return result

@dataclass
class ScoringConfig3:
    # 区域与方向阈值
    black_bin_region: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]
    red_bin_region: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]


    # # 计分规则
    time_full: int = 10                    # 时间满分
    time_threshold_sec: int = 40           # ≤12秒得满分
    time_penalty_per_sec: int = 2          # 超出每秒扣2分，最低0
class ScoringEvaluator3:
    def __init__(
        self,
        config: ScoringConfig3,
        is_in_region_fn: Callable[[Tuple[float, float, float], Any], bool],
    ):
        self.cfg = config
        self._is_in_region = is_in_region_fn

        self.score: int = 0
        self.already_reported_success: bool = False

        self.box_black1_pos_awarded: bool = False
        self.box_red1_pos_awarded: bool = False
        self.box_black2_pos_awarded: bool = False
        self.box_red2_pos_awarded: bool = False
        self.bonus_awarded: bool = False

        self.start_time: float = time.time()

    def reset(self, start_time: Optional[float] = None):
        self.score = 0
        self.already_reported_success = False

        self.box_black1_pos_awarded = False
        self.box_red1_pos_awarded = False
        self.box_black2_pos_awarded = False
        self.box_red2_pos_awarded = False
        self.bonus_awarded = False

        self.start_time = time.time() if start_time is None else start_time

    def evaluate(
        self,
        pos_xyz_box_red1: Tuple[float, float, float],
        pos_xyz_box_black1: Tuple[float, float, float],
        pos_xyz_box_red2: Tuple[float, float, float],
        pos_xyz_box_black2: Tuple[float, float, float],
        now: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        输入：当前位置/朝向，返回一次评估结果。
        只做纯逻辑，不做任何发布/IO。
        """
        now = time.time() if now is None else now
        result = {
            # 判定输出
            "box_red1_in_region": False,
            "box_black1_in_region": False,
            "box_red2_in_region": False,
            "box_black2_in_region": False,

            # 事件标志（用于上层决定ROS行为）
            "need_publish_success_true": False,
            "need_publish_success_false": False,  # 在未成功阶段持续发 False

            # 设备控制
            "need_stop_conveyor": False,       # 终点成功后停传送带

            # 计分输出
            "score_delta": 0,
            "total_score": self.score,
            "elapsed_sec": now - self.start_time,
            "box_red1_pos_added": False,
            "box_black1_pos_added": False,
            "box_red2_pos_added": False,
            "box_black2_pos_added": False,
            "bonus_added": False,
            "success_triggered": False,
            "time_score_added": 0,    # 时间分
        }
        try:
            box_red1_in_region = self._is_in_region(pos_xyz_box_red1, self.cfg.red_bin_region)
            box_black1_in_region = self._is_in_region(pos_xyz_box_black1, self.cfg.black_bin_region)
            box_red2_in_region = self._is_in_region(pos_xyz_box_red2, self.cfg.red_bin_region)
            box_black2_in_region = self._is_in_region(pos_xyz_box_black2, self.cfg.black_bin_region)
        except Exception:
            box_red1_in_region = False
            box_black1_in_region = False
            box_red2_in_region = False
            box_black2_in_region = False

        result["box_red1_in_region"] = box_red1_in_region
        result["box_black1_in_region"] = box_black1_in_region
        result["box_red2_in_region"] = box_red2_in_region
        result["box_black2_in_region"] = box_black2_in_region

        # —— 中间点一次性加分 —— #
        if box_red1_in_region:
            # 位置20分（只加一次）
            if not self.box_red1_pos_awarded:
                self.box_red1_pos_awarded = True
                self.score += 20
                result["score_delta"] += 20
                result["box_red1_pos_added"] = True

        if box_black1_in_region:
            # 位置20分（只加一次）
            if not self.box_black1_pos_awarded:
                self.box_black1_pos_awarded = True
                self.score += 20
                result["score_delta"] += 20
                result["box_black1_pos_added"] = True
        
        if box_red2_in_region:
            # 位置20分（只加一次）
            if not self.box_red2_pos_awarded:
                self.box_red2_pos_awarded = True
                self.score += 20
                result["score_delta"] += 20
                result["box_red2_pos_added"] = True
        
        if box_black2_in_region:
            # 位置20分（只加一次）
            if not self.box_black2_pos_awarded:
                self.box_black2_pos_awarded = True
                self.score += 20
                result["score_delta"] += 20
                result["box_black2_pos_added"] = True

        # —— 终点成功 —— #
        if box_red1_in_region and box_black1_in_region and box_red2_in_region and box_black2_in_region and (not self.already_reported_success):
            self.bonus_awarded
            self.score += 10
            result["score_delta"] += 10
            result["bonus_added"] = True
            result["success_triggered"] = True
            self.already_reported_success = True
            result["need_publish_success_true"] = True
            result["need_stop_conveyor"] = True

            # # 时间分
            elapsed = now - self.start_time
            if elapsed <= self.cfg.time_threshold_sec:
                time_score = self.cfg.time_full
            else:
                over = elapsed - self.cfg.time_threshold_sec
                time_score = max(0, self.cfg.time_full - over * self.cfg.time_penalty_per_sec)

            self.score += time_score
            result["score_delta"] += time_score
            result["elapsed_sec"] = elapsed
            result["time_score_added"] = int(time_score)
        else:
            # 未成功阶段建议持续发 False
            if not self.already_reported_success:
                result["need_publish_success_false"] = True

        result["total_score"] = self.score
        return result
