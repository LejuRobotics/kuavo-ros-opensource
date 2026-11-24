"""Route board frames to independent hand solver sessions."""

from .pose import PoseTracker
from .sdk import SG100SDK
from .uwb_decode import to_sdk_arrays


class HandRouter:
    def __init__(self, board_ids, *, ini_paths=None, clock=None, stale_timeout=2.0):
        if not board_ids or set(board_ids) - {"left", "right"}:
            raise ValueError("select at least one hand")
        values = list(board_ids.values())
        if any(type(value) is not int or not 0 <= value <= 0xFFFFFFFF for value in values):
            raise ValueError("board ID must be an unsigned 32-bit integer")
        if len(set(values)) != len(values):
            raise ValueError("left and right board IDs must be different")
        self.board_ids = dict(board_ids)
        self._hand_by_id = {value: hand for hand, value in board_ids.items()}
        self.stale_timeout = stale_timeout
        paths = ini_paths or {}
        self.solvers = {
            hand: SG100SDK(paths.get(hand), hand=hand, clock=clock) for hand in board_ids
        }
        self.trackers = {hand: PoseTracker() for hand in board_ids}
        self._last_seen = dict.fromkeys(board_ids)
        self._online = dict.fromkeys(board_ids, False)
        self._needs_reset = dict.fromkeys(board_ids, False)

    def process(self, frame, now):
        hand = self._hand_by_id.get(frame["board_id"])
        if hand is None:
            return None
        previous = self._last_seen[hand]
        if previous is not None and now - previous >= self.stale_timeout:
            self._online[hand] = False
            self._needs_reset[hand] = True
        became_online = not self._online[hand]
        if self._needs_reset[hand]:
            self.solvers[hand].reset()
            self.trackers[hand].reset()
            self._needs_reset[hand] = False
        position, orientation = to_sdk_arrays(frame)
        position, orientation = self.trackers[hand].process(position, orientation, now)
        angles = self.solvers[hand].solve(position, orientation)
        self._last_seen[hand] = now
        self._online[hand] = True
        return {
            "hand": hand,
            "board_id": frame["board_id"],
            "seq": frame["seq"],
            "joint_angles": angles,
            "became_online": became_online,
            # 消歧后的五指空间数据 (与 solve() 输入完全相同的数组):
            # position[15]/orientation[20], wire 顺序 thumb,index,middle,ring,pinky,
            # 四元数 wxyz; 仅供 ROS 诊断/手势分支使用, 不参与 IK。
            "tracked_positions": position,
            "tracked_orientations": orientation,
        }

    def expire(self, now):
        expired = []
        for hand, last in self._last_seen.items():
            if self._online[hand] and last is not None and now - last >= self.stale_timeout:
                self._online[hand] = False
                self._needs_reset[hand] = True
                expired.append(hand)
        return expired

    def reset(self):
        for hand in self.solvers:
            self._online[hand] = False
            self._last_seen[hand] = None
            self._needs_reset[hand] = True
