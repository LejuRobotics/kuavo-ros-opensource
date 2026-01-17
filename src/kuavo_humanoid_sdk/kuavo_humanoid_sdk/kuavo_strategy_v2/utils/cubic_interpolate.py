import numpy as np
from scipy.spatial.transform import Rotation as R
from pydrake.trajectories import PiecewisePolynomial
from scipy.spatial.transform import Slerp, Rotation as R


class CubicInterpolator:
    def __init__(self, t_values=None, pos_list=None):
        self.t_values = []
        self.interpolant = None
        self.quat_list = []
        self.force_list = []

        if t_values is not None and pos_list is not None:
            self._init_pos_interpolant(t_values, pos_list)

    def _init_pos_interpolant(self, t_values, pos_list):
        if len(t_values) != len(pos_list):
            raise ValueError("t_values and pos_list must have the same length.")

        self.t_values = list(t_values)
        num = pos_list[0].shape[0]

        # pos_list: List[np.ndarray] -> y_values: (dim, N)
        y_values = np.column_stack(pos_list)  # Shape: (dim, N)

        # Construct cubic spline with zero end derivatives
        self.interpolant = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            self.t_values,
            y_values,
            np.zeros(num),  # start velocity
            np.zeros(num),  # end velocity
        )

    def add_quaternion(self, quat_list):
        if len(quat_list) != len(self.t_values):
            raise ValueError("quat_list length must match t_values length.")
        self.quat_list = quat_list

    def add_force(self, force_list):
        if len(force_list) != len(self.t_values):
            raise ValueError("force_list length must match t_values length.")
        self.force_list = force_list

    def get_pos(self, t):
        t = min(t, self.t_values[-1])
        return self.interpolant.value(t).flatten()

    def get_vel(self, t):
        t = min(t, self.t_values[-1])
        return self.interpolant.derivative(1).value(t).flatten()

    def get_acc(self, t):
        t = min(t, self.t_values[-1])
        return self.interpolant.derivative(2).value(t).flatten()

    def get_quat(self, t):
        if not self.quat_list:
            raise RuntimeError("Quaternion list not set.")

        index = len(self.t_values) - 1
        for i, ti in enumerate(self.t_values):
            if ti > t:
                index = i - 1
                break

        if index >= len(self.quat_list) - 1:
            return self.quat_list[-1]

        t1 = self.t_values[index]
        t2 = self.t_values[index + 1]
        alpha = (t - t1) / (t2 - t1)

        # Create a slerp interpolator between two rotations
        key_times = [0, 1]
        key_rots = R.from_quat([self.quat_list[index], self.quat_list[index + 1]])
        slerp = Slerp(key_times, key_rots)
        interp_rot = slerp([alpha])[0]
        return interp_rot.as_quat()

    def get_force(self, t):
        if not self.force_list:
            raise RuntimeError("Force list not set.")

        index = len(self.t_values) - 1
        for i, ti in enumerate(self.t_values):
            if ti > t:
                index = i - 1
                break

        if index >= len(self.force_list) - 1:
            return self.force_list[-1]

        f1 = self.force_list[index]
        f2 = self.force_list[index + 1]
        t1 = self.t_values[index]
        t2 = self.t_values[index + 1]
        alpha = (t - t1) / (t2 - t1)
        return f1 * (1 - alpha) + f2 * alpha

    def get_end_time(self):
        return self.t_values[-1]


if __name__ == "__main__":
    import numpy as np

    # 时间和位置（假设每个位置是3维）
    t_values = [0.0, 1.0, 2.0]
    pos_list = [np.array([0, 0, 0]), np.array([1, 1, 1]), np.array([2, 0, 2])]
    quat_list = [np.array([0, 0, 0, 1]), np.array([0.707, 0, 0, 0.707]), np.array([1, 0, 0, 0])]
    force_list = [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]

    interp = CubicInterpolator(t_values, pos_list)
    interp.add_quaternion(quat_list)
    interp.add_force(force_list)

    print("Position at t=1.5:", interp.get_pos(1.5))
    print("Quaternion at t=1.5:", interp.get_quat(1.5))
    print("Force at t=1.5:", interp.get_force(1.5))
