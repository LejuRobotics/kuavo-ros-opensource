import numpy as np
import scipy

class KalmanFilter3D:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        """
        初始化卡尔曼滤波器
        :param initial_state: 初始状态 [x, y, z, vx, vy, vz]
        :param initial_covariance: 初始协方差矩阵
        :param process_noise: 过程噪声协方差矩阵
        :param measurement_noise: 测量噪声协方差矩阵
        """
        self.state = initial_state  # 状态 [x, y, z, vx, vy, vz]
        self.covariance = initial_covariance  # 状态协方差矩阵
        self.process_noise = process_noise  # 过程噪声协方差矩阵
        self.measurement_noise = measurement_noise  # 测量噪声协方差矩阵

        # 状态转移矩阵
        self.F = np.array([[1, 0, 0, 1, 0, 0],
                           [0, 1, 0, 0, 1, 0],
                           [0, 0, 1, 0, 0, 1],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])

        # 测量矩阵
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0]])

    def predict(self):
        """
        预测步骤
        """
        self.state = self.F @ self.state
        self.covariance = self.F @ self.covariance @ self.F.T + self.process_noise

    def update(self, measurement):
        """
        更新步骤
        :param measurement: 测量值 [x, y, z]
        """
        innovation = measurement - self.H @ self.state
        innovation_covariance = self.H @ self.covariance @ self.H.T + self.measurement_noise
        kalman_gain = self.covariance @ self.H.T @ scipy.linalg.inv(innovation_covariance)

        self.state = self.state + kalman_gain @ innovation
        self.covariance = (np.eye(6) - kalman_gain @ self.H) @ self.covariance

    def filter(self, measurement):
        """
        执行一次滤波
        :param measurement: 测量值 [x, y, z]
        :return: 滤波后的状态 [x, y, z]
        """
        self.predict()
        self.update(measurement)
        return self.state[:3]

# 示例使用
if __name__ == "__main__":
    initial_state = np.array([0, 0, 0, 0, 0, 0])  # 初始状态 [x, y, z, vx, vy, vz]
    initial_covariance = np.eye(6)  # 初始协方差矩阵
    process_noise = np.eye(6) * 0.01  # 过程噪声协方差矩阵
    measurement_noise = np.eye(3) * 0.1  # 测量噪声协方差矩阵

    kf = KalmanFilter3D(initial_state, initial_covariance, process_noise, measurement_noise)

    measurements = [np.array([1.1, 0.9, 1.0]), np.array([2.1, 1.9, 2.0]), np.array([3.1, 3.0, 3.1])]

    for measurement in measurements:
        filtered_state = kf.filter(measurement)
        print("Filtered State:", filtered_state)
