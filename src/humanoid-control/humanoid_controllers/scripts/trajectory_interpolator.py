#!/usr/bin/env python3

import numpy as np
from scipy.interpolate import PchipInterpolator
from kuavo_msgs.msg import footPose, footPoses

class TrajectoryInterpolator:
    """轨迹插值器类，提供多种插值方法"""
    
    def __init__(self):
        self.methods = {
            "trigonometric_quintic": self._trigonometric_quintic_interpolation,
            "spline": self._spline_interpolation
        }
    
    def interpolate_trajectory(self, prev_foot_pose, next_foot_pose, swing_height=0.00, 
                              method="ellipse_quintic", num_points=7, is_first_step=False, 
                              down_stairs=False, plot=False):
        """
        轨迹插值主函数
        
        Args:
            prev_foot_pose: 上一个落点位置 [x, y, z, yaw]
            next_foot_pose: 下一个落点位置 [x, y, z, yaw]
            swing_height: 抬脚最大高度，默认0.10米
            method: 插值方法 ("ellipse_quintic", "spline")
            num_points: 轨迹点数量（固定为7，与三次样条保持一致）
            is_first_step: 是否为第一步
            down_stairs: 是否为下楼梯
            plot: 是否绘制轨迹图
            
        Returns:
            additionalFootPoseTrajectory: 包含腾空相轨迹的footPoses消息
        """
        """
        轨迹插值主函数
        
        Args:
            prev_foot_pose: 上一个落点位置 [x, y, z, yaw]
            next_foot_pose: 下一个落点位置 [x, y, z, yaw]
            swing_height: 抬脚最大高度，默认0.10米
            method: 插值方法 ("ellipse_quintic", "spline", "cycloid")
            num_points: 轨迹点数量
            is_first_step: 是否为第一步
            down_stairs: 是否为下楼梯
            plot: 是否绘制轨迹图
            
        Returns:
            additionalFootPoseTrajectory: 包含腾空相轨迹的footPoses消息
        """
        if method not in self.methods:
            print(f"警告：未知的插值方法 '{method}'，使用默认的三角函数+五次多项式方法")
            method = "trigonometric_quintic"
        
        return self.methods[method](prev_foot_pose, next_foot_pose, swing_height, 
                                   num_points, is_first_step, down_stairs, plot)
    
    def _trigonometric_quintic_interpolation(self, prev_foot_pose, next_foot_pose, swing_height, 
                                           num_points, is_first_step, down_stairs, plot):
        """三角函数+五次多项式插值方法（Z方向使用三角函数，XY方向使用摆线）"""
        additionalFootPoseTrajectory = footPoses()
        
        # 计算移动距离
        x_distance = next_foot_pose[0] - prev_foot_pose[0]
        y_distance = next_foot_pose[1] - prev_foot_pose[1]
        z_distance = next_foot_pose[2] - prev_foot_pose[2]
        
        # 下楼梯时使用反向规划（先多项式再三角函数）
        if down_stairs:
            return self._trigonometric_quintic_interpolation_downstairs(prev_foot_pose, next_foot_pose, swing_height, 
                                                                      num_points, is_first_step)
        
        # 三角函数参数设置（上楼梯）
        if is_first_step:
            # 第一步：更保守的参数
            trig_ratio = 0.6  # 三角函数部分占比
            max_height_ratio = 1.0  # 最高点相对于总高度的比例
        else:
            # 后续步骤：优化参数
            trig_ratio = 0.6  # 三角函数部分占比
            max_height_ratio = 0.9  # 最高点相对于总高度的比例
        
        # 计算基准高度（取两个落点中较高的点）
        base_height = max(prev_foot_pose[2], next_foot_pose[2])
        min_height = min(prev_foot_pose[2], next_foot_pose[2])
        
        # 三角函数最高点高度参考三次样条：base_height + swing_height
        max_height = base_height + swing_height
        
        print(f"三角函数参数：")
        print(f"  基准高度 = {base_height:.3f}, 最低高度 = {min_height:.3f}")
        print(f"  最高点高度 = {max_height:.3f} (基准高度 + swing_height)")
        print(f"  三角函数占比 = {trig_ratio:.2f}")
        print(f"  XY方向插值 = 摆线插值")
        print(f"  Z方向插值 = 三角函数（正弦函数）")
        
        # 1. 生成三角函数轨迹的4个控制点（Z方向三角函数，XY方向摆线）
        trig_control_points = []
        
        # 使用三角函数生成控制点（确保在最高点零加速度）
        trig_progress = [0.0, 0.33, 0.67, 1.0]  # 三角函数内部进度
        
        for i, progress in enumerate(trig_progress):
            # 计算平滑进度（使用三次多项式确保在t=1时导数为0）
            t = progress
            smooth_progress = 3 * t**2 - 2 * t**3  # 三次多项式，在t=1时导数为0
            
            # 三角函数方程（Z方向）- 使用正弦函数从起点到最高点
            # z = start_z + (max_height - start_z) * sin(π/2 * smooth_progress)
            start_z = prev_foot_pose[2]
            z = start_z + (max_height - start_z) * np.sin(np.pi/2 * smooth_progress)
            
            # XY方向使用摆线插值
            # 摆线参数：t从0到1
            t_cycloid = progress * trig_ratio  # 归一化到三角函数部分的时间
            
            # 摆线方程：x = t - sin(t), y = 1 - cos(t)
            # 映射到实际坐标
            cycloid_x = t_cycloid - np.sin(2 * np.pi * t_cycloid) / (2 * np.pi)
            cycloid_y = (1 - np.cos(2 * np.pi * t_cycloid)) / 2
            
            # 映射到实际XY坐标
            x = prev_foot_pose[0] + x_distance * cycloid_x
            y = prev_foot_pose[1] + y_distance * cycloid_y
            
            trig_control_points.append([x, y, z])
            
            # 在最高点验证零加速度
            if abs(smooth_progress - 1.0) < 0.01:  # 接近最高点
                print(f"  最高点验证：progress={smooth_progress:.3f}, z={z:.3f}, 加速度为零")
                print(f"  平滑进度：{smooth_progress:.3f}, 时间参数：{t:.3f}")
        
        print(f"三角函数控制点（4个点，输出时删除起始点）：")
        for i, point in enumerate(trig_control_points):
            point_type = "起始点" if i == 0 else "中间点" if i == 1 else "中间点" if i == 2 else "终点"
            print(f"  点{i+1} ({point_type}): ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")
        
        # 2. 生成多项式轨迹的控制点
        polynomial_control_points = []
        
        # 控制点1：多项式起点（后移，避免与三角函数末端重合）
        t_poly_start = trig_ratio + (1 - trig_ratio) * 0.32  # 三角函数占比后32%位置
        
        # XY方向使用摆线规划
        cycloid_x_poly_start = t_poly_start - np.sin(2 * np.pi * t_poly_start) / (2 * np.pi)
        cycloid_y_poly_start = (1 - np.cos(2 * np.pi * t_poly_start)) / 2
        
        x_poly_start = prev_foot_pose[0] + x_distance * cycloid_x_poly_start
        y_poly_start = prev_foot_pose[1] + y_distance * cycloid_y_poly_start
        
        # Z方向平滑下降（从三角函数终点高度开始）
        z_trig_end = trig_control_points[-1][2]  # 三角函数终点高度
        z_poly_start = z_trig_end + (next_foot_pose[2] - z_trig_end) * 0.15  # 下降15%
        polynomial_control_points.append([x_poly_start, y_poly_start, z_poly_start])
        
        # 控制点2：中间点（使用摆线插值）
        t_mid = trig_ratio + (1 - trig_ratio) * 0.64  # 多项式部分64%位置
        
        # 摆线插值
        cycloid_x_mid = t_mid - np.sin(2 * np.pi * t_mid) / (2 * np.pi)
        cycloid_y_mid = (1 - np.cos(2 * np.pi * t_mid)) / 2
        
        x_mid = prev_foot_pose[0] + x_distance * cycloid_x_mid
        y_mid = prev_foot_pose[1] + y_distance * cycloid_y_mid
        z_mid = next_foot_pose[2] + (z_poly_start - next_foot_pose[2]) * 0.5  # 平滑下降
        polynomial_control_points.append([x_mid, y_mid, z_mid])
        
        # 控制点3：目标位置
        x_end = next_foot_pose[0]
        y_end = next_foot_pose[1]
        z_end = next_foot_pose[2]
        polynomial_control_points.append([x_end, y_end, z_end])
        
        print(f"多项式控制点（3个点，删除重合点）：")
        for i, point in enumerate(polynomial_control_points):
            point_type = "多项式起点" if i == 0 else "中间点" if i == 1 else "终点"
            print(f"  点{i+1} ({point_type}): ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")
        
        # 3. 生成完整轨迹（7个控制点：4个三角函数点 + 3个多项式点）
        full_trajectory = trig_control_points + polynomial_control_points
        
        # 删除第一个点（三角函数起始点）和最后一个点（多项式终点）
        full_trajectory = full_trajectory[1:-1]
        
        # 4. 生成时间序列（调整时间分布，让后半段更均匀）
        # 时间分配：三角函数部分占trig_ratio，多项式部分占(1-trig_ratio)
        # 延长三角函数部分时间，让抬腿更慢
        extended_trig_ratio = trig_ratio * 1.3  # 延长30%
        trig_times = [extended_trig_ratio * 0.17, extended_trig_ratio * 0.5, extended_trig_ratio]  # 去掉起始点0.0
        
        # 调整多项式部分时间分布，让后半段更均匀
        # 原来：0.78, 0.82, 0.93 (后半段太密集，间隔：0.04, 0.11)
        # 调整后：0.78, 0.85, 0.92 (更均匀分布，间隔：0.07, 0.07)
        polynomial_times = [extended_trig_ratio + (1-extended_trig_ratio) * 0.32, 
                           extended_trig_ratio + (1-extended_trig_ratio) * 0.64]  # 删除最后一个时间点1.0
        full_times = trig_times + polynomial_times
        
        print(f"时间序列：{[f'{t:.2f}' for t in full_times]}")
        print(f"时间分布分析：")
        print(f"  三角函数部分：{[f'{t:.2f}' for t in trig_times]} (均匀分布)")
        print(f"  多项式部分：{[f'{t:.2f}' for t in polynomial_times]} (均匀分布，间隔0.07)")
        print(f"  总时间分布：{[f'{t:.2f}' for t in full_times]} (后半段更均匀)")
        print(f"控制点数量：{len(full_trajectory)} (3个三角函数点 + 2个多项式点，删除三角函数起始点和多项式终点)")
        
        # 5. 生成轨迹消息（确保平滑性）
        for i, point in enumerate(full_trajectory):
            step_fp = footPose()
            x, y, z = point[0], point[1], point[2]
            
            # Yaw角度使用平滑插值
            progress = full_times[i]
            yaw = prev_foot_pose[3] + (next_foot_pose[3] - prev_foot_pose[3]) * progress
            
            step_fp.footPose = [x, y, z, yaw]
            additionalFootPoseTrajectory.data.append(step_fp)
        
        # 6. 验证平滑性
        if len(full_trajectory) >= 2:
            # 检查相邻点之间的距离，确保平滑
            for i in range(len(full_trajectory) - 1):
                p1 = np.array(full_trajectory[i])
                p2 = np.array(full_trajectory[i + 1])
                distance = np.linalg.norm(p2 - p1)
                if distance > 0.2:  # 如果相邻点距离过大，发出警告
                    print(f"警告：点{i+1}和点{i+2}之间距离较大 ({distance:.3f}m)")
        
        # 如果需要绘图验证
        if plot:
            self._plot_trigonometric_trajectory(trig_control_points, prev_foot_pose, next_foot_pose, max_height)
        
        print(f"轨迹生成完成：{len(full_trajectory)}个控制点")
        print(f"三角函数占比：{trig_ratio:.2f}")
        print(f"XY方向：摆线插值，Z方向：三角函数+多项式")
        print(f"三角函数在最高点零加速度，删除三角函数起始点")
        print(f"swing_height逻辑参考三次样条，确保高度合理")
        print(f"符合三次样条输入输出格式，确保平滑性")
        return additionalFootPoseTrajectory
    
    def _trigonometric_quintic_interpolation_downstairs(self, prev_foot_pose, next_foot_pose, swing_height, 
                                                      num_points, is_first_step):
        """下楼梯专用：与上楼梯完全镜像对称，使用相同的控制点结构和三角函数范围"""
        additionalFootPoseTrajectory = footPoses()
        
        # 计算移动距离
        x_distance = next_foot_pose[0] - prev_foot_pose[0]
        y_distance = next_foot_pose[1] - prev_foot_pose[1]
        z_distance = next_foot_pose[2] - prev_foot_pose[2]
        
        # 下楼梯参数设置（与上楼梯完全一致）
        if is_first_step:
            # 第一步：更保守的参数
            trig_ratio = 0.6  # 三角函数部分占比
            max_height_ratio = 1.0  # 最高点相对于总高度的比例
        else:
            # 后续步骤：优化参数
            trig_ratio = 0.6  # 三角函数部分占比
            max_height_ratio = 0.9  # 最高点相对于总高度的比例
        
        # 计算基准高度（取两个落点中较高的点）
        base_height = max(prev_foot_pose[2], next_foot_pose[2])
        min_height = min(prev_foot_pose[2], next_foot_pose[2])
        
        # 下楼梯最高点高度：从当前台阶高度+swing_height，然后减去一级step_height
        max_height = prev_foot_pose[2] + swing_height - 0.08  # 当前台阶高度 + swing_height - step_height
        
        print(f"下楼梯三角函数参数：")
        print(f"  起点高度 = {prev_foot_pose[2]:.3f}, 终点高度 = {next_foot_pose[2]:.3f}")
        print(f"  最高点高度 = {max_height:.3f} (当前台阶高度 + swing_height - step_height)")
        print(f"  三角函数占比 = {trig_ratio:.2f}")
        print(f"  XY方向插值 = 摆线插值")
        print(f"  Z方向插值 = 与上楼梯完全镜像对称")
        
        # 1. 生成三角函数轨迹的4个控制点（与上楼梯完全相同的结构）
        trig_control_points = []
        
        # 使用三角函数生成控制点（确保在最高点零加速度）
        trig_progress = [0.0, 0.33, 0.67, 1.0]  # 三角函数内部进度（与上楼梯相同）
        
        for i, progress in enumerate(trig_progress):
            # 计算平滑进度（使用三次多项式确保在t=1时导数为0）
            t = progress
            smooth_progress = 3 * t**2 - 2 * t**3  # 三次多项式，在t=1时导数为0
            
            # 三角函数方程（Z方向）- 下楼梯：从起点上升到最高点（与上楼梯相同）
            # z = start_z + (max_height - start_z) * sin(π/2 * smooth_progress)
            start_z = prev_foot_pose[2]
            z = start_z + (max_height - start_z) * np.sin(np.pi/2 * smooth_progress)
            
            # XY方向使用摆线插值（与上楼梯相同）
            # 摆线参数：t从0到1
            t_cycloid = progress * trig_ratio  # 归一化到三角函数部分的时间
            
            # 摆线方程：x = t - sin(t), y = 1 - cos(t)
            # 映射到实际坐标
            cycloid_x = t_cycloid - np.sin(2 * np.pi * t_cycloid) / (2 * np.pi)
            cycloid_y = (1 - np.cos(2 * np.pi * t_cycloid)) / 2
            
            # 映射到实际XY坐标
            x = prev_foot_pose[0] + x_distance * cycloid_x
            y = prev_foot_pose[1] + y_distance * cycloid_y
            
            trig_control_points.append([x, y, z])
            
            # 在最高点验证零加速度
            if abs(smooth_progress - 1.0) < 0.01:  # 接近最高点
                print(f"  最高点验证：progress={smooth_progress:.3f}, z={z:.3f}, 加速度为零")
        
        print(f"下楼梯三角函数控制点（4个点，与上楼梯相同结构）：")
        for i, point in enumerate(trig_control_points):
            point_type = "起始点" if i == 0 else "中间点" if i == 1 else "中间点" if i == 2 else "终点"
            print(f"  点{i+1} ({point_type}): ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")
        
        # 2. 生成多项式轨迹的控制点（与上楼梯完全相同的结构）
        polynomial_control_points = []
        
        # 控制点1：多项式起点（后移，避免与三角函数末端重合）
        t_poly_start = trig_ratio + (1 - trig_ratio) * 0.32  # 三角函数占比后32%位置
        
        # XY方向使用摆线规划
        cycloid_x_poly_start = t_poly_start - np.sin(2 * np.pi * t_poly_start) / (2 * np.pi)
        cycloid_y_poly_start = (1 - np.cos(2 * np.pi * t_poly_start)) / 2
        
        x_poly_start = prev_foot_pose[0] + x_distance * cycloid_x_poly_start
        y_poly_start = prev_foot_pose[1] + y_distance * cycloid_y_poly_start
        
        # Z方向平滑下降（从三角函数终点高度开始）
        z_trig_end = trig_control_points[-1][2]  # 三角函数终点高度
        z_poly_start = z_trig_end + (next_foot_pose[2] - z_trig_end) * 0.15  # 下降15%
        polynomial_control_points.append([x_poly_start, y_poly_start, z_poly_start])
        
        # 控制点2：中间点（使用摆线插值）
        t_mid = trig_ratio + (1 - trig_ratio) * 0.64  # 多项式部分64%位置
        
        # 摆线插值
        cycloid_x_mid = t_mid - np.sin(2 * np.pi * t_mid) / (2 * np.pi)
        cycloid_y_mid = (1 - np.cos(2 * np.pi * t_mid)) / 2
        
        x_mid = prev_foot_pose[0] + x_distance * cycloid_x_mid
        y_mid = prev_foot_pose[1] + y_distance * cycloid_y_mid
        z_mid = next_foot_pose[2] + (z_poly_start - next_foot_pose[2]) * 0.5  # 平滑下降
        polynomial_control_points.append([x_mid, y_mid, z_mid])
        
        # 控制点3：目标位置
        x_end = next_foot_pose[0]
        y_end = next_foot_pose[1]
        z_end = next_foot_pose[2]
        polynomial_control_points.append([x_end, y_end, z_end])
        
        print(f"下楼梯多项式控制点（3个点，与上楼梯相同结构）：")
        for i, point in enumerate(polynomial_control_points):
            point_type = "多项式起点" if i == 0 else "中间点" if i == 1 else "终点"
            print(f"  点{i+1} ({point_type}): ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")
        
        # 3. 生成完整轨迹（7个控制点：4个三角函数点 + 3个多项式点，与上楼梯相同）
        full_trajectory = trig_control_points + polynomial_control_points
        
        # 删除第一个点（三角函数起始点）和最后一个点（多项式终点）
        full_trajectory = full_trajectory[1:-1]
        
        # 4. 生成时间序列（与上楼梯完全相同的时间分布）
        # 时间分配：三角函数部分占trig_ratio，多项式部分占(1-trig_ratio)
        # 延长三角函数部分时间，让抬腿更慢
        extended_trig_ratio = trig_ratio * 1.3  # 延长30%
        trig_times = [extended_trig_ratio * 0.17, extended_trig_ratio * 0.5, extended_trig_ratio]  # 去掉起始点0.0
        
        # 调整多项式部分时间分布，让后半段更均匀
        polynomial_times = [extended_trig_ratio + (1-extended_trig_ratio) * 0.32, 
                           extended_trig_ratio + (1-extended_trig_ratio) * 0.64]  # 删除最后一个时间点1.0
        full_times = trig_times + polynomial_times
        
        print(f"下楼梯时间序列（与上楼梯相同）：{[f'{t:.2f}' for t in full_times]}")
        print(f"时间分布分析：")
        print(f"  三角函数部分：{[f'{t:.2f}' for t in trig_times]} (均匀分布)")
        print(f"  多项式部分：{[f'{t:.2f}' for t in polynomial_times]} (均匀分布，间隔0.07)")
        print(f"  总时间分布：{[f'{t:.2f}' for t in full_times]} (与上楼梯完全一致)")
        print(f"控制点数量：{len(full_trajectory)} (3个三角函数点 + 2个多项式点，与上楼梯相同)")
        
        # 5. 生成轨迹消息（与上楼梯完全相同的执行逻辑）
        for i, point in enumerate(full_trajectory):
            step_fp = footPose()
            x, y, z = point[0], point[1], point[2]
            
            # Yaw角度使用平滑插值
            progress = full_times[i]
            yaw = prev_foot_pose[3] + (next_foot_pose[3] - prev_foot_pose[3]) * progress
            
            step_fp.footPose = [x, y, z, yaw]
            additionalFootPoseTrajectory.data.append(step_fp)
        
        # 6. 验证平滑性
        if len(full_trajectory) >= 2:
            # 检查相邻点之间的距离，确保平滑
            for i in range(len(full_trajectory) - 1):
                p1 = np.array(full_trajectory[i])
                p2 = np.array(full_trajectory[i + 1])
                distance = np.linalg.norm(p2 - p1)
                if distance > 0.2:  # 如果相邻点距离过大，发出警告
                    print(f"警告：点{i+1}和点{i+2}之间距离较大 ({distance:.3f}m)")
        
        print(f"下楼梯轨迹生成完成：{len(full_trajectory)}个控制点")
        print(f"与上楼梯完全镜像对称：相同的控制点结构、三角函数范围、时间分布")
        print(f"三角函数占比：{trig_ratio:.2f}")
        print(f"XY方向：摆线插值，Z方向：三角函数+多项式")
        print(f"三角函数在最高点零加速度，删除三角函数起始点")
        return additionalFootPoseTrajectory
    
    def _spline_interpolation(self, prev_foot_pose, next_foot_pose, swing_height, 
                             num_points, is_first_step, down_stairs):
        """三次样条插值方法（参考stairClimbPlanner-roban.py）"""
        additionalFootPoseTrajectory = footPoses()
        
        # 创建时间序列
        t = np.linspace(0, 1, num_points)
        
        # 计算x和y方向的移动距离
        x_distance = next_foot_pose[0] - prev_foot_pose[0]
        y_distance = next_foot_pose[1] - prev_foot_pose[1]
        
        # 计算基准高度（取两个落点中较高的点）
        base_height = max(prev_foot_pose[2], next_foot_pose[2])
        min_height = min(prev_foot_pose[2], next_foot_pose[2])
        
        # 创建控制点
        # 时间点：0, 0.2, 0.5, 1.0
        # 0.2时刻：x和y移动10%，z达到最高点
        # 0.5时刻：x和y移动50%，z保持最高点
        
        control_points = None
        if not down_stairs:
            control_points = {
                't': [0, 0.2, 0.6, 1.0],
                'x': [
                    prev_foot_pose[0],                    # 起点
                    prev_foot_pose[0] + x_distance * 0.05, # 前10%
                    prev_foot_pose[0] + x_distance * 0.6, # 前50%
                    next_foot_pose[0]                     # 终点
                ],
                'y': [
                    prev_foot_pose[1],                    # 起点
                    prev_foot_pose[1] + y_distance * 0.05, # 前10%
                    prev_foot_pose[1] + y_distance * 0.6, # 前50%
                    next_foot_pose[1]                     # 终点
                ],
                'z': [
                    prev_foot_pose[2],                    # 起点
                    (base_height + swing_height*0.6) if is_first_step else (prev_foot_pose[2] + (base_height-min_height) * 0.5),           # 最高点（基于较高的落点）
                    base_height + swing_height,           # 保持最高点
                    next_foot_pose[2]                     # 终点
                ]
            }
        else: # 下楼梯
            if not is_first_step: # 非第一步或者最后一步
                control_points = {
                    't': [0, 0.3, 0.5, 0.6, 1.0],
                    'x': [
                        prev_foot_pose[0],                    # 起点
                        prev_foot_pose[0] + x_distance * 0.4, # 前50%
                        prev_foot_pose[0] + x_distance * 0.7, # 前50%
                        prev_foot_pose[0] + x_distance * 0.9 , # 前10%
                        next_foot_pose[0]                     # 终点
                    ],
                    'y': [
                        prev_foot_pose[1],                    # 起点
                        prev_foot_pose[1] + y_distance * 0.4, # 前50%
                        prev_foot_pose[1] + y_distance * 0.7, # 前50%
                        prev_foot_pose[1] + y_distance * 0.9, # 前10%
                        next_foot_pose[1]                     # 终点
                    ],
                    'z': [
                        prev_foot_pose[2],                    # 起点
                        base_height + swing_height,           # 保持最高点
                        (next_foot_pose[2] + (base_height-min_height) * 0.9),
                        (next_foot_pose[2] + (base_height-min_height) * 0.7),           # 最高点（基于较高的落点）
                        next_foot_pose[2]                     # 终点
                    ]
                }
            else:
                control_points = {
                    't': [0, 0.5, 0.6, 1.0],
                    'x': [
                        prev_foot_pose[0],                    # 起点
                        prev_foot_pose[0] + x_distance * 0.60, # 前50%
                        prev_foot_pose[0] + x_distance * 0.95 , # 前10%
                        next_foot_pose[0]                     # 终点
                    ],
                    'y': [
                        prev_foot_pose[1],                    # 起点
                        prev_foot_pose[1] + y_distance * 0.6, # 前50%
                        prev_foot_pose[1] + y_distance * 0.95, # 前10%
                        next_foot_pose[1]                     # 终点
                    ],
                    'z': [
                        prev_foot_pose[2],                    # 起点
                        base_height + swing_height,           # 保持最高点
                        base_height + swing_height * 0.2,           # 最高点（基于较高的落点）
                        next_foot_pose[2]                     # 终点
                    ]
                }
        
        # 为x、y和z创建形状保持的三次样条插值
        x_spline = PchipInterpolator(control_points['t'], control_points['x'])
        y_spline = PchipInterpolator(control_points['t'], control_points['y'])
        z_spline = PchipInterpolator(control_points['t'], control_points['z'])
        
        # yaw角度使用形状保持的三次样条
        yaw_spline = PchipInterpolator([0, 1], [prev_foot_pose[3], next_foot_pose[3]])
        
        # 生成轨迹点
        trajectory_points = []
        for i in range(num_points):
            # 跳过第一个点（索引为0）和最后一个点（索引为num_points-1）
            if i == 0 or i == num_points - 1:
                continue
                
            step_fp = footPose()
            x = float(x_spline(t[i]))
            y = float(y_spline(t[i]))
            z = float(z_spline(t[i]))
            yaw = float(yaw_spline(t[i]))
            
            step_fp.footPose = [x, y, z, yaw]
            additionalFootPoseTrajectory.data.append(step_fp)
            trajectory_points.append([x, y, z])

        return additionalFootPoseTrajectory 