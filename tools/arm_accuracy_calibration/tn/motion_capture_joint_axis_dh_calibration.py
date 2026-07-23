import numpy as np
import pandas as pd  # 新增：用于读取CSV
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ===================== 核心配置：只改这里！DH参数/基座/初始轴都在这 =====================
DH_CONFIG = {
    # 基座坐标系参数（J0坐标系）
    "base_origin": np.array([0, 0, 0]),  # J0坐标系原点
    "base_axes": {                       # J0坐标系x/y/z轴方向
        "x": np.array([1, 0, 0]),
        "y": np.array([0, 1, 0]),
        "z": np.array([0, 0, 1])
    },
    # 关节初始轴猜测（按关节号配置）
    "joint_init_axis": {
        "J1": np.array([0, 1, 0]),  # J1真实轴初始猜测（比如Y轴）
        "J2": np.array([1, 0, 0]),  # 可扩展J2
        "J3": np.array([0, 0, 1])   # 可扩展J3
    },
    # 其他标定参数
    "joint_name": "J1",             # 当前标定关节（J1/J2/J3）
    "csv_path": lambda name: f"{name}_marker_pos.csv",  # 一行lambda函数，动态生成路径
}

# ===================== 核心函数：拟合3D圆（无硬编码轴向） =====================
def fit_3d_circle(points, dh_config):
    """
    输入：
        points: 3D点云（N,3）
        dh_config: DH配置字典
    输出：center, radius, axis, dist_from_base_z
    """
    # 从配置读取参数
    base_z = dh_config["base_axes"]["z"]
    base_origin = dh_config["base_origin"]
    init_axis = dh_config["joint_init_axis"][dh_config["joint_name"]]

    # 步骤1：初始化拟合目标
    def objective(params):
        cx, cy, cz, nx, ny, nz = params
        axis = np.array([nx, ny, nz])
        axis = axis / np.linalg.norm(axis)
        center = np.array([cx, cy, cz])
        
        # 用中位数计算参考半径（更稳定）
        d_axis_list = [np.linalg.norm(np.cross(p-center, axis)) for p in points]
        ref_radius = np.median(d_axis_list)
        distances = [(d - ref_radius)**2 for d in d_axis_list]
        return np.sum(distances)
    
    # 步骤2：初始猜测
    init_center = np.mean(points, axis=0)
    init_guess = np.concatenate([init_center, init_axis])
    
    # 步骤3：优化拟合
    bounds = [
        (None, None), (None, None), (None, None),  # center无边界
        (-1, 1), (-1, 1), (-1, 1)                  # axis分量限制
    ]
    result = minimize(objective, init_guess, method='L-BFGS-B', bounds=bounds)
    cx, cy, cz, nx, ny, nz = result.x
    center = np.array([cx, cy, cz])
    axis = np.array([nx, ny, nz])
    axis = axis / np.linalg.norm(axis)
    radius = np.median([np.linalg.norm(np.cross(p-center, axis)) for p in points])
    
    # 步骤4：计算基座z轴到旋转轴的公垂线长度
    cross = np.cross(base_z, axis)
    if np.linalg.norm(cross) < 1e-6:
        dist_from_base_z = 0.0
    else:
        dist_from_base_z = np.abs(np.dot(center - base_origin, cross)) / np.linalg.norm(cross)
    
    return center, radius, axis, dist_from_base_z

# ===================== 核心函数：计算DH参数（配置驱动） =====================
def calculate_dh_params(center, axis, dh_config, theta_cmd_list=None, points=None):
    """
    输入：
        center/radius/axis: 拟合结果
        dh_config: DH配置字典
        theta_cmd_list/points: 指令角和对应点（可选）
    输出：dh_params字典
    """
    base_z = dh_config["base_axes"]["z"]
    base_origin = dh_config["base_origin"]

    # 1. 计算α（基座z轴与旋转轴的夹角）
    alpha = np.arccos(np.clip(np.dot(base_z, axis), -1.0, 1.0))
    alpha_deg = np.degrees(alpha)
    
    # 2. 计算a（公垂线长度）
    cross = np.cross(base_z, axis)
    if np.linalg.norm(cross) < 1e-6:
        a = 0.0
    else:
        a = np.abs(np.dot(center - base_origin, cross)) / np.linalg.norm(cross)
    
    # 3. 计算d（沿旋转轴的偏置）
    v = center - base_origin
    d = np.dot(v, axis)
    
    # 4. 计算θ零位偏移（无指令角则为0）
    theta_offset = 0.0
    theta_offset_deg = 0.0
    if theta_cmd_list is not None and points is not None:
        zero_idx = np.argmin(np.abs(theta_cmd_list))
        zero_point = points[zero_idx]
        v_zero = zero_point - center
        proj = np.dot(v_zero, axis) * axis
        v_perp = v_zero - proj
        
        ref_dir = dh_config["base_axes"]["x"]
        ref_dir = ref_dir - np.dot(ref_dir, axis) * axis
        ref_dir = ref_dir / np.linalg.norm(ref_dir)
        
        theta_actual = np.arccos(np.clip(np.dot(v_perp / np.linalg.norm(v_perp), ref_dir), -1.0, 1.0))
        theta_offset = theta_actual - np.radians(theta_cmd_list[zero_idx])
        theta_offset_deg = np.degrees(theta_offset)
    
    dh_params = {
        'α(rad)': alpha, 'α(deg)': alpha_deg,
        'a(m)': a,
        'd(m)': d,
        'θ_offset(rad)': theta_offset, 'θ_offset(deg)': theta_offset_deg
    }
    return dh_params

# ===================== 可视化函数：通用化 =====================
def visualize_3d_circle(points, center, radius, axis, dh_config):
    """通用3D可视化，绘制真实采集点和拟合结果"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制真实动捕点（CSV读取的点）
    ax.scatter(points[:,0], points[:,1], points[:,2], c='blue', label='动捕采集点', s=50)
    # 绘制拟合的圆心
    ax.scatter(center[0], center[1], center[2], c='red', marker='*', s=200, label='圆心（旋转轴上点）')
    # 绘制旋转轴
    axis_len = radius * 2
    ax.quiver(center[0], center[1], center[2],
              axis[0]*axis_len, axis[1]*axis_len, axis[2]*axis_len,
              color='green', linewidth=2, label='关节旋转轴')
    
    # 生成拟合圆
    theta = np.linspace(0, 2*np.pi, 100)
    if np.linalg.norm(np.cross(axis, np.array([1,0,0]))) > 1e-6:
        u = np.cross(axis, np.array([1,0,0]))
    else:
        u = np.cross(axis, np.array([0,1,0]))
    u = u - np.dot(u, axis)*axis
    u = u / np.linalg.norm(u)
    v = np.cross(axis, u)
    v = v / np.linalg.norm(v)
    circle_points = center + radius * (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v))
    ax.plot(circle_points[:,0], circle_points[:,1], circle_points[:,2], c='orange', linewidth=2, label='拟合圆')
    
    # 标注坐标系
    ax.set_xlabel(f'X (m) (base_x: {dh_config["base_axes"]["x"]})')
    ax.set_ylabel(f'Y (m) (base_y: {dh_config["base_axes"]["y"]})')
    ax.set_zlabel(f'Z (m) (base_z: {dh_config["base_axes"]["z"]})')
    ax.legend()
    ax.set_title(f'{dh_config["joint_name"]} 3D圆拟合结果（真实动捕数据）')
    plt.show()

# ===================== 主程序：读取CSV + 拟合 + 计算DH + 可视化 =====================
if __name__ == "__main__":
    # ---------------------- 1. 读取CSV文件（核心修改：替换模拟数据） ----------------------
    try:
        # 读取CSV，只保留pos_x/pos_y/pos_z列
        csv_file = DH_CONFIG["csv_path"](DH_CONFIG["joint_name"])
        df = pd.read_csv(csv_file, usecols=["timestamp", "pos_x", "pos_y", "pos_z"])
        # 数据清洗：删除空值、重置索引
        df = df.dropna(subset=["pos_x", "pos_y", "pos_z"]).reset_index(drop=True)
        # 提取3D坐标点（N,3）
        points = df[["pos_x", "pos_y", "pos_z"]].values
        
        # 检查数据有效性
        if len(points) < 5:  # 至少需要5个点拟合圆
            raise ValueError(f"CSV文件有效点数不足（仅{len(points)}个），无法拟合3D圆！")
        print(f"成功读取CSV：{len(points)}个有效动捕点")
    
    except FileNotFoundError:
        print(f"错误：未找到CSV文件！请检查路径：{csv_file}")
        exit(1)
    except KeyError as e:
        print(f"错误：CSV文件缺少列 {e}！请确保格式为 timestamp,pos_x,pos_y,pos_z")
        exit(1)
    except Exception as e:
        print(f"读取CSV出错：{str(e)}")
        exit(1)
    
    # ---------------------- 2. 拟合3D圆和旋转轴 ----------------------
    center, radius, axis, a = fit_3d_circle(points, DH_CONFIG)
    print(f"\n=== {DH_CONFIG['joint_name']} 拟合结果 ===")
    print(f"圆心坐标: {center.round(6)} (m)")
    print(f"旋转轴方向: {axis.round(6)}")
    print(f"圆半径: {radius.round(6)} (m)")
    print(f"基座z轴到旋转轴的公垂线长度(a): {a.round(6)} (m)")
    
    # ---------------------- 3. 计算DH参数 ----------------------
    # 若有指令角数据，可补充theta_cmd_list，否则theta_offset为0
    dh_params = calculate_dh_params(center, axis, DH_CONFIG)
    print(f"\n=== {DH_CONFIG['joint_name']} DH参数 ===")
    for key, value in dh_params.items():
        print(f"{key}: {value.round(6)}")
    
    # ---------------------- 4. 可视化结果 ----------------------
    visualize_3d_circle(points, center, radius, axis, DH_CONFIG)
