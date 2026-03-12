import numpy as np
from PIL import Image
import random
from scipy.ndimage import gaussian_filter

# 设置图像的尺寸
width = 512  
height = 512  

# 生成随机高度场
def generate_road_heightfield(width, height): 
    heightfield = np.zeros((height, width), dtype=np.float32)
    
    # 使用Perlin噪声风格的方法生成更自然的地形
    # 创建多个频率的噪声层并叠加
    # scales = [1, 2, 4, 8, 16]  # 不同尺度的噪声
    # weights = [1.0, 0.5, 0.25, 0.125, 0.0625]  # 对应权重（低频主导）
    scales = [16]  # 不同尺度的噪声
    weights = [1.0]  # 对应权重（低频主导）
    
    for scale, weight in zip(scales, weights):
        # 为每个尺度生成噪声
        small_width = max(1, width // scale)
        small_height = max(1, height // scale)
        
        # 生成小尺寸的随机噪声（进一步降低基础噪声幅度）
        small_noise = np.random.rand(small_height, small_width) * 80 * weight  # 进一步降低基础噪声幅度到20
        
        # 插值放大到原始尺寸
        if scale > 1:
            # 线性插值放大
            y_coords = np.linspace(0, small_height-1, height)
            x_coords = np.linspace(0, small_width-1, width)
            y_grid, x_grid = np.meshgrid(y_coords, x_coords, indexing='ij')
            
            # 简单的双线性插值
            y_floor = np.floor(y_grid).astype(int)
            x_floor = np.floor(x_grid).astype(int)
            y_ceil = np.minimum(y_floor + 1, small_height - 1)
            x_ceil = np.minimum(x_floor + 1, small_width - 1)
            
            # 权重计算
            wy = y_grid - y_floor
            wx = x_grid - x_floor
            
            # 双线性插值
            interpolated = (
                small_noise[y_floor, x_floor] * (1 - wy) * (1 - wx) +
                small_noise[y_ceil, x_floor] * wy * (1 - wx) +
                small_noise[y_floor, x_ceil] * (1 - wy) * wx +
                small_noise[y_ceil, x_ceil] * wy * wx
            )
            heightfield += interpolated
        else:
            # 最大尺度直接使用
            heightfield += np.repeat(np.repeat(small_noise, scale, axis=0), scale, axis=1)[:height, :width]
    
    # 添加一些温和的大范围起伏（进一步降低密度和幅度）
    for i in range(0, height, 40):  # 进一步增大间隔到40，进一步降低密度
        for j in range(0, width, 40):  
            large_wave = random.randint(0, 15*6)  # 进一步降低大范围起伏幅度到0-15
            y_end = min(i+40, height)
            x_end = min(j+40, width)
            heightfield[i:y_end, j:x_end] += large_wave * 0.15 * 4  # 进一步降低影响系数到0.15

    # 设置图像四边1个像素点为平地（高度值为0）
    border_width = 1
    heightfield[:border_width, :] = 0  # 上边
    heightfield[-border_width:, :] = 0  # 下边
    heightfield[:, :border_width] = 0  # 左边
    heightfield[:, -border_width:] = 0  # 右边

    # 对四边进行额外的平滑处理，确保从零到当前高度的平滑过渡
    transition_width = 65  # 过渡区域宽度
    
    # 上边过渡处理
    for i in range(border_width, min(border_width + transition_width, height)):
        # 计算过渡因子（从0到1）
        factor = (i - border_width) / transition_width
        # 对每一列进行线性插值
        for j in range(width):
            heightfield[i, j] = heightfield[i, j] * factor
    
    # 下边过渡处理
    for i in range(max(height - border_width - transition_width, 0), height - border_width):
        # 计算过渡因子（从1到0）
        factor = (height - border_width - i - 1) / transition_width
        # 对每一列进行线性插值
        for j in range(width):
            heightfield[i, j] = heightfield[i, j] * factor
    
    # 左边过渡处理
    for j in range(border_width, min(border_width + transition_width, width)):
        # 计算过渡因子（从0到1）
        factor = (j - border_width) / transition_width
        # 对每一行进行线性插值
        for i in range(height):
            heightfield[i, j] = heightfield[i, j] * factor
    
    # 右边过渡处理
    for j in range(max(width - border_width - transition_width, 0), width - border_width):
        # 计算过渡因子（从1到0）
        factor = (width - border_width - j - 1) / transition_width
        # 对每一行进行线性插值
        for i in range(height):
            heightfield[i, j] = heightfield[i, j] * factor

    # 多层次高斯平滑处理，让变化更加缓和
    # # 第一次粗略平滑
    heightfield = gaussian_filter(heightfield, sigma=4)  # 增加sigma值（原来是4）
    # # 第二次精细平滑
    # heightfield = gaussian_filter(heightfield, sigma=2)
    
    # 归一化图像到[0, 255]范围
    heightfield = np.clip(heightfield, 0, 255).astype(np.uint8)
    
    return heightfield  

heightfield = generate_road_heightfield(width, height)

image = Image.fromarray(heightfield)  
image.save("road_heightfield_smooth.png")  

# 显示图像信息
print(f"生成的高度场图像已保存为 road_heightfield_smooth.png")
print(f"图像尺寸: {width} x {height}")
print(f"高度值范围: {heightfield.min()} - {heightfield.max()}")

# 显示图像
# image.show()