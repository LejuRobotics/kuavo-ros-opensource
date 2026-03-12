import numpy as np
from PIL import Image
import random
from scipy.ndimage import gaussian_filter

# 设置图像的尺寸
width = 512  
height = 512  

# 生成随机高度场
def generate_road_heightfield(width, height): 
    heightfield = np.zeros((height, width), dtype=np.uint8)
    
    # 随机噪声
    for i in range(height):  
        for j in range(width):  
            noise = random.randint(0, 30)  # 可以调整高度变化的范围
            heightfield[i, j] = noise  

    # 添加一些大范围的“波动”，模拟较大的起伏
    for i in range(0, height, 10):  
        for j in range(0, width, 10):  
            large_wave = random.randint(0, 100)  
            heightfield[i:i+10, j:j+10] += large_wave

    heightfield[width//2-15:width//2+15, height//2-15:height//2+15] = 0  # 设置原点附近的区域为平地

    # 对图像进行平滑处理，以减少噪点
    heightfield = gaussian_filter(heightfield, sigma=2)  # 调整sigma值控制平滑程度
    
    # 归一化图像到[0, 255]范围
    heightfield = np.clip(heightfield, 0, 255).astype(np.uint8)
    
    return heightfield  

heightfield = generate_road_heightfield(width, height)

image = Image.fromarray(heightfield)  
image.save("road_heightfield.png")  

# 显示图像
# image.show()
