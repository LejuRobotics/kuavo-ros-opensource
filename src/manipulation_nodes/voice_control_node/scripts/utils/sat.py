import numpy as np

class RotatingRectangle:
    def __init__(self, center, width, height, angle):
        self.center = center
        self.width = width
        self.height = height
        self.angle = angle

    def set_rotation(self, angle):
        self.angle = angle
    
    def rotate_point(self, point):
        """旋转点"""
        px, py = point
        cos_theta = np.cos(self.angle)
        sin_theta = np.sin(self.angle)
        return (cos_theta * px - sin_theta * py, sin_theta * px + cos_theta * py)
    
    def get_vertices(self):
        """获取旋转长方形的顶点"""
        cx, cy = self.center
        half_w, half_h = self.width / 2, self.height / 2
        vertices = [
            (half_w, half_h),
            (-half_w, half_h),
            (-half_w, -half_h),
            (half_w, -half_h)
        ]
        return [(cx + x, cy + y) for x, y in (self.rotate_point(v) for v in vertices)]

    @staticmethod
    def project(vertices, axis):
        """将顶点投影到轴上"""
        projections = [np.dot(v, axis) for v in vertices]
        return min(projections), max(projections)

    @staticmethod
    def is_separating_axis(vertices1, vertices2, axis):
        """检查是否为分离轴"""
        min1, max1 = RotatingRectangle.project(vertices1, axis)
        min2, max2 = RotatingRectangle.project(vertices2, axis)
        return max1 < min2 or max2 < min1

    def is_collision(self, other):
        """检测两个旋转长方形是否发生碰撞"""
        vertices1 = self.get_vertices()
        vertices2 = other.get_vertices()
        
        # 获取所有可能的分离轴
        axes = []
        for i in range(4):
            edge = (vertices1[i][0] - vertices1[i-1][0], vertices1[i][1] - vertices1[i-1][1])
            axis = (-edge[1], edge[0])
            axes.append(axis)
        for i in range(4):
            edge = (vertices2[i][0] - vertices2[i-1][0], vertices2[i][1] - vertices2[i-1][1])
            axis = (-edge[1], edge[0])
            axes.append(axis)
        
        # 检查所有轴上的投影是否重叠
        for axis in axes:
            if self.is_separating_axis(vertices1, vertices2, axis):
                return False
        return True

    def show(self, color):
        """绘制旋转长方形"""
        vertices = self.get_vertices()
        vertices.append(vertices[0])  # 闭合多边形
        xs, ys = zip(*vertices)
        plt.fill(xs, ys, color=color, alpha=0.5)


# 示例
if __name__ == "__main__":
    rect1 = RotatingRectangle(center=(0, 0.1), width=0.2, height=0.1, angle=0)
    rect2 = RotatingRectangle(center=(0, -0.1), width=0.2, height=0.1, angle=0)

    collision = rect1.is_collision(rect2)
    print("发生碰撞:", collision)

    rect1.set_rotation(-np.pi / 4)
    rect2.set_rotation(np.pi / 4)
    collision = rect1.is_collision(rect2)
    print("发生碰撞:", collision)

    # visualize
    import matplotlib.pyplot as plt
    # 绘制长方形
    plt.figure()
    rect1.show(color='blue')
    rect2.show(color='red')

    # 设置显示区域
    plt.xlim(-0.3, 0.3)
    plt.ylim(-0.3, 0.3)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid()
    plt.title(f'Rotated Rectangle Collision Detection: {collision}')
    plt.show()