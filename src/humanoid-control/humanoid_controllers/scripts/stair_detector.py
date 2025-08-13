#!/usr/bin/env python3

import rospy
import numpy as np
from grid_map_msgs.msg import GridMap
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import cv2
from scipy.ndimage import gaussian_filter
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


class StairDetector:
    def __init__(self):
        # 订阅高程图话题
        self.elevation_sub = rospy.Subscriber(
            "/elevation_mapping/elevation_map_raw",
            GridMap,
            self.elevation_callback
        )
        
        # 发布可视化标记
        self.marker_pub = rospy.Publisher(
            "/stair_planes",
            MarkerArray,
            queue_size=10
        )
        
        # 发布处理步骤的可视化图像
        self.raw_map_pub = rospy.Publisher("/stair_detection/raw_map", Image, queue_size=1)
        self.filtered_map_pub = rospy.Publisher("/stair_detection/filtered_map", Image, queue_size=1)
        self.gradient_map_pub = rospy.Publisher("/stair_detection/gradient_map", Image, queue_size=1)
        self.plane_mask_pub = rospy.Publisher("/stair_detection/plane_mask", Image, queue_size=1)
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 存储高程图数据
        self.elevation_map = None
        self.map_resolution = None
        self.map_origin = None
        
        # 楼梯检测参数
        self.min_plane_size = 0.1  # 减小最小平面尺寸（米）
        self.max_plane_size = 2.0  # 最大平面尺寸（米）
        self.min_plane_area = 0.1  # 最小平面面积（平方米）
        self.max_plane_area = 1.0   # 最大平面面积（平方米）
        self.max_plane_height_diff = 0.03  # 减小平面内最大高度差
        self.min_stair_height = 0.08  # 最小台阶高度
        self.max_stair_height = 0.20  # 最大台阶高度
        self.min_stair_depth = 0.25  # 最小台阶深度
        self.max_stair_depth = 0.35  # 最大台阶深度
        self.min_ground_height = 0.05  # 添加地面高度阈值（米）
        
        # 噪声过滤参数
        self.gaussian_sigma = 0.1  # 减小高斯滤波参数
        self.height_threshold = 0.05  # 减小高度阈值
        
        # 添加平面检测参数
        self.dbscan_eps = 0.5  # DBSCAN聚类距离阈值（米）
        self.dbscan_min_samples = 3  # DBSCAN最小样本数
        
        # 添加环形缓冲区相关参数
        self.buffer_start_index = None  # 环形缓冲区的起始索引
        
        # 平面合并参数
        self.max_merge_height_diff = 0.03  # 合并平面的最大高度差（米）
        self.min_merge_overlap_ratio = 0.3  # 合并平面的最小重叠比例

        self.stair_centers_pub = rospy.Publisher('/stair_centers', PointCloud2, queue_size=10)
        
    def normalize_image(self, img):
        """将数据归一化到0-255范围用于可视化"""
        if img is None:
            return None
            
        # 添加调试信息
        
        # 移除无效值
        valid_mask = ~np.isnan(img) & ~np.isinf(img)
        if not np.any(valid_mask):
            rospy.logwarn("No valid values in image")
            return None
            
        # 只使用有效值进行归一化
        valid_data = img[valid_mask]
        img_min = np.min(valid_data)
        img_max = np.max(valid_data)
        
        if img_max == img_min:
            rospy.logwarn("All values are the same")
            return np.zeros_like(img, dtype=np.uint8)
            
        # 归一化到0-255
        normalized = np.zeros_like(img, dtype=np.uint8)
        normalized[valid_mask] = ((img[valid_mask] - img_min) / (img_max - img_min) * 255).astype(np.uint8)
        
        # 应用颜色映射
        colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
        return colored
        
    def elevation_callback(self, msg):
        """处理高程图数据
        
        GridMap数据格式：
        - header: 包含时间戳和坐标系信息
        - info: 包含地图信息（分辨率、尺寸、原点位置）
        - layers: 图层名称列表
        - data: 每个图层的数据（一维数组）
        
        高程图特点：
        - 以机器人为中心的高度矩阵
        - map_origin表示高程图中心点在世界坐标系中的位置
        - outer_start_index和inner_start_index与高程图中心点位置相关
        """
        try:
            # 检查必要的层是否存在
            if 'elevation_inpainted' not in msg.layers:
                rospy.logwarn("elevation_inpainted layer not found in grid map")
                return
                
            # 更新地图数据
            elevation_layer_index = msg.layers.index('elevation_inpainted')
            rows = int(msg.info.length_y / msg.info.resolution)
            cols = int(msg.info.length_x / msg.info.resolution)
            
            # 获取原始一维数据
            raw_data = np.array(msg.data[elevation_layer_index].data)
            
            # 打印调试信息
            # print("原始数据形状:", raw_data.shape)
            # print("地图尺寸 - rows:", rows, "cols:", cols)
            # print("地图分辨率:", msg.info.resolution)
            # print("地图原点位置:", msg.info.pose.position)
            # print("环形缓冲区索引 - outer:", msg.outer_start_index, "inner:", msg.inner_start_index)
            
            # 计算一维数组中的起始位置
            start_index = msg.outer_start_index + msg.inner_start_index * rows
            
            # 重排一维数据
            reordered_data = np.concatenate([
                raw_data[start_index:],  # 从起始位置到末尾
                raw_data[:start_index]   # 从开始到起始位置
            ])
            
            # 将重排后的数据重塑为二维矩阵并进行转置
            self.elevation_map = reordered_data.reshape(rows, cols).T
            
            # 保存地图参数
            self.map_resolution = msg.info.resolution
            self.map_origin = msg.info.pose
            
            # 发布原始高程图
            raw_img = self.normalize_image(self.elevation_map)
            if raw_img is not None:
                self.raw_map_pub.publish(self.bridge.cv2_to_imgmsg(raw_img, "bgr8"))
            
            # 暂时注释掉平面检测，专注于高程图显示
            stair_planes = self.detect_stair_planes()
            print("检测到的平面数量:", len(stair_planes))
            self.publish_visualization(stair_planes)
            
        except Exception as e:
            rospy.logerr(f"Error in elevation_callback: {str(e)}")
            rospy.logerr(f"Message layers: {msg.layers}")
            rospy.logerr(f"Message info: {msg.info}")
        
    def filter_noise(self, elevation_map):
        """过滤高程图中的噪声"""
        # 1. 处理NaN值
        valid_mask = ~np.isnan(elevation_map)
        if not np.any(valid_mask):
            return elevation_map
            
        # 使用有效值的均值填充NaN
        mean_height = np.mean(elevation_map[valid_mask])
        filtered_map = elevation_map.copy()
        filtered_map[~valid_mask] = mean_height
        
        # 2. 高斯滤波平滑高度
        smoothed_map = gaussian_filter(filtered_map, sigma=self.gaussian_sigma)
        
        # 3. 移除异常值
        height_diff = np.abs(filtered_map - smoothed_map)
        mask = height_diff > self.height_threshold
        filtered_map[mask] = smoothed_map[mask]
        
        # 发布滤波后的图像
        filtered_img = self.normalize_image(filtered_map)
        if filtered_img is not None:
            self.filtered_map_pub.publish(self.bridge.cv2_to_imgmsg(filtered_img, "bgr8"))
        
        return filtered_map
        
    def filter_overlapping_planes(self, planes):
        """过滤重叠或过于接近的平面
        
        Args:
            planes: 检测到的平面列表
            
        Returns:
            filtered_planes: 过滤后的平面列表
        """
        if not planes:
            return []
            
        # 设置过滤参数
        min_center_distance = 0.05  # 中心点最小距离（米）
        max_overlap_ratio = 0.3    # 最大允许重叠比例
        
        filtered_planes = []
        used_indices = set()
        
        # 对平面按面积从大到小排序
        sorted_indices = sorted(range(len(planes)), 
                             key=lambda k: planes[k]['area'],
                             reverse=True)
        
        for i in sorted_indices:
            if i in used_indices:
                continue
                
            plane_i = planes[i]
            should_keep = True
            
            # 检查与已保留的平面的关系
            for kept_plane in filtered_planes:
                # 1. 检查中心点距离
                dx = plane_i['center'][0] - kept_plane['center'][0]
                dy = plane_i['center'][1] - kept_plane['center'][1]
                dz = abs(plane_i['center'][2] - kept_plane['center'][2])
                center_distance = (dx * dx + dy * dy) ** 0.5
                
                # 如果中心点太近或高度差太小，跳过这个平面
                if center_distance < min_center_distance and dz < self.min_stair_height:
                    should_keep = False
                    break
                    
                # 2. 检查重叠情况
                # 计算两个矩形的重叠面积
                x_overlap = max(0, min(plane_i['center'][0] + plane_i['size'][0]/2,
                                     kept_plane['center'][0] + kept_plane['size'][0]/2) -
                                 max(plane_i['center'][0] - plane_i['size'][0]/2,
                                     kept_plane['center'][0] - kept_plane['size'][0]/2))
                                     
                y_overlap = max(0, min(plane_i['center'][1] + plane_i['size'][1]/2,
                                     kept_plane['center'][1] + kept_plane['size'][1]/2) -
                                 max(plane_i['center'][1] - plane_i['size'][1]/2,
                                     kept_plane['center'][1] - kept_plane['size'][1]/2))
                                     
                overlap_area = x_overlap * y_overlap
                min_area = min(plane_i['area'], kept_plane['area'])
                
                # 如果重叠比例过大，跳过这个平面
                if overlap_area > max_overlap_ratio * min_area:
                    should_keep = False
                    break
            
            if should_keep:
                filtered_planes.append(plane_i)
                used_indices.add(i)
        
        return filtered_planes

    def merge_overlapping_planes(self, planes):
        """合并重叠且高度相近的平面"""
        if not planes:
            return []
            
        merged_planes = []
        used_indices = set()
        
        for i in range(len(planes)):
            if i in used_indices:
                continue
                
            plane_i = planes[i]
            merged_indices = {i}
            
            # 检查与其他平面的关系
            for j in range(len(planes)):
                if j in used_indices or j == i:
                    continue
                    
                plane_j = planes[j]
                
                # 检查高度差
                height_diff = abs(plane_i['height'] - plane_j['height'])
                if height_diff > self.max_merge_height_diff:
                    continue
                    
                # 计算重叠面积
                x_overlap = max(0, min(plane_i['center'][0] + plane_i['size'][0]/2,
                                     plane_j['center'][0] + plane_j['size'][0]/2) -
                                 max(plane_i['center'][0] - plane_i['size'][0]/2,
                                     plane_j['center'][0] - plane_j['size'][0]/2))
                                     
                y_overlap = max(0, min(plane_i['center'][1] + plane_i['size'][1]/2,
                                     plane_j['center'][1] + plane_j['size'][1]/2) -
                                 max(plane_i['center'][1] - plane_i['size'][1]/2,
                                     plane_j['center'][1] - plane_j['size'][1]/2))
                                     
                overlap_area = x_overlap * y_overlap
                min_area = min(plane_i['area'], plane_j['area'])
                
                # 如果重叠面积足够大且高度相近，合并这两个平面
                if overlap_area > self.min_merge_overlap_ratio * min_area:
                    merged_indices.add(j)
            
            # 如果找到了需要合并的平面
            if len(merged_indices) > 1:
                # 计算合并后的平面参数
                merged_planes_list = [planes[idx] for idx in merged_indices]
                total_area = sum(p['area'] for p in merged_planes_list)
                
                # 使用面积加权平均计算新的中心点和高度
                center_x = sum(p['center'][0] * p['area'] for p in merged_planes_list) / total_area
                center_y = sum(p['center'][1] * p['area'] for p in merged_planes_list) / total_area
                avg_height = sum(p['height'] * p['area'] for p in merged_planes_list) / total_area
                
                # 计算合并后的边界框
                min_x = min(p['center'][0] - p['size'][0]/2 for p in merged_planes_list)
                max_x = max(p['center'][0] + p['size'][0]/2 for p in merged_planes_list)
                min_y = min(p['center'][1] - p['size'][1]/2 for p in merged_planes_list)
                max_y = max(p['center'][1] + p['size'][1]/2 for p in merged_planes_list)
                
                width = max_x - min_x
                height = max_y - min_y
                
                # 创建合并后的平面
                merged_plane = {
                    'center': (center_x, center_y, avg_height),
                    'size': (width, height),
                    'area': width * height,
                    'height': avg_height
                }
                
                # 检查合并后的平面是否符合尺寸要求
                if (self.min_plane_area <= merged_plane['area'] <= self.max_plane_area and
                    self.min_plane_size <= width <= self.max_plane_size and
                    self.min_plane_size <= height <= self.max_plane_size):
                    merged_planes.append(merged_plane)
                used_indices.update(merged_indices)
            elif i not in used_indices:
                # 如果没有需要合并的平面，直接添加原平面
                merged_planes.append(plane_i)
                used_indices.add(i)
        
        return merged_planes

    def detect_stair_planes(self):
        """检测楼梯平面区域
        
        坐标系统说明：
        - 高程图已经调整为与机器人坐标系对齐
        - 机器人前方对应数组上方
        - 机器人右侧对应数组右侧
        """
        if self.elevation_map is None:
            print("没有检测到高程图")
            return []
            
        # 1. 获取有效点
        valid_mask = ~np.isnan(self.elevation_map)
        if not np.any(valid_mask):
            print("没有有效点")
            return []
            
        # 2. 准备聚类数据
        y_indices, x_indices = np.where(valid_mask)
        valid_heights = self.elevation_map[valid_mask]
        
        # 跳过低于地面高度阈值的点
        height_mask = valid_heights >= self.min_ground_height
        if not np.any(height_mask):
            return []
            
        x_indices = x_indices[height_mask]
        y_indices = y_indices[height_mask]
        valid_heights = valid_heights[height_mask]
        
        # 3. 按高度预分组
        eps_h = 0.03  # 高度差阈值（3cm）
        height_groups = {}
        
        # 对高度进行量化，将相近高度的点分到同一组
        height_bins = np.floor(valid_heights / eps_h)
        unique_bins = np.unique(height_bins)
        
        # 4. 对每个高度组进行空间聚类
        stair_planes = []
        cluster_img = np.zeros((self.elevation_map.shape[0], self.elevation_map.shape[1], 3), dtype=np.uint8)
        
        # 计算地图中心点（像素坐标）
        map_center_x = self.elevation_map.shape[1] / 2
        map_center_y = self.elevation_map.shape[0] / 2
        
        eps_xy = self.map_resolution * 2  # 空间距离阈值
        min_samples = 5  # 最小样本数
        
        total_clusters = 0
        total_noise = 0
        
        for height_bin in unique_bins:
            # 获取当前高度组的点
            height_mask = height_bins == height_bin
            current_heights = valid_heights[height_mask]
            current_x = x_indices[height_mask]
            current_y = y_indices[height_mask]
            
            # 如果点数太少，跳过
            if len(current_heights) < min_samples:
                continue
                
            # 准备空间坐标
            xy_coords = np.column_stack((
                current_x * self.map_resolution,
                current_y * self.map_resolution
            ))
            
            # 使用DBSCAN进行空间聚类
            db = DBSCAN(eps=eps_xy, min_samples=min_samples).fit(xy_coords)
            
            # 统计信息
            n_clusters = len(set(db.labels_)) - (1 if -1 in db.labels_ else 0)
            n_noise = np.sum(db.labels_ == -1)
            total_clusters += n_clusters
            total_noise += n_noise
            
            # 处理每个聚类
            for label in set(db.labels_):
                if label == -1:  # 跳过噪声点
                    continue
                    
                # 获取当前聚类的所有点
                cluster_mask = db.labels_ == label
                cluster_x = current_x[cluster_mask]
                cluster_y = current_y[cluster_mask]
                cluster_heights = current_heights[cluster_mask]
                
                # 计算统计信息
                avg_height = np.mean(cluster_heights)
                height_std = np.std(cluster_heights)
                
                # 如果高度变化在阈值内，认为是平面
                if height_std < self.max_plane_height_diff:
                    # 转换为像素坐标
                    points = np.column_stack((cluster_x, cluster_y)).astype(np.int32)
                    
                    # 计算最小外接矩形
                    rect = cv2.minAreaRect(points)
                    box = cv2.boxPoints(rect)
                    box = np.intp(box)
                    
                    # 在图像上绘制检测结果
                    color = np.random.randint(0, 255, 3).tolist()
                    cv2.drawContours(cluster_img, [box], 0, color, -1)
                    
                    # 获取矩形参数（像素坐标系）
                    center_pixel = rect[0]  # (x, y)
                    width_pixel, height_pixel = rect[1]
                    
                    # 1. 转换中心点到世界坐标系（米）
                    center_x = -(center_pixel[1] - map_center_y) * self.map_resolution
                    center_y = -(center_pixel[0] - map_center_x) * self.map_resolution
                    
                    # 2. 添加地图原点偏移
                    center_x += self.map_origin.position.x
                    center_y += self.map_origin.position.y
                    center_z = avg_height
                    
                    # 3. 计算实际尺寸（米）
                    width = width_pixel * self.map_resolution
                    height = height_pixel * self.map_resolution
                    
                    # 计算平面面积并检查是否在允许范围内
                    plane_area = width * height
                    if not (self.min_plane_area <= plane_area <= self.max_plane_area and
                           self.min_plane_size <= width <= self.max_plane_size and
                           self.min_plane_size <= height <= self.max_plane_size):
                        continue
                    
                    # 存储平面信息
                    stair_planes.append({
                        'center': (center_x, center_y, center_z),
                        'size': (width, height),
                        'area': plane_area,
                        'height': avg_height,
                        'id': -1  # 初始ID为-1
                    })
        
        print(f"总聚类数: {total_clusters}")
        # print(f"总噪声点数: {total_noise}")
        
        # 首先合并重叠的平面
        stair_planes = self.merge_overlapping_planes(stair_planes)
        
        # 然后过滤掉太近的平面
        stair_planes = self.filter_overlapping_planes(stair_planes)
        
        # 检测直线并分配ID
        self._assign_plane_ids(stair_planes)
        
        # 发布检测结果可视化
        self.plane_mask_pub.publish(self.bridge.cv2_to_imgmsg(cluster_img, "bgr8"))
        self.publish_visualization(stair_planes)
        # 按照中心坐标的 x 值递增排序
        sorted_stair_planes = sorted(stair_planes, key=lambda plane: plane['center'][0])

        # 提取排序后的中心坐标
        sorted_centers = [plane['center'] for plane in sorted_stair_planes]

        # 打印排序后的中心坐标
        # print("按照 x 坐标递增排序的台阶中心坐标:", sorted_centers)
        # 发布楼梯中心坐标
        self.publish_stair_centers(sorted_centers)

        return stair_planes

    def _assign_plane_ids(self, planes):
        """为平面分配ID
        
        规则：
        1. 检测三个或更多点是否在一条直线上
        2. 根据到机器人的距离对直线上的平面进行排序
        3. 为直线上的平面分配从0开始的ID，其他平面标记为-1
        """
        if not planes:
            return
            
        # 获取机器人位置（假设在原点）
        robot_pos = np.array([0.0, 0.0])
        
        # 提取所有平面的中心点
        centers = np.array([p['center'][:2] for p in planes])
        
        # 使用RANSAC检测直线
        best_line = None
        best_inliers = []
        best_score = 0
        
        # RANSAC参数
        n_iterations = 100
        min_inliers = 3
        distance_threshold = 0.1  # 10cm
        
        for _ in range(n_iterations):
            # 随机选择两个点
            idx = np.random.choice(len(centers), 2, replace=False)
            p1, p2 = centers[idx]
            
            # 计算直线参数
            if np.all(p1 == p2):
                continue
                
            # 计算直线方程 ax + by + c = 0
            a = p2[1] - p1[1]
            b = p1[0] - p2[0]
            c = p2[0] * p1[1] - p1[0] * p2[1]
            
            # 计算所有点到直线的距离
            distances = np.abs(a * centers[:, 0] + b * centers[:, 1] + c) / np.sqrt(a*a + b*b)
            
            # 统计内点
            inliers = np.where(distances < distance_threshold)[0]
            
            if len(inliers) >= min_inliers:
                score = len(inliers)
                if score > best_score:
                    best_score = score
                    best_line = (a, b, c)
                    best_inliers = inliers
        
        # 如果找到直线，为直线上的平面分配ID
        if best_line is not None and len(best_inliers) >= min_inliers:
            # 计算直线上的点到机器人的距离
            line_planes = [planes[i] for i in best_inliers]
            distances = [np.linalg.norm(np.array(p['center'][:2]) - robot_pos) for p in line_planes]
            
            # 根据距离排序
            sorted_indices = np.argsort(distances)
            
            # 分配ID
            for i, idx in enumerate(sorted_indices):
                planes[best_inliers[idx]]['id'] = i

    def publish_visualization(self, stair_planes):
        """在RViz中可视化检测到的平面"""
        marker_array = MarkerArray()
        
        for i, plane in enumerate(stair_planes):
            # 创建一个立方体标记表示平面
            plane_marker = Marker()
            plane_marker.header.frame_id = "odom"  # 使用odom坐标系
            plane_marker.header.stamp = rospy.Time.now()
            plane_marker.ns = "stair_planes"
            plane_marker.id = i * 2  # 使用偶数ID给平面
            plane_marker.type = Marker.CUBE
            plane_marker.action = Marker.ADD
            
            # 设置位置
            plane_marker.pose.position.x = plane['center'][0]
            plane_marker.pose.position.y = plane['center'][1]
            plane_marker.pose.position.z = plane['center'][2]
            
            # 设置方向（保持水平）
            plane_marker.pose.orientation.x = 0.0
            plane_marker.pose.orientation.y = 0.0
            plane_marker.pose.orientation.z = 0.0
            plane_marker.pose.orientation.w = 1.0
            
            # 设置尺寸
            plane_marker.scale.x = plane['size'][0]  # 长度
            plane_marker.scale.y = plane['size'][1]  # 宽度
            plane_marker.scale.z = 0.02  # 高度设置为2cm
            
            # 根据高度设置颜色
            plane_marker.color.a = 0.5  # 半透明
            # 使用高度映射到颜色
            h = (plane['height'] - self.min_stair_height) / (self.max_stair_height - self.min_stair_height)
            h = max(0.0, min(1.0, h))  # 限制在[0,1]范围内
            r, g, b = self.hsv_to_rgb(h, 1.0, 1.0)
            plane_marker.color.r = r
            plane_marker.color.g = g
            plane_marker.color.b = b
            
            marker_array.markers.append(plane_marker)
            
            # 创建一个球体标记表示中心点
            center_marker = Marker()
            center_marker.header.frame_id = "odom"
            center_marker.header.stamp = rospy.Time.now()
            center_marker.ns = "plane_centers"
            center_marker.id = i * 2 + 1  # 使用奇数ID给中心点
            center_marker.type = Marker.SPHERE
            center_marker.action = Marker.ADD
            
            # 设置位置（与平面相同）
            center_marker.pose.position.x = plane['center'][0]
            center_marker.pose.position.y = plane['center'][1]
            center_marker.pose.position.z = plane['center'][2]
            
            # 设置方向（球体不需要方向）
            center_marker.pose.orientation.w = 1.0
            
            # 设置尺寸（球体直径5cm）
            center_marker.scale.x = 0.05
            center_marker.scale.y = 0.05
            center_marker.scale.z = 0.05
            
            # 设置颜色（使用白色）
            center_marker.color.r = 1.0
            center_marker.color.g = 1.0
            center_marker.color.b = 1.0
            center_marker.color.a = 1.0  # 不透明
            
            marker_array.markers.append(center_marker)
            
            # 添加文本标记显示ID
            text_marker = Marker()
            text_marker.header.frame_id = "odom"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "plane_ids"
            text_marker.id = i * 3 + 2  # 使用新的ID序列
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # 设置位置（稍微高于平面中心）
            text_marker.pose.position.x = plane['center'][0]
            text_marker.pose.position.y = plane['center'][1]
            text_marker.pose.position.z = plane['center'][2] + 0.1  # 在平面上方10cm
            
            # 设置方向
            text_marker.pose.orientation.w = 1.0
            
            # 设置文本内容
            text_marker.text = f"{plane['id'] if 'id' in plane else 'N/A'}"
            
            # 设置文本大小
            text_marker.scale.z = 0.1  # 文本高度10cm
            
            # 设置文本颜色（白色）
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
            
        self.marker_pub.publish(marker_array)
        
    @staticmethod
    def hsv_to_rgb(h, s, v):
        """将HSV颜色转换为RGB"""
        if s == 0.0:
            return v, v, v
            
        i = int(h * 6.0)
        f = (h * 6.0) - i
        p = v * (1.0 - s)
        q = v * (1.0 - s * f)
        t = v * (1.0 - s * (1.0 - f))
        i = i % 6
        
        if i == 0:
            return v, t, p
        if i == 1:
            return q, v, p
        if i == 2:
            return p, v, t
        if i == 3:
            return p, q, v
        if i == 4:
            return t, p, v
        if i == 5:
            return v, p, q

    def publish_stair_centers(self, centers):
        """发布楼梯中心坐标，使用PointCloud2消息类型"""
        # 创建PointCloud2消息
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "odom"  # 根据实际情况修改坐标系
        
        # 准备点云数据
        points = []
        for center in centers:
            points.append([center[0], center[1], center[2]])
        
        # 创建PointCloud2消息
        pc2 = point_cloud2.create_cloud_xyz32(header, points)
        
        # 发布消息
        self.stair_centers_pub.publish(pc2)

if __name__ == '__main__':
    try:
        rospy.init_node('stair_detector', anonymous=True)
        detector = StairDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 

