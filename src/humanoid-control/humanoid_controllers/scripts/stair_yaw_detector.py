#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32MultiArray
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN
import tf2_ros
import tf2_geometry_msgs

class AdvancedStairYawDetector:
    def __init__(self):
        self.boundary_sub = rospy.Subscriber(
            "/leju/fused_hull_boundry_fuse_elevation", 
            MarkerArray, 
            self.boundary_callback
        )
        
        self.yaw_pub = rospy.Publisher("/stair_segments_yaw", Float32MultiArray, queue_size=10)
        
        self.min_points = 3  # 最小点数要求
        self.dbscan_eps = 0.2  # DBSCAN聚类半径
        self.dbscan_min_samples = 3  # DBSCAN最小样本数
        
        # 角度修正参数
        self.flip_direction = True      # 翻转方向标志
        self.angle_offset = 1.57        # 角度偏移(rad)，根据实际情况调整
        
        # TF2相关设置
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.source_frame = "odom"
        self.target_frame = "base_link"
        self.transform_timeout = rospy.Duration(1.0)  # TF转换超时时间
        
        rospy.loginfo("Advanced stair yaw detector initialized with angle correction")
    
    def boundary_callback(self, marker_array):
        stair_segments = self.extract_stair_segments(marker_array)
        if not stair_segments:
            rospy.logwarn("No stair segments found")
            return
        
        # 计算每个楼梯段的yaw
        self.stair_yaws = [self.calculate_segment_yaw(seg) for seg in stair_segments]
        self.corrected_yaws = [self.correct_angle(yaw) for yaw in self.stair_yaws]
        
        # 发布修正后的yaw值
        yaw_msg = Float32MultiArray()
        yaw_msg.data = self.corrected_yaws
        self.yaw_pub.publish(yaw_msg)
        
        # 输出原始和修正后的角度
        rospy.loginfo(f"检测到 {len(self.stair_yaws)} 个楼梯段:")
        for i, (corrected) in enumerate(self.corrected_yaws):
            rospy.loginfo(f"  楼梯段 {i}: Yaw = {corrected:.4f} rad")
    
    def extract_stair_segments(self, marker_array):
        """从MarkerArray中提取楼梯段，处理非四个点的情况"""
        segments = []
        
        for marker in marker_array.markers:
            points = [(p.x, p.y, p.z) for p in marker.points]
            
            # 处理点数不足的情况
            if len(points) < self.min_points:
                rospy.logwarn(f"Skipping segment with {len(points)} points (min required: {self.min_points})")
                continue
            
            # 坐标转换: 从odom转换到base_link
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, 
                    self.source_frame, 
                    rospy.Time(0), 
                    self.transform_timeout
                )
                transformed_points = self.transform_points(points, transform)
                segments.append(transformed_points)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"TF transform error: {e}")
                continue
        
        return segments
    
    def transform_points(self, points, transform):
        """使用tf2直接进行坐标转换"""
        transformed = []
        
        for p in points:
            # 创建PointStamped消息
            point_stamped = tf2_geometry_msgs.PointStamped()
            point_stamped.header.frame_id = self.source_frame
            point_stamped.header.stamp = rospy.Time(0)
            point_stamped.point.x = p[0]
            point_stamped.point.y = p[1]
            point_stamped.point.z = p[2]
            
            try:
                # 执行坐标转换
                transformed_point = self.tf_buffer.transform(point_stamped, self.target_frame, timeout=self.transform_timeout)
                transformed.append((transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))
            except Exception as e:
                rospy.logerr(f"Error transforming point: {e}")
                # 如果转换失败，保留原始点
                transformed.append(p)
        
        return transformed
    
    def process_non_rectangular_points(self, points):
        """处理非矩形点集的策略"""
        xy_points = np.array([[p[0], p[1]] for p in points])
        
        # 尝试聚类（处理多节楼梯或噪声点）
        if len(points) > 8:  # 点数较多时可能是多个楼梯段或噪声
            clusters = self.cluster_points(xy_points)
            if clusters and len(clusters[0]) >= self.min_points:
                # 只返回最大的聚类
                largest_cluster = max(clusters, key=len)
                return [(p[0], p[1], 0.0) for p in largest_cluster]
        
        # 尝试凸包提取（处理不规则形状）
        try:
            hull = ConvexHull(xy_points)
            convex_points = xy_points[hull.vertices]
            return [(p[0], p[1], 0.0) for p in convex_points]
        except:
            rospy.logwarn("Failed to compute convex hull, using all points")
            return points
    
    def cluster_points(self, points):
        """使用DBSCAN算法对点集进行聚类"""
        if len(points) < self.dbscan_min_samples:
            return []
        
        db = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_
        
        # 分离不同的聚类
        clusters = []
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)  # -1是噪声点
        
        for cluster_id in range(n_clusters):
            cluster_points = points[labels == cluster_id]
            if len(cluster_points) >= self.min_points:
                clusters.append(cluster_points)
        
        return clusters
    
    def calculate_segment_yaw(self, segment_points):
        """计算楼梯段的Yaw值，支持任意数量的点"""
        if len(segment_points) < 2:
            return 0.0
        
        xy_points = np.array([[p[0], p[1]] for p in segment_points])
        
        # 使用主成分分析确定主轴方向
        center = np.mean(xy_points, axis=0)
        centered_data = xy_points - center
        cov_matrix = np.cov(centered_data.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        major_axis = eigenvectors[:, np.argmax(eigenvalues)]
        
        # 计算Yaw值
        yaw = np.arctan2(major_axis[1], major_axis[0])
        
        return yaw
    
    def correct_angle(self, raw_yaw):
        """修正角度偏差"""
        # 翻转方向
        # corrected_yaw = raw_yaw if not self.flip_direction else -raw_yaw
        corrected_yaw = raw_yaw
        
        # 应用角度偏移
        corrected_yaw += self.angle_offset
        
        # 归一化到[-π, π]
        corrected_yaw = (corrected_yaw + np.pi) % (2 * np.pi) - np.pi
        
        return corrected_yaw

if __name__ == "__main__":
    rospy.init_node("advanced_stair_yaw_detector")
    detector = AdvancedStairYawDetector()
    rospy.spin()
