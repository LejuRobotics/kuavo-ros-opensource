# from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera, IMUSensor
import numpy as np
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import struct
"""
    imu_data:  
    { 'time': 0.27000001072883606, 
      'physics_step': 111.0, 
      'lin_acc': array([ 0.22668934, -0.01075992,  9.798907  ], dtype=float32), 
      'ang_vel': array([-0.0045824 , -0.03133204, -0.00079955], dtype=float32), 
      'orientation': array([ 9.9981683e-01,  4.3590643e-04, -1.9111114e-02,  9.3401712e-04], dtype=float32)}
"""

class Sensor:
    def __init__(self, usd_path, prim_path, enable_pointcloud_flag=False):  # pylint: disable=W0613
        # add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        # 配置彩色相机
        self.color_sensor = Camera(
            prim_path=f"{prim_path}/camera_base/RSD455/Camera_OmniVision_OV9782_Color",
            name="color_camera",
            resolution=[640, 480],  # D455默认分辨率
            frequency=30,  # D455默认帧率30fps
            position=None,
            orientation=None,
        )
        
        # 配置深度相机
        self.depth_sensor = Camera(
            prim_path=f"{prim_path}/camera_base/RSD455/Camera_Pseudo_Depth",
            name="depth_camera",
            resolution=[640, 480],
            frequency=30,
            position=None,
            orientation=None,
        )
        
        # 创建ros发布器
        self.freq_pub = rospy.Publisher('/kuavo_isaac_sim/sensor_estimate', Float32, queue_size=10)
        self.rgb_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)
        self.pointcloud_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=10)
        self.bridge = CvBridge()
        
        # 创建IMU传感器
        self.imu_sensor = IMUSensor(
            prim_path=f"{prim_path}/base_link/Imu_Sensor",
            name="imu_sensor",
            frequency=1000,
            linear_acceleration_filter_size=10,
            angular_velocity_filter_size=5,
            orientation_filter_size=5
        )
        
        # 预计算点云字段
        self.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1),
        ]

        # 是否使能标志
        self.enable_pointcloud_flag_bool = enable_pointcloud_flag
    
    def initialize(self):
        """初始化传感器"""
        # 初始化相机
        self.color_sensor.initialize()
        self.depth_sensor.initialize()
        self.imu_sensor.initialize()
        
        # 配置RGB相机
        self.color_sensor.set_projection_mode("perspective")
        self.color_sensor.set_focal_length(1.93)
        
        # 配置深度相机
        self.depth_sensor.set_projection_mode("perspective")
        self.depth_sensor.set_focal_length(1.93)
        self.depth_sensor.set_clipping_range(0.6, 6.0)  # 设置深度范围
        
        try:
            # 为深度相机添加必要的注释器
            self.depth_sensor.add_distance_to_image_plane_to_frame()
            self.depth_sensor.add_normals_to_frame()
            self.depth_sensor.add_motion_vectors_to_frame()
            self.depth_sensor.add_semantic_segmentation_to_frame()
            if self.enable_pointcloud_flag_bool:
                # 添加点云支持
                self.depth_sensor.add_pointcloud_to_frame(include_unlabelled=True)
            
            # 设置相机内参（可选）
            aspect_ratio = self.depth_sensor.get_aspect_ratio()
            horizontal_aperture = self.depth_sensor.get_horizontal_aperture()
            vertical_aperture = horizontal_aperture / aspect_ratio
            self.depth_sensor.set_vertical_aperture(vertical_aperture)
            
        except Exception as e:
            print(f"Warning: Failed to configure depth camera features: {e}")

    def create_point_cloud(self, rgb_data, depth_data):
        """创建点云数据"""
        try:
            if rgb_data is None or depth_data is None:
                return None

            height, width = depth_data.shape
            fx = self.depth_sensor.get_focal_length()
            fy = fx  # 假设fx=fy
            cx = width / 2
            cy = height / 2

            points = []
            for v in range(height):
                for u in range(width):
                    z = depth_data[v, u]
                    if z > 0:  # 忽略无效深度值
                        # 反投影到3D空间
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        
                        # 获取对应的RGB值
                        r = float(rgb_data[v, u, 0]) / 255.0
                        g = float(rgb_data[v, u, 1]) / 255.0
                        b = float(rgb_data[v, u, 2]) / 255.0
                        
                        # 打包RGB值为一个浮点数
                        rgb = struct.unpack('f', struct.pack('i', 
                            int(r*255) << 16 | int(g*255) << 8 | int(b*255)))[0]
                        
                        points.append([x, y, z, rgb])

            return np.array(points, dtype=np.float32)
            
        except Exception as e:
            print(f"Error creating point cloud: {e}")
            return None

    def get_obs(self):
        """获取观测数据"""
        try:
            # 更新当前帧
            color_frame = self.color_sensor.get_current_frame()
            depth_frame = self.depth_sensor.get_current_frame()
            
            # 获取RGB和深度数据
            rgb_data = self.color_sensor.get_rgb()
            
            try:
                if self.enable_pointcloud_flag_bool:
                    # 尝试直接获取点云数据 | 该方法可以直接获取点云数据
                    pointcloud_data = self.depth_sensor.get_pointcloud()
                    if pointcloud_data is not None:
                        # 创建点云消息
                        header = rospy.Header()
                        header.stamp = rospy.Time.now()
                        header.frame_id = "camera_depth_optical_frame"
                        
                        # 修改点云字段定义，只包含xyz坐标
                        fields = [
                            PointField('x', 0, PointField.FLOAT32, 1),
                            PointField('y', 4, PointField.FLOAT32, 1),
                            PointField('z', 8, PointField.FLOAT32, 1)
                        ]
                        # 创建点云消息
                        pc_msg = pc2.create_cloud(header, fields, pointcloud_data)
                        self.pointcloud_pub.publish(pc_msg)
                        
                # 获取深度数据
                try:
                    # 尝试获取深度图像
                    depth_data = depth_frame.get("distance_to_image_plane")
                    if depth_data is None:
                        # 如果上面的方法失败，尝试直接获取深度数据
                        depth_data = self.depth_sensor.get_depth()
                        
                    # 打印深度数据信息用于调试
                    if depth_data is not None:
                        # print("Depth data info:")
                        # print(f"Shape: {depth_data.shape}")
                        # print(f"Type: {depth_data.dtype}")
                        # print(f"Range: [{np.min(depth_data)}, {np.max(depth_data)}]")
                        
                        # 确保数据类型正确
                        if depth_data.dtype != np.float32:
                            depth_data = depth_data.astype(np.float32)
                    
                except Exception as e:
                    print(f"Error getting depth data: {e}")
                    depth_data = None
                
                # 发布RGB图像
                if rgb_data is not None:
                    try:
                        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_data, encoding="rgb8")
                        rgb_msg.header.stamp = rospy.Time.now()
                        rgb_msg.header.frame_id = "camera_color_optical_frame"
                        self.rgb_pub.publish(rgb_msg)
                    except Exception as e:
                        print(f"Error publishing RGB image: {e}")
                
                # 发布深度图像
                if depth_data is not None:
                    try:
                        depth_msg = self.bridge.cv2_to_imgmsg(depth_data, encoding="32FC1")
                        depth_msg.header.stamp = rospy.Time.now()
                        depth_msg.header.frame_id = "camera_depth_optical_frame"
                        self.depth_pub.publish(depth_msg)
                    except Exception as e:
                        print(f"Error publishing depth image: {e}")
                
                return {
                    "rgb": rgb_data,
                    "depth": depth_data,
                    "color_frame": color_frame,
                    "depth_frame": depth_frame
                }
                
            except Exception as e:
                print(f"Error processing depth/pointcloud data: {e}")
            
        except Exception as e:
            print(f"Error in get_obs: {e}")
            return {
                "rgb": None,
                "depth": None,
                "color_frame": None,
                "depth_frame": None
            }
