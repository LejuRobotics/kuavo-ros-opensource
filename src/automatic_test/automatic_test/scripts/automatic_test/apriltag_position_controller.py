#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import os
from gazebo_msgs.srv import DeleteModel, SpawnModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty

class AprilTagPositionController:
    """
    AprilTag位置控制器
    
    使用删除和重新生成的方式来移动AprilTag
    """
    
    def __init__(self):
        # 等待Gazebo服务
        rospy.loginfo("等待Gazebo服务...")
        rospy.wait_for_service('/gazebo/delete_model', timeout=5.0)
        rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=5.0)
        # 创建服务代理
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        
        # AprilTag模型路径
        self.xacro_path = os.path.join(
            os.path.dirname(__file__), 
            '../../../../kuavo_assets/models/apriltag/urdf/ground_apriltag.xacro'
        )
        
        # 点光源SDF文件路径
        self.point_light_sdf_path = os.path.join(
            os.path.dirname(__file__), 
            '../../../../gazebo/gazebo-sim/models/point_light/point_light.sdf'
        )
        
        rospy.loginfo("AprilTag和灯光位置控制器初始化完成")
    
    def _get_xacro_content(self):
        """获取XACRO文件内容"""
        try:
            # 使用xacro命令处理文件
            cmd = f"xacro {self.xacro_path}"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            if result.returncode == 0:
                return result.stdout
            else:
                rospy.logerr(f"xacro处理失败: {result.stderr}")
                return None
        except Exception as e:
            rospy.logerr(f"读取XACRO文件失败: {e}")
            return None
    
    def move_apriltag(self, x, y, z=0.0005, yaw_rad=0.0):
        """
        移动AprilTag到新位置
        
        Args:
            x, y, z (float): 新位置坐标
            yaw_rad (float): 朝向角度（弧度）
            
        Returns:
            bool: 是否成功移动
        """
        try:
            # 1. 删除现有模型
            rospy.loginfo("删除现有AprilTag模型...")
            delete_resp = self.delete_model("ground_apriltags")
            if not delete_resp.success:
                rospy.logwarn(f"删除模型失败: {delete_resp.status_message}")
                return False
            
            rospy.sleep(0.5)  # 等待删除完成
            
            # 2. 获取XACRO内容
            xacro_content = self._get_xacro_content()
            if not xacro_content:
                rospy.logerr("无法获取XACRO内容")
                return False
            
            # 3. 创建新位姿
            pose = Pose(
                position=Point(x=x, y=y, z=z),
                orientation=Quaternion(0, 0, 0, 1)  # 无旋转
            )
            
            # 4. 重新生成模型
            rospy.loginfo(f"重新生成AprilTag模型到位置 ({x}, {y}, {z})...")
            spawn_resp = self.spawn_model(
                model_name="ground_apriltags",
                model_xml=xacro_content,
                initial_pose=pose,
                reference_frame="world"
            )
            
            if spawn_resp.success:
                rospy.loginfo("✅ AprilTag移动成功")
                return True
            else:
                rospy.logwarn(f"❌ 重新生成模型失败: {spawn_resp.status_message}")
                return False
                
        except Exception as e:
            rospy.logerr(f"移动AprilTag时发生错误: {e}")
            return False
    
    def reset_apriltag_position(self):
        """重置AprilTag到默认位置"""
        return self.move_apriltag(0, 0, 0.0005, 0)
    
    def move_apriltag_to_april_tag_area(self):
        """移动AprilTag到AprilTag区域"""
        return self.move_apriltag(0.5, 0.5, 0.0005, 0)
    
    def _get_sdf_content(self):
        """获取SDF文件内容"""
        try:
            with open(self.point_light_sdf_path, 'r') as f:
                return f.read()
        except Exception as e:
            rospy.logerr(f"读取SDF文件失败: {e}")
            return None
    
    def move_point_light(self, x, y, z, model_name='user_point_light_0'):
        """
        移动点光源到新位置
        
        Args:
            x, y, z (float): 新位置坐标
            model_name (str): 灯光模型名称
            
        Returns:
            bool: 是否成功移动
        """
        try:
            # 1. 删除现有灯光模型
            rospy.loginfo(f"删除现有灯光模型 {model_name}...")
            delete_resp = self.delete_model(model_name)
            if not delete_resp.success:
                rospy.logwarn(f"删除灯光模型失败: {delete_resp.status_message}")
                return False
            
            rospy.sleep(0.5)  # 等待删除完成
            
            # 2. 使用命令行方式重新生成灯光模型
            rospy.loginfo(f"重新生成灯光模型到位置 ({x}, {y}, {z})...")
            
            # 构建spawn命令
            spawn_cmd = [
                'rosrun', 'gazebo_ros', 'spawn_model',
                '-sdf', '-model', model_name,
                '-file', self.point_light_sdf_path,
                '-x', str(x), '-y', str(y), '-z', str(z)
            ]
            
            # 执行命令
            result = subprocess.run(spawn_cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                rospy.loginfo("✅ 灯光移动成功")
                return True
            else:
                rospy.logwarn(f"❌ 重新生成灯光模型失败: {result.stderr}")
                return False
                
        except Exception as e:
            rospy.logerr(f"移动灯光时发生错误: {e}")
            return False
    
    def reset_point_light_position(self, model_name='user_point_light_0'):
        """重置灯光到默认位置"""
        return self.move_point_light(0, 0, 1.0, model_name)
    
    def move_point_light_to_april_tag_area(self, model_name='user_point_light_0'):
        """移动灯光到AprilTag区域上方"""
        return self.move_point_light(0.5, 0.5, 1.5, model_name)

def main():
    """主函数 - 演示AprilTag位置控制"""
    rospy.init_node('apriltag_position_controller', anonymous=True)
    
    # 创建控制器
    controller = AprilTagPositionController()
    
    # 等待系统稳定
    rospy.sleep(2.0)
    
    print("=== AprilTag和灯光位置控制演示 ===")
    
    # 1. 移动AprilTag到新位置
    print("1. 移动AprilTag到位置 (2, 0)...")
    if controller.move_apriltag(2, 0, 0.0005, 0):
        print("   AprilTag移动成功")
    else:
        print("   AprilTag移动失败")
    
    rospy.sleep(2.0)  # 等待观察
    
    # 2. 移动灯光到AprilTag区域上方
    print("2. 移动灯光到AprilTag区域上方...")
    if controller.move_point_light(2, 0, 1.5):
        print("   灯光移动成功")
    else:
        print("   灯光移动失败")
    
    rospy.sleep(2.0)  # 等待观察
    
    # 3. 移动AprilTag到另一个位置
    print("3. 移动AprilTag到位置 (0, 2)...")
    if controller.move_apriltag(0, 2, 0.0005, 0):
        print("   AprilTag移动成功")
    else:
        print("   AprilTag移动失败")
    
    rospy.sleep(2.0)  # 等待观察
    
    # 4. 移动灯光跟随AprilTag
    print("4. 移动灯光跟随AprilTag...")
    if controller.move_point_light(0, 2, 1.5):
        print("   灯光移动成功")
    else:
        print("   灯光移动失败")
    
    rospy.sleep(2.0)  # 等待观察
    
    # 5. 重置AprilTag到默认位置
    print("5. 重置AprilTag到默认位置...")
    if controller.reset_apriltag_position():
        print("   AprilTag重置成功")
    else:
        print("   AprilTag重置失败")
    
    # 6. 重置灯光到默认位置
    print("6. 重置灯光到默认位置...")
    if controller.reset_point_light_position():
        print("   灯光重置成功")
    else:
        print("   灯光重置失败")
    
    print("\n演示完成！")
    print("\n使用方法:")
    print("rosrun automatic_test apriltag_position_controller.py")
    print("或者在其他脚本中导入:")
    print("from apriltag_position_controller import AprilTagPositionController")
    print("controller = AprilTagPositionController()")
    print("controller.move_apriltag(x, y, z, yaw)")
    print("controller.move_point_light(x, y, z, model_name)")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
