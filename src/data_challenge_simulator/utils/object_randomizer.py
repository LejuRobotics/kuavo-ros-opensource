import rospy
from kuavo_msgs.srv import SetObjectPosition, SetObjectPositionRequest
import sys

class ObjectRandomizer:
    def __init__(self, timeout=10.0):
        """
        初始化物体随机化器
        
        Args:
            timeout (float): 等待服务的超时时间
        """
        self.service_name = '/set_object_position'
        self.timeout = timeout
        self._service_proxy = None
        self._init_service_proxy()
    
    def _init_service_proxy(self):
        """初始化服务代理"""
        try:
            rospy.loginfo(f"Waiting for service {self.service_name}...")
            rospy.wait_for_service(self.service_name, timeout=self.timeout)
            self._service_proxy = rospy.ServiceProxy(self.service_name, SetObjectPosition)
            rospy.loginfo(f"Service {self.service_name} is ready!")
        except rospy.ROSException as e:
            rospy.logerr(f"Service {self.service_name} not available: {e}")
            sys.exit(1)
    
    def set_object_position(self, object_name, position=None, orientation=None):
        """
        设置物体到指定位置和姿态
        
        Args:
            object_name (str): 物体名称
            position (dict): 位置 {'x': float, 'y': float, 'z': float}
            orientation (dict): 姿态四元数 {'w': float, 'x': float, 'y': float, 'z': float}
        
        Returns:
            dict: 操作结果 {'success': bool, 'message': str, 'final_position': dict}
        """
        if not self._service_proxy:
            return {'success': False, 'message': 'Service proxy not initialized'}
        
        req = SetObjectPositionRequest()
        req.object_name = object_name
        req.randomize = False
        
        # 设置位置
        if position:
            req.position.x = position.get('x', 0.0)
            req.position.y = position.get('y', 0.0)
            req.position.z = position.get('z', 0.0)
        
        # 设置姿态
        if orientation:
            req.orientation.x = orientation.get('x', 0.0)
            req.orientation.y = orientation.get('y', 0.0)
            req.orientation.z = orientation.get('z', 0.0)
            req.orientation.w = orientation.get('w', 0.0)
        
        try:
            response = self._service_proxy(req)
            return {
                'success': response.success,
                'message': response.message,
                'final_position': {
                    'x': response.final_position.x,
                    'y': response.final_position.y,
                    'z': response.final_position.z
                }
            }
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return {'success': False, 'message': f'Service call failed: {e}'}
    
    def randomize_object_position(self, object_name, position_ranges, orientation=None):
        """
        随机化物体位置
        
        Args:
            object_name (str): 物体名称
            position_ranges (dict): 位置范围 {
                'x': [min_val, max_val],
                'y': [min_val, max_val], 
                'z': [min_val, max_val]
            }
            orientation (dict, optional): 固定姿态，如果不提供则保持原有姿态
        
        Returns:
            dict: 操作结果 {'success': bool, 'message': str, 'final_position': dict}
        """
        if not self._service_proxy:
            return {'success': False, 'message': 'Service proxy not initialized'}
        
        req = SetObjectPositionRequest()
        req.object_name = object_name
        req.randomize = True
        
        # 设置随机范围
        req.x_min = position_ranges['x'][0]
        req.x_max = position_ranges['x'][1]
        req.y_min = position_ranges['y'][0]
        req.y_max = position_ranges['y'][1]
        req.z_min = position_ranges['z'][0]
        req.z_max = position_ranges['z'][1]
        
        if orientation:
            req.orientation.x = orientation.get('x', 0.0)
            req.orientation.y = orientation.get('y', 0.0)
            req.orientation.z = orientation.get('z', 0.0)
            req.orientation.w = orientation.get('w', 0.0)
        
        try:
            response = self._service_proxy(req)
            return {
                'success': response.success,
                'message': response.message,
                'final_position': {
                    'x': response.final_position.x,
                    'y': response.final_position.y,
                    'z': response.final_position.z
                }
            }
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return {'success': False, 'message': f'Service call failed: {e}'}
    
    def randomize_multiple_objects(self, object_configs):
        """
        随机化多个物体的位置
        
        Args:
            object_configs (list): 物体配置列表，每个元素是一个字典：
            {
                'name': 'object_name',
                'position_ranges': {
                    'x': [min_val, max_val],
                    'y': [min_val, max_val],
                    'z': [min_val, max_val]
                },
                'orientation': {'w': float, 'x': float, 'y': float, 'z': float} (可选)
            }
        
        Returns:
            dict: 操作结果汇总
        """
        results = {}
        success_count = 0
        
        for config in object_configs:
            object_name = config['name']
            position_ranges = config['position_ranges']
            orientation = config.get('orientation', None)
            
            result = self.randomize_object_position(object_name, position_ranges, orientation)
            results[object_name] = result
            
            if result['success']:
                success_count += 1
                rospy.loginfo(f"Successfully randomized {object_name} to position {result['final_position']}")
            else:
                rospy.logwarn(f"Failed to randomize {object_name}: {result['message']}")
        
        return {
            'results': results,
            'success_count': success_count,
            'total_count': len(object_configs),
            'all_success': success_count == len(object_configs)
        }
