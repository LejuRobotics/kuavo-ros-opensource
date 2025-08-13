#!/usr/bin/env python3
# coding: utf-8
import rospy
from std_msgs.msg import Bool,String
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import LoadMap, LoadMapRequest, LoadMapResponse
from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import GetCurrentMap, GetCurrentMapRequest, GetCurrentMapResponse
from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import GetAllMaps, GetAllMapsRequest, GetAllMapsResponse
from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import TaskPointOperation, TaskPointOperationRequest, TaskPointOperationResponse
from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import InitialPoseWithTaskPoint, InitialPoseWithTaskPointRequest, InitialPoseWithTaskPointResponse
from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import NavigateToTaskPoint, NavigateToTaskPointRequest, NavigateToTaskPointResponse
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import TaskPoint as TaskPointMsg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray,GoalID
from enum import Enum
class NavigationStatus(Enum):
    """Navigation status."""
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    
class Navigation:
    """Navigation system interface for controlling navigation functionality of Kuavo humanoid robot.
    
    Provides functionality to play music files.
    """
    
    def __init__(self):
        """Initialize the navigation system."""
        self.init_with_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)
        self.init_with_task_point_pub = rospy.Publisher('initialpose_with_taskpoint', String, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10, latch=True)
        self.move_base_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.status = NavigationStatus.PENDING
         # Wait for publisher initialization

    def status_callback(self, msg: GoalStatusArray):
        """Callback for the status topic."""
        if len(msg.status_list) > 0:
            self.status = NavigationStatus(msg.status_list[0].status)

    def srv_load_map(self, map_name: str):
        """Load the map."""
        try:
            service_name = '/load_map'
            rospy.wait_for_service(service_name,timeout=3.0)
            load_map_client = rospy.ServiceProxy(service_name, LoadMap)

            # request
            req = LoadMapRequest()
            req.map_name = map_name

            # response  
            res = load_map_client(req)
            if res.success:
                return True
            else:
                return False
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service `load_map` call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Service `load_map` call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"Service `load_map` call failed: {e}")
        return False
    
    def srv_get_current_map(self):
        """Get the current map."""
        try:
            service_name = 'get_current_map'
            rospy.wait_for_service(service_name,timeout=3.0)
            get_current_map_client = rospy.ServiceProxy(service_name, GetCurrentMap)        

            # request
            req = GetCurrentMapRequest()

            # response
            res = get_current_map_client(req)
            return res.current_map
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service `get_current_map` call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Service `get_current_map` call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"Service `get_current_map` call failed: {e}")
        return None
    
    def srv_get_all_maps(self):
        """Get all maps."""
        try:
            service_name = 'get_all_maps'
            rospy.wait_for_service(service_name,timeout=3.0)
            get_all_maps_client = rospy.ServiceProxy(service_name, GetAllMaps)  
            
            # request   
            req = GetAllMapsRequest()

            # response
            res = get_all_maps_client(req)
            return res.maps
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service `get_all_maps` call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Service `get_all_maps` call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"Service `get_all_maps` call failed: {e}")
        return None
    
    def srv_navigate_to_task_point(self, task_point_name: String):
        """Navigate to the task point."""
        try:
            service_name = 'navigate_to_task_point'
            rospy.wait_for_service(service_name,timeout=3.0)
            navigate_to_task_point_client = rospy.ServiceProxy(service_name, NavigateToTaskPoint)
            
            # request
            req = NavigateToTaskPointRequest()
            req.task_name = task_point_name
            
            # response
            res = navigate_to_task_point_client(req)
            return res.success
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service `navigate_to_task_point` call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Service `navigate_to_task_point` call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"Service `navigate_to_task_point` call failed: {e}")
        return False
    
    def srv_init_localization_by_task_point(self, task_point_name: str):
        """Initialize the localization by task point."""
        try:
            service_name = '/initialpose_with_taskpoint'
            rospy.wait_for_service(service_name,timeout=3.0)
            init_localization_by_task_point_client = rospy.ServiceProxy(service_name, InitialPoseWithTaskPoint)
            
            # request
            req = InitialPoseWithTaskPointRequest()
            req.task_point_name = task_point_name
            # response
            res = init_localization_by_task_point_client(req)
            return res.success
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service `/initialpose_with_taskpoint` call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Service `/initialpose_with_taskpoint` call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"Service `/initialpose_with_taskpoint` call failed: {e}")
        return False
    
    def pub_init_localization_by_pose(self, pose: Pose):
        """Initialize the localization by pose."""
        try:
            # request
            initialpose_msg = PoseWithCovarianceStamped()
            initialpose_msg.header.frame_id = "map"
            initialpose_msg.header.stamp = rospy.Time.now()
            initialpose_msg.pose.pose = pose
            
            # 转发到/initialpose话题
            self.init_with_pose_pub.publish(initialpose_msg)
        except Exception as e:
            SDKLogger.error(f"Failed to initialize localization by pose: {e}")
            return False
        return True
            
    def pub_init_localization_by_task_point(self, task_point_name: str):
        """Initialize the localization by task point."""
        try:
            self.init_with_task_point_pub.publish(task_point_name)
            rospy.sleep(3)
        except Exception as e:
            SDKLogger.error(f"Failed to initialize localization by task point: {e}")
            return False
        return True

    def pub_navigate_to_goal(self, goal: Pose): 
        """Navigate to the goal."""
        try:
            # request
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.pose = goal

            # 转发到/move_base/goal话题
            self.goal_pub.publish(goal_msg)
        except Exception as e:
            SDKLogger.error(f"Failed to navigate to goal: {e}")
            return False
        return True

    def get_current_status(self):
        """Get the current status."""
        return self.status

    def pub_stop_navigation(self):
        """Stop the navigation.Cancel all goal."""
        try:
            cancel_msg = GoalID()
            cancel_msg.goal_id = self.goal_id
            self.move_base_cancel.publish(cancel_msg)
        except Exception as e:
            SDKLogger.error(f"Failed to stop navigation: {e}")
            return False
        return True



    
    
