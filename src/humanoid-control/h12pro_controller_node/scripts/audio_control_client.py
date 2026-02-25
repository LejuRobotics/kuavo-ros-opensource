#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
from h12pro_controller_node.srv import audio_control

class AudioControlClient:
    def __init__(self):
        rospy.init_node('audio_control_client', anonymous=True)

        # 等待服务可用
        rospy.loginfo("Waiting for audio_control service...")
        rospy.wait_for_service('audio_control')

        # 创建服务代理
        self.audio_control_service = rospy.ServiceProxy('audio_control', audio_control)

        rospy.loginfo("Audio Control Client ready")

    def send_request(self, action_type, launch_file, session_name):
        """发送音频控制请求"""
        try:
            rospy.loginfo(f"Sending request: action={action_type}, launch_file={launch_file}, session={session_name}")

            # 发送服务请求
            response = self.audio_control_service(action_type, launch_file, session_name)

            # 处理响应
            if response.success:
                rospy.loginfo(f"✅ Success: {response.message}")
                return True
            else:
                rospy.logerr(f"❌ Failed: {response.message}")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
            return False

def main():
    # 使用launch文件传递参数
    try:
        # 等待ROS参数服务器初始化
        rospy.sleep(0.1)

        # 获取参数，使用多种尝试方式
        action = None
        launch_file = None
        session_name = None

        # 尝试不同的参数访问方式
        param_names = [
            'default_action', '~default_action', '/default_action',
            'action', '~action', '/action'
        ]

        for param_name in param_names:
            try:
                action = rospy.get_param(param_name)
                rospy.loginfo(f"✅ Found action parameter: {param_name} = {action}")
                break
            except:
                continue

        param_names = [
            'default_launch_file', '~default_launch_file', '/default_launch_file',
            'launch_file', '~launch_file', '/launch_file'
        ]

        for param_name in param_names:
            try:
                launch_file = rospy.get_param(param_name)
                rospy.loginfo(f"✅ Found launch_file parameter: {param_name} = {launch_file}")
                break
            except:
                continue

        param_names = [
            'default_session_name', '~default_session_name', '/default_session_name',
            'session_name', '~session_name', '/session_name'
        ]

        for param_name in param_names:
            try:
                session_name = rospy.get_param(param_name)
                rospy.loginfo(f"✅ Found session_name parameter: {param_name} = {session_name}")
                break
            except:
                continue

        # 检查是否所有参数都找到了
        if action is None or launch_file is None or session_name is None:
            raise KeyError("One or more required parameters are missing")

        rospy.loginfo(f"🎯 Successfully loaded parameters:")
        rospy.loginfo(f"   - action: {action}")
        rospy.loginfo(f"   - launch_file: {launch_file}")
        rospy.loginfo(f"   - session_name: {session_name}")

    except KeyError as e:
        rospy.logerr(f"❌ Missing required ROS parameter: {e}")
        rospy.logerr("Please ensure all required parameters are set in the launch file")
        sys.exit(1)
    except Exception as e:
        rospy.logerr(f"❌ Failed to get ROS parameters: {e}")
        sys.exit(1)

    try:
        client = AudioControlClient()
        success = client.send_request(action, launch_file, session_name)

        if success:
            print(f"🎵 Audio control operation '{action}' completed successfully!")
        else:
            print(f"💥 Audio control operation '{action}' failed!")
            sys.exit(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Audio Control Client interrupted")
        sys.exit(1)

if __name__ == "__main__":
    main()
