
import time
from kuavo_humanoid_sdk import RobotControlBlockly, RobotMicrophone, RobotNavigation, RobotSpeech

def main():
    # 创建机器人控制实例
    robot_control = RobotControlBlockly()
    robot_navigation = RobotNavigation()
    robot_speech = RobotSpeech()

    map = "map_2025-06-30_02-48-15"
    robot_navigation.load_map(map)
    # robot_navigation.init_localization_by_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    robot_navigation.navigate_to_goal(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    robot_navigation.navigate_to_task_point("task1")
    robot_navigation.navigate_to_task_point("task2")
    print(robot_navigation.get_all_maps())
    map = robot_navigation.get_current_map()
    print(map)

    # 设置手臂模式
    robot_control.control_robot_height("down", 0.2, 0.0)
    robot_control.set_arm_control_mode(0)

    # 启动机器人运动
    robot_control.start()
    time.sleep(4)

    # 开始行走
    robot_control.walk(0.4, 0.0, 0.0)
    time.sleep(1)
    robot_control.walk(0.3, 0.0, 0.0)
    
    time.sleep(4)
    # 停止机器人
    robot_control.stop()
    
    robot_control.set_arm_control_mode(1)
    
    # 开始行走
    robot_control.walk(0.4, 0.0, 0.0)
    
    time.sleep(4)
    # 停止机器人
    robot_control.stop()

    # 执行动作文件
    robot_control.excute_action_file("roban2")
    # 执行指定目录动作文件
    robot_control.excute_action_file("roban2", "roban2")
    # 执行指定目录动作文件并播放音乐
    robot_control.excute_action_file("roban2", "roban2", "2_抱拳.wav")
    # 播放音乐
    robot_control.play_music("2_抱拳.wav")
    # 停止播放音乐
    robot_control.stop_music()
    # 执行动作文件并播放音乐
    robot_control.excute_action_file("roban2", music_file="2_抱拳.wav")
    # 复位机器人本体    
    robot_control.to_stance()

    # 控制机器人头部运动
    robot_control.control_robot_head(25, 25)
    # time.sleep(3)
    robot_control.control_robot_head(-10, 10)
    # time.sleep(3)
    robot_control.control_robot_head(0, 0)
    # time.sleep(3)
    robot_control.control_robot_head_only_yaw(25)
    # time.sleep(3)
    robot_control.control_robot_head_only_yaw(-25)
    # time.sleep(3)
    robot_control.control_robot_head_only_pitch(25)
    # time.sleep(3)
    robot_control.control_robot_head_only_pitch(10)
    # time.sleep(2)

    # 控制机器人高度
    robot_control.control_robot_height("down", 0.1, 0.0)
    # time.sleep(4)
    robot_control.control_robot_height("down", 0.2, 0.0)
    # time.sleep(4)
    robot_control.control_robot_height("up", 0.1, 0.0)
    # time.sleep(4)
    robot_control.control_robot_height("up", 0.15, 0.0)
    # time.sleep(4)

    # # 控制机器人手臂运动
    robot_control.to_stance()
    robot_control.arm_reset()
    robot_control.control_arm_target_pose_by_single("left", 0.223, 0.263, -0.116, -180, -31.324,-0.000)
    robot_control.control_arm_target_pose_by_single("right", 0.223, -0.263, -0.116, -180, -31.324,-0.000)
    robot_control.to_stance()
    robot_control.control_arm_target_pose(0.223, 0.263, -0.116, -180, -31.324,-0.000, 0.223, -0.263, -0.116, -180, -31.324,-0.000)
    robot_control.to_stance()


    # 使用麦克风检测关键词
    mirc = RobotMicrophone()
    # 等待唤醒词，60秒超时
    timeout = 60
    if mirc.wait_for_wake_word(timeout):
        ## 一些动作
        print("Wake word detected")
        robot_control.control_arm_target_pose(0.223, 0.263, -0.116, -180, -31.324,-0.000, 0.223, -0.263, -0.116, -180, -31.324,-0.000)
    else:
        ## 检测到唤醒词失败
        print("Wake word not detected")

    
    # 语音对话功能
    if robot_speech.establish_doubao_speech_connection(app_id="", access_key=""):
        robot_speech.start_speech()
        time.sleep(120)
        robot_speech.stop_speech()

if __name__ == "__main__":
    # 运行主函数
    main()
