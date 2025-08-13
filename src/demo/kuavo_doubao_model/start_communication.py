import sys
import select
from robot_speech import RobotSpeech
import rospy

def main():
    rospy.init_node('voice_conversation', anonymous=True)
    robot_speech = RobotSpeech()

    if not robot_speech.establish_doubao_speech_connection(
        app_id="xxxxx", 
        access_key="xxxxx"
    ):
        return

    robot_speech.start_speech()
    print("输入c停止")

    while not rospy.is_shutdown():
        # 非阻塞检测键盘输入
        if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
            if sys.stdin.read(1).lower() == 'c':
                break

    robot_speech.stop_speech()

if __name__ == "__main__":
    main()
    
