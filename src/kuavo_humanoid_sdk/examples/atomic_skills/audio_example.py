import time
from kuavo_humanoid_sdk import KuavoSDK,KuavoRobotAudio

if not KuavoSDK().Init(log_level='INFO'):# Init!
    print("Init KuavoSDK failed, exit!")
    exit(1)
    
if __name__ == "__main__":
    kuavo_robot = KuavoRobotAudio()
    kuavo_robot.play_audio("2_抱拳.wav")
    time.sleep(3)
    kuavo_robot.text_to_speech("你好，世界")
    time.sleep(3)
    kuavo_robot.text_to_speech("你好，世界")
    kuavo_robot.stop_music()