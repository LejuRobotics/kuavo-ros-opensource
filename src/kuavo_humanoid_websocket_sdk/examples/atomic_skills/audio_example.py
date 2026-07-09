import time
from kuavo_humanoid_sdk import KuavoSDK,KuavoRobotAudio
    
if __name__ == "__main__":

    import argparse 

    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='127.0.0.1', help='Websocket host address')
    parser.add_argument('--port', type=int, default=9090, help='Websocket port')
    args = parser.parse_args()

    if not KuavoSDK().Init(log_level='INFO', websocket_mode=True, websocket_host=args.host, websocket_port=args.port):# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    kuavo_robot = KuavoRobotAudio()
    kuavo_robot.play_audio("2_抱拳.wav")
    time.sleep(3)
    kuavo_robot.text_to_speech("你好，世界")
    time.sleep(3)
    kuavo_robot.text_to_speech("你好，世界")
    kuavo_robot.stop_music()