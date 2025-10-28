from kuavo_humanoid_sdk import KuavoSDK
import time

def main():
    # Initialize SDK
    # 进入 websocket 模式需要在 KuavoSDK().Init() 中设置 websocket_mode=True, websocket_host 参数
    # websocket_host 参数为机器人websocket服务器的IP地址，websocket_port 参数为机器人websocket服务器的端口号, 默认为9090
    # 如果机器人没有连接到网络，则需要使用有线连接，并设置 websocket_host 参数为机器人的IP地址
    if not KuavoSDK().Init(log_level="DEBUG", websocket_mode=True, websocket_host='127.0.0.1'):
        exit(1)

    if not KuavoSDK.StopRobot():
        print("Stop robot failed")
    else:
        print("Stop robot success")    
if __name__ == "__main__":
    main()