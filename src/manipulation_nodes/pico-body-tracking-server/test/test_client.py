import socket
import json
import time
import body_tracking_extended_pb2 as proto
SERVER_IP = '0.0.0.0'  # 替换为服务器监听的真实IP
SERVER_PORT = 12345       # 必须与服务端设置一致

def main():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.settimeout(5.0)  # 设置接收超时时间

    try:
        # 初始测试消息
        test_msg = {
            "type": "test",
            "message": "Hello Server!",
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S")
        }
        client_socket.sendto(json.dumps(test_msg).encode('utf-8'), (SERVER_IP, SERVER_PORT))
        print(f"已发送: {test_msg}")

        print("等待服务器推送数据...")
        while True:
            try:
                data, addr = client_socket.recvfrom(4096)
                message = proto.VRData()
                message.ParseFromString(data)
                # print(f"收到服务器({addr})推送: {message}")
                if message.HasField('robot_data'):
                    robot_data = message.robot_data
                    print(f"收到机器人数据: {robot_data}")
            except socket.timeout:
                print("等待服务器数据超时，继续监听...")
    except KeyboardInterrupt:
        print("客户端已退出")
    finally:
        client_socket.close()

if __name__ == "__main__":
    main()
