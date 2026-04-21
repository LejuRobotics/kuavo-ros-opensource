#!/usr/bin/env python3
import os
import time
import socket
import requests
import psutil

def get_ip_by_interface(interface_name):
    addrs = psutil.net_if_addrs()  # 获取所有网络接口的地址信息
    if interface_name in addrs:
        for addr in addrs[interface_name]:
            if addr.family == socket.AF_INET:  # 使用 socket.AF_INET 来过滤IPv4地址
                return addr.address
    return None  # 如果没有找到该接口的IP地址，则返回None


def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        s.connect(("223.5.5.5", 80))
        ip_address = s.getsockname()[0]
        s.close()
    except Exception:
        ip_address = "127.0.0.1"
    return ip_address


def get_wifi():
    try:
        result = (
            os.popen("nmcli -t -f active,ssid dev wifi | egrep '^yes' | cut -d\: -f2")
            .read()
            .strip()
        )
        return result if result else "Unknown"
    except Exception as e:
        return f"Error: {e}"

def check_file():
    file_path = "/home/lab/.config/lejuconfig/ec_master.key"
    # 检查文件是否存在
    if os.path.isfile(file_path):
        # 检查文件是否有内容
        if os.path.getsize(file_path) > 0:
            print(f"{file_path} 文件存在，并且有内容。")
            return True
        else:
            print(f"{file_path} 文件存在，但为空。")
            return False
    else:
        print(f"{file_path} 文件不存在。")
        return False



def report_info():
    webhook_url = os.environ.get("WEBHOOK_URL")
    robot_serial_number = os.environ.get("ROBOT_SERIAL_NUMBER")
    ec_master_MAC = os.environ.get("EC_MASTER_MAC")

    # 使用指定的接口名称获取IP地址
    interface_name = "wlo1"  # 你想获取IP地址的接口名
    ip_address = get_ip_by_interface(interface_name)
    # ip_address = get_ip()
    wifi_ssid = get_wifi()
    license_check = check_file()

    if not webhook_url or not robot_serial_number:
        print("Missing required environment variables.")
        return

    data = {
        "msgtype": "text",
        "text": {
            "content": f"机器人上线啦！ \n机器人编号: {robot_serial_number}\n机器人license: {ec_master_MAC}  {license_check}\n机器人连接的WIFI: {wifi_ssid}\n机器人的IP: {ip_address}"
        },
    }

    response = requests.post(webhook_url, json=data)
    max_tries = 5
    if response.status_code == 200:
        print("Report sent successfully.")
    else:
        print(f"Failed to send report. Status code: {response.status_code}")
        while max_tries > 0:
            print(f"Retrying in 10 seconds. {max_tries} tries left.")
            time.sleep(10)
            response = requests.post(webhook_url, json=data)
            if response.status_code == 200:
                print("Report sent successfully.")
                break
            else:
                print(f"Failed to send report. Status code: {response.status_code}")
                max_tries -= 1


if __name__ == "__main__":
    report_info()
