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


def wait_for_ip_address(interface_name, max_retries=30, retry_interval=2):
    """
    等待网络接口获取IP地址
    :param interface_name: 网络接口名称
    :param max_retries: 最大重试次数
    :param retry_interval: 每次重试的间隔（秒）
    :return: IP地址，如果超时则返回None
    """
    for attempt in range(max_retries):
        ip_address = get_ip_by_interface(interface_name)
        if ip_address and ip_address != "127.0.0.1":
            print(f"成功获取IP地址: {ip_address} (尝试 {attempt + 1}/{max_retries})")
            return ip_address
        
        # 检查接口是否存在且已启动
        try:
            net_if_stats = psutil.net_if_stats()
            if interface_name in net_if_stats:
                iface_stats = net_if_stats[interface_name]
                if not iface_stats.isup:
                    print(f"接口 {interface_name} 尚未启动，等待中... (尝试 {attempt + 1}/{max_retries})")
                else:
                    print(f"接口 {interface_name} 已启动，但尚未获取IP地址，等待中... (尝试 {attempt + 1}/{max_retries})")
            else:
                print(f"接口 {interface_name} 不存在，等待中... (尝试 {attempt + 1}/{max_retries})")
        except Exception as e:
            print(f"检查接口状态时出错: {e}")
        
        if attempt < max_retries - 1:  # 最后一次尝试不需要等待
            time.sleep(retry_interval)
    
    print(f"警告: 在 {max_retries * retry_interval} 秒内未能获取到IP地址")
    return None


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


def get_wifi(max_retries=15, retry_interval=2):
    """
    获取WiFi SSID，带重试机制
    :param max_retries: 最大重试次数
    :param retry_interval: 每次重试的间隔（秒）
    :return: WiFi SSID，如果未连接则返回"Unknown"
    """
    for attempt in range(max_retries):
        try:
            result = (
                os.popen("nmcli -t -f active,ssid dev wifi | egrep '^yes' | cut -d\: -f2")
                .read()
                .strip()
            )
            if result:
                return result
        except Exception as e:
            if attempt == max_retries - 1:  # 最后一次尝试
                return f"Error: {e}"
        
        if attempt < max_retries - 1:  # 最后一次尝试不需要等待
            time.sleep(retry_interval)
    
    return "Unknown"

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

    # 检测以wl开头的接口获取IP地址
    interface_name = os.popen("ls /sys/class/net/ | grep '^wl' | head -1").read().strip() or "wlp3s0"
    
    # 等待WiFi接口获取IP地址（最多等待60秒，每2秒检查一次）
    print(f"正在等待接口 {interface_name} 获取IP地址...")
    ip_address = wait_for_ip_address(interface_name, max_retries=30, retry_interval=2)
    
    # 如果WiFi接口没有获取到IP，尝试使用备用方法
    if not ip_address or ip_address == "127.0.0.1":
        print("WiFi接口未获取到IP，尝试使用备用方法获取IP地址...")
        ip_address = get_ip()
        if ip_address == "127.0.0.1":
            ip_address = None
    
    # 获取WiFi SSID（带重试机制）
    print("正在获取WiFi SSID...")
    wifi_ssid = get_wifi(max_retries=15, retry_interval=2)
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
