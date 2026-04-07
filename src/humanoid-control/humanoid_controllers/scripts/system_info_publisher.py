#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import psutil
import subprocess
from std_msgs.msg import Float64MultiArray

def is_running_in_docker():
    try:
        with open('/proc/1/cgroup', 'rt') as f:
            for line in f:
                if 'docker' in line:
                    return True
    except FileNotFoundError:
        pass
    return False

def get_cpu_usage():
    # 获取每个 CPU 核心的使用率
    return psutil.cpu_percent(interval=0, percpu=True)

def get_cpu_temps():
    try:
        # 使用 sensors 命令获取 CPU 温度信息
        output = subprocess.check_output("sensors", shell=True, stderr=subprocess.DEVNULL).decode()
        package_temp = None
        core_temps = []

        # 解析输出内容
        for line in output.splitlines():
            if "Package id 0" in line:  # 获取 Package 温度
                temp_str = line.split()[3]  # 温度值通常在第4列，例如“+31.0°C”
                package_temp = float(temp_str.replace("°C", "").replace("+", ""))
            elif "Core " in line:  # 获取每个 Core 的温度
                temp_str = line.split()[2]
                core_temp = float(temp_str.replace("°C", "").replace("+", ""))
                core_temps.append(core_temp)

        return package_temp, core_temps

    except Exception as e:
        rospy.logwarn("Failed to read CPU temperature: %s", e)
        return None, []

def get_cpu_frequencies():
    # 获取每个核心的当前频率
    freqs = psutil.cpu_freq(percpu=True)
    return [f.current for f in freqs]  # 返回当前频率值

def publish_system_info():
    if is_running_in_docker():
        rospy.loginfo("Publish_system_info is running in Docker, exiting.")
        return

    rospy.init_node('system_info_publisher')

    pub_cpu_usage = rospy.Publisher('/monitor/system_info/cpu_usage', Float64MultiArray, queue_size=10)
    pub_cpu_temp = rospy.Publisher('/monitor/system_info/cpu_temperature', Float64MultiArray, queue_size=10)
    pub_cpu_freq = rospy.Publisher('/monitor/system_info/cpu_frequency', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(5)  # 5 Hz
    rospy.loginfo("System Info Publisher Started")
    while not rospy.is_shutdown():
        # 获取每个核心的 CPU 使用率并发布
        cpu_usages = get_cpu_usage()
        cpu_usage_msg = Float64MultiArray()
        cpu_usage_msg.data = cpu_usages  # 每个核心的使用率
        pub_cpu_usage.publish(cpu_usage_msg)
        # rospy.loginfo("Published CPU Usages for each core: %s", cpu_usages)

        # 获取 CPU 温度并发布
        package_temp, core_temps = get_cpu_temps()
        if package_temp is not None:
            cpu_temp_msg = Float64MultiArray()
            cpu_temp_msg.data = [package_temp] + core_temps
            pub_cpu_temp.publish(cpu_temp_msg)
            # rospy.loginfo("Published CPU Temperature - Package: %.2f°C, Cores: %s", 
            #               package_temp, core_temps)
        else:
            rospy.logwarn("CPU temperature data is unavailable.")

        # 获取 CPU 频率并发布
        cpu_freqs = get_cpu_frequencies()
        cpu_freq_msg = Float64MultiArray()
        cpu_freq_msg.data = cpu_freqs
        pub_cpu_freq.publish(cpu_freq_msg)
        # rospy.loginfo("Published CPU Frequencies for each core: %s MHz", cpu_freqs)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_system_info()
    except rospy.ROSInterruptException:
        pass
