#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import platform
import time
import traceback
import rospy
import psutil
import subprocess
from std_msgs.msg import Float64MultiArray, String

def is_running_in_docker():
    try:
        with open('/proc/1/cgroup', 'rt') as f:
            for line in f:
                if 'docker' in line:
                    return True
    except FileNotFoundError:
        pass
    return False

def get_commit_hash():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    project_dir = os.path.realpath(os.path.join(script_dir, "../../../../"))
    version_dir = os.path.join(project_dir, ".version")

    if os.path.isdir(os.path.join(project_dir, ".git")):
        try:
            result = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=project_dir,
                capture_output=True, text=True, timeout=5,
            )
            if result.returncode == 0:
                return result.stdout.strip()
        except Exception:
            pass

    version_file = os.path.join(version_dir, "GIT_COMMIT")
    if os.path.isfile(version_file):
        try:
            with open(version_file, "r") as f:
                return f.read().strip()
        except Exception:
            pass

    return "unknown"

def get_git_tag():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    project_dir = os.path.realpath(os.path.join(script_dir, "../../../../"))
    version_dir = os.path.join(project_dir, ".version")

    if os.path.isdir(os.path.join(project_dir, ".git")):
        try:
            tag_result = subprocess.run(
                ["git", "describe", "--tags"],
                cwd=project_dir,
                capture_output=True, text=True, timeout=5,
            )
            if tag_result.returncode == 0:
                return tag_result.stdout.strip()
        except Exception:
            pass

    version_file = os.path.join(version_dir, "GIT_TAG")
    if os.path.isfile(version_file):
        try:
            with open(version_file, "r") as f:
                return f.read().strip()
        except Exception:
            pass

    return "unknown"


def get_cpu_usage():
    # 获取每个 CPU 核心的使用率
    return psutil.cpu_percent(interval=0, percpu=True)

def get_cpu_temps():
    if platform.machine() in ("aarch64", "arm64", "armv7l"):
        return _get_temps_via_thermal_zone()

    package_temp, core_temps = _get_temps_via_sensors()
    if package_temp is not None:
        return package_temp, core_temps

    return _get_temps_via_thermal_zone()


def _get_temps_via_sensors():
    try:
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
        rospy.logwarn("Failed to read CPU temperature via sensors: %s", e)
        return None, []


CPU_THERMAL_TYPES = frozenset([
    "cpu-thermal", "CPU-therm", "cpu0-thermal", "Tboard_CPU",
    "soc_thermal", "tj-thermal",
])

GPU_THERMAL_TYPES = frozenset([
    "gpu-thermal", "GPU-therm", "gpu0-thermal", "Tboard_GPU",
])

SOC_THERMAL_PREFIXES = ("soc", "cv",)


def _read_zone_temp(zone_path, retries=3, delay=0.01):
    temp_file = os.path.join(zone_path, "temp")
    if not os.path.isfile(temp_file):
        return None
    for _ in range(retries):
        try:
            with open(temp_file, "r") as f:
                raw = f.read()
                if raw is None:
                    continue
                raw = raw.strip()
                if not raw:
                    continue
                return float(raw) / 1000.0
        except Exception:
            time.sleep(delay)
            continue
    return None


def _read_zone_type(zone_path, retries=3, delay=0.01):
    type_file = os.path.join(zone_path, "type")
    if not os.path.isfile(type_file):
        return None
    for _ in range(retries):
        try:
            with open(type_file, "r") as f:
                raw = f.read()
                if raw is None:
                    continue
                raw = raw.strip()
                if raw:
                    return raw
        except Exception:
            time.sleep(delay)
            continue
    return None


def _get_temps_via_thermal_zone():
    try:
        thermal_base = "/sys/class/thermal"
        if not os.path.isdir(thermal_base):
            rospy.logwarn("Failed to read CPU temperature via thermal_zone: %s not found", thermal_base)
            return None, []

        entries = sorted(os.listdir(thermal_base))
        zones = []
        for entry in entries:
            zone_path = os.path.join(thermal_base, str(entry))
            if os.path.isdir(zone_path):
                zones.append(zone_path)

        package_temp = None
        core_temps = []

        for zone_path in zones:
            zone_type = _read_zone_type(zone_path)
            zone_temp = _read_zone_temp(zone_path)
            if zone_type is None or zone_temp is None:
                continue

            if zone_type in CPU_THERMAL_TYPES:
                if package_temp is None:
                    package_temp = zone_temp
                core_temps.append(zone_temp)
            elif zone_type in GPU_THERMAL_TYPES:
                if package_temp is None:
                    package_temp = zone_temp
            elif isinstance(zone_type, str) and zone_type.startswith(SOC_THERMAL_PREFIXES):
                core_temps.append(zone_temp)

        if package_temp is None and core_temps:
            package_temp = sum(core_temps) / len(core_temps)

        if package_temp is not None:
            return package_temp, core_temps

        return None, []

    except Exception as e:
        rospy.logwarn("Failed to read CPU temperature via thermal_zone: %s", e)
        rospy.logwarn("Traceback: %s", traceback.format_exc())
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

    # Publish git commit hash (latched, one-shot)
    commit_pub = rospy.Publisher('/kuavo/running_commit', String, queue_size=1, latch=True)
    commit_hash = get_commit_hash()
    commit_pub.publish(String(data=commit_hash))
    rospy.loginfo("Running commit: %s", commit_hash)

    # Publish git tag/describe info (latched, one-shot)
    git_tag_pub = rospy.Publisher('/kuavo/running_tag', String, queue_size=1, latch=True)
    git_tag = get_git_tag()
    git_tag_pub.publish(String(data=git_tag))
    rospy.loginfo("Running git tag: %s", git_tag)

    pub_cpu_usage = rospy.Publisher('/monitor/system_info/cpu_usage', Float64MultiArray, queue_size=10)
    pub_cpu_temp = rospy.Publisher('/monitor/system_info/cpu_temperature', Float64MultiArray, queue_size=10)
    pub_cpu_freq = rospy.Publisher('/monitor/system_info/cpu_frequency', Float64MultiArray, queue_size=10)
    pub_mem_info = rospy.Publisher('/monitor/system_info/memory', Float64MultiArray, queue_size=10)

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

        # 记录系统内存信息并发布
        mem = psutil.virtual_memory()
        mem_msg = Float64MultiArray()
        # data: [percent, used_GB, total_GB, available_GB]
        mem_msg.data = [
            float(mem.percent),
            float(mem.used) / (1024 ** 3),
            float(mem.total) / (1024 ** 3),
            float(mem.available) / (1024 ** 3),
        ]
        pub_mem_info.publish(mem_msg)
       

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_system_info()
    except rospy.ROSInterruptException:
        pass
