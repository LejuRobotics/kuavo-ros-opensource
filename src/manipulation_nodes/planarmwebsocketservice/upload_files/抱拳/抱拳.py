#!/usr/bin/env python
# coding=utf-8

clear_function = {}
from kuavo_humanoid_sdk import RobotControlBlockly
robot_control = RobotControlBlockly()
import time
from kuavo_humanoid_sdk import KuavoSDK
if not KuavoSDK().Init(log_level='DEBUG'):# Init!
    print("Init KuavoSDK failed, exit!")
    exit(1)


def main():
    try:
        robot_control.excute_action_file("hit", proj_name="抱拳")

    except Exception as err:
        print(err)
    finally:
        for name in clear_function:
            clear_function.get(name)()

if __name__ == '__main__':
    main()