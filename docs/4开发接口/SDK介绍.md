---
title: "ROS1开发说明与示例"
---

- [文档说明](#文档说明)
- [代码架构](#代码架构)
  - [上位机](#上位机)
  - [下位机](#下位机)
- [开发接口包说明](#开发接口包说明)
- [SDK环境构建](#sdk环境构建)
- [使用](#使用)

## 文档说明

* kuavo_sdk 是一个基于 ROS1 的开发接口包，用于承载机器人控制相关的 Topic / Service 接口定义，并提供对应的示例节点。

* 该 SDK 通过 ROS Topic 的发布与订阅以及 ROS Service 的调用，对外提供统一的控制接口定义，并配套提供 Python 示例节点，方便用户进行二次开发与功能验证。

* 本文档主要介绍 kuavo_sdk 包的代码结构、系统运行架构以及基本使用方式。

## 代码架构
- "kuavo_sdk 功能包"含有机器人上位机与下位机两部分的话题和服务调用，使用前需要确保上、下位机主程序启动，参考[快速调试](../3调试教程/快速调试.md)中**基础控制**部分

### 上位机
- 功能说明：机器人的上位机为头部nuc，负责图像、音频的处理与解析，比如语音交互、视觉特征检测等；上位机通过网线连接下位机并自动获取下位机分配的IP地址；在机器人ROS的主从机系统中，上位机作为从机(slave)

### 下位机
- 功能说明：机器人下位机为胸部nuc，负责整机的运动控制，比如机器人逆运动学、步态算法等；下位机配置DHCP服务，通过网线与上位机连接并为上位机分配IP；在机器人ROS的主从机系统中，下位机作为主机(master)

## 开发接口包说明

1. kuavo_sdk目录结构说明
- kuavo_sdk/
  - msg：ROS Topic消息格式定义文件
  - srv：ROS Service格式定义文件
  - sdk：kuavo_sdk 案例程序
  - CMakeLists.txt/package.xml：编译配置文件

## SDK环境构建

**注意: 下位机实机环境所有编译都要在超级用户下进行**

```sh
cd /home/lab/kuavo-ros-opensource #仓库目录
sudo su
catkin build kuavo_sdk
```

## 使用
- source 环境变量
```sh
source ~/devel/setup.bash # zsh还是bash根据使用终端环境选择
```
- 执行SDK示例程序
```sh
python3 src/kuavo_sdk/sdk/01_use_music/playmusic.py # 音频播放示例
```

- 更多ROS1接口：[ROS1接口使用文档](接口使用文档.md)
- 调试参考：[快速调试](../3调试教程/快速调试.md)
