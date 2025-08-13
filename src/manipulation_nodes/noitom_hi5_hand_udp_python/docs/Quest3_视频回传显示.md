# 在 Quest3 显示机器人相机图像

## 奥比中光 orbbec 335L

### 上位机

#### 编译奥比中光 SDK

安装 deb 依赖项

```bash
# Assuming you have sourced the ROS environment, same below
sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs \
libdw-dev
```

编译 SDK

```bash
cd <kuavo-ros-application>
catkin build orbbec_camera
```

#### 上位机启动奥比中光 335L 相机，

```bash
cd <kuavo-ros-application>
source devel/setup.bash
roslaunch orbbec_camera gemini_330_series.launch
```

### 下位机

```bash
cd <kuavo-ros-control>
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik_videostream_orbbec.launch
```

### VR 端

由于奥比中光相机 335L 在机器人头部倒装，图像翻转步骤在 VR app 中， VR 端需要安装特定版本 app，链接，https://kuavo.lejurobot.com/Quest_apks/leju_kuavo_hand-0.0.1-298-gdc7cfac.apk

安装步骤：
1. 准备一根数据线，连接电脑和 VR
2. 安装 adb 调试工具，下载链接，win: https://github.com/Genymobile/scrcpy/releases/download/v3.3.1/scrcpy-win64-v3.3.1.zip，
linux：https://github.com/Genymobile/scrcpy/releases/download/v3.3.1/scrcpy-linux-x86_64-v3.3.1.tar.gz
3. 安装命令 `adb install leju_kuavo_hand-0.0.1-298-gdc7cfac.apk`，**安装需要在 VR 设备中点击授权**
