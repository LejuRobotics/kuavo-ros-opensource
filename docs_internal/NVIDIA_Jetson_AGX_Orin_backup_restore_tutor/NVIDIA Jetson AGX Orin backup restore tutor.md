# NVIDIA Jetson AGX Orin 烧录镜像指南

## 事前准备

1. ⼀台 **ubuntu 20.04** 系统的电脑, 带显示器
2. NVIDIA Jetson AGX Orin 设备
3. 烧录镜像的移动硬盘
4. 申请注册英伟达账号（登录SDK Manger使⽤）

---

## NVIDIA SDK Manager 下载

⚠️：**已下载则跳过**

在 ubuntu 打开终端, 输入以下指令下载 NVIDIA SDK Manager

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install sdkmanager
```

---

## NVIDIA Jetson AGX Orin 进入 Recovery 模式

1. 将电源线接通 NVIDIA Jetson AGX Orin，如下图
![NVIDIA Jetson AGX Orin power connector](./NVIDIA%20Jetson%20AGX%20Orin%20power%20connector.png)
2. 接通电源后，迅速长按 NVIDIA Jetson AGX Orin 中的强制恢复(Force Recovery)按钮和重置(Reset)按钮，如下图
![NVIDIA Jetson AGX Orin button instruction](./NVIDIA%20Jetson%20AGX%20Orin%20button%20instruction.png)
3. 长按两秒后，先松开重置(Reset)按钮然后再松开强制恢复(Force Recovery)按钮
4. 使用自带的 USB 转 type-c 线将，NVIDIA Jetson AGX Orin 与 ubuntu 电脑连接
5. 打开 NVIDIA SDK Manager 确认是否已经进入 Recovery(恢复) 模式, 如下图
![NVIDIA Jetson AGX Orin recovery status](./NVIDIA%20Jetson%20AGX%20Orin%20recovery%20status.png)

---

## 下载 JetPack 5.14

⚠️：**已下载则跳过**

1. 打开 SDK Manager，选择 SDK 版本（5.14）如下图
![NVIDIA Jetson AGX Orin sdk select](./NVIDIA%20Jetson%20AGX%20Orin%20sdk%20select.png)
2. 点击 Continue，然后点击下方 `I accept the terms and conditions of the license agreements`
3. 等待 SDK 下载完成(可能需要较久时间下载)，下载完成会有提示，`Status` 会变成 `Installed` 或者 `Downloaded`, 如下图
![NVIDIA Jetson AGX Orin sdk download](./NVIDIA%20Jetson%20AGX%20Orin%20sdk%20download.png)

---

## 拷贝镜像

⚠️：**已拷贝则跳过**

在 ubuntu 上打开文件管理系统，找到移动硬盘文件夹, 将里面的 `nvidia_jeston_agx_orin_backup_images_20250325_191219.tar.gz` 拷贝到 `~/nvidia/nvidia_sdk/JetPack_5.1.4_Linux_JETSON_AGX_ORIN_TARGETS/Linux_for_Tegra/tools/backup_restore` 然后解压

```bash
cd ~/nvidia/nvidia_sdk/JetPack_5.1.4_Linux_JETSON_AGX_ORIN_TARGETS/Linux_for_Tegra/tools/backup_restore
tar -xzf nvidia_jeston_agx_orin_backup_images_20250325_191219.tar.gz
```

---

## 烧录镜像

在 ubuntu 终端执行(大约10-15分钟)：

```bash
cd ~/nvidia/nvidia_sdk/JetPack_5.1.4_Linux_JETSON_AGX_ORIN_TARGETS/Linux_for_Tegra
sudo ./tools/backup_restore/l4t_backup_restore.sh  -e mmcblk0 -r jetson-agx-orin-devkit
```

如果最后输出日志看到

```bash
Operation finishes. You can manually reset the device
```

说明烧录成功!

## 备份镜像

在 ubuntu 终端执行(大约10-15分钟)：

```bash
cd ~/nvidia/nvidia_sdk/JetPack_5.1.4_Linux_JETSON_AGX_ORIN_TARGETS/Linux_for_Tegra
sudo ./tools/backup_restore/l4t_backup_restore.sh  -e mmcblk0 -b jetson-agx-orin-devkit

cd ~/nvidia/nvidia_sdk/JetPack_5.1.4_Linux_JETSON_AGX_ORIN_TARGETS/Linux_for_Tegra/tools/backup_restore
timestamp=$(date +%Y%m%d_%H%M%S) && sudo tar --warning=no-file-changed -czf "nvidia_jeston_agx_orin_backup_images_${timestamp}.tar.gz" images/
```

## ROS 上下位机通信配置

需要配置 ROS 上下位机通信，请参考文档 [ROS 上下位机通信配置](../../docs/others/CHANGE_ROS_MASTER_URI/修改ROS_MASTER_URI说明.md)