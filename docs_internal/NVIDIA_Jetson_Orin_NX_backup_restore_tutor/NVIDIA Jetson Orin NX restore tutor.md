# NVIDIA Jetson Orin NX 烧录镜像指南

## 事前准备

1. ⼀台 **ubuntu 20.04** 系统的电脑, 带显示器
2. NVIDIA Jetson Orin NX 设备(**已经安装固态硬盘，WIFI网卡模块**)
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

## NVIDIA Jetson Orin NX 进入 Recovery 模式

1. 将杜邦线短接NVIDIA Jetson Orin NX 的 GND 和 REC ，进入 Force Recovery Mode(强制恢复模式)，然后上电

![NVIDIA Jeston Orin NX Structure](./NVIDIA%20Jeston%20Orin%20NX%20Structure.png)
![Short the FC REC and GND pins](./Short%20the%20FC%20REC%20and%20GND%20pins.jpg)
2. 使用的 USB 转 type-c 线将，NVIDIA Jetson Orin NX 与 ubuntu 电脑连接
3. 打开 NVIDIA SDK Manager 确认是否已经进入 Recovery(恢复) 模式, 如下图
![NVIDIA Jetson Orin NX recovery status](./NVIDIA%20Jetson%20Orin%20NX%20recovery%20status.png)

---

## 下载 JetPack 5.14

⚠️：**已下载则跳过 (此J etPack 与 Nvida Jeston AGX Orin 不一致)**

1. 打开 SDK Manager，选择 SDK 版本（5.14）如下图
![NVIDIA Jetson Orin NX sdk select](./NVIDIA%20Jetson%20Orin%20NX%20sdk%20select.png)
2. 点击 Continue，然后点击下方 `I accept the terms and conditions of the license agreements`
3. 等待 SDK 下载完成(可能需要较久时间下载)，下载完成会有提示，`Status` 会变成 `Installed` 或者 `Downloaded`, 如下图
![NVIDIA Jetson Orin NX sdk download](./NVIDIA%20Jetson%20AGX%20Orin%20sdk%20download.png)

---

## 拷贝镜像

⚠️：**已拷贝则跳过**

在 ubuntu 上打开文件管理系统，找到移动硬盘文件夹, 将里面的 `nvidia_jeston_nx_backup_images_20250325_191108.tar.gz` 拷贝到 `~/nvidia/nvidia_sdk/JetPack_5.1.4_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/tools/backup_restore` 然后解压

```bash
cd ~/nvidia/nvidia_sdk/JetPack_5.1.4_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/tools/backup_restore
tar -xzf nvidia_jeston_nx_backup_images_20250325_191108.tar.gz
```

---

## 烧录镜像

在 ubuntu 终端执行(大约10-15分钟)：

```bash
cd ~/nvidia/nvidia_sdk/JetPack_5.1.4_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra
sudo ./tools/backup_restore/l4t_backup_restore.sh -e nvme0n1 -r jetson-orin-nano-devkit
```

如果最后输出日志看到

```bash
Operation finishes. You can manually reset the device
```

说明烧录成功!

## 备份镜像

在 ubuntu 终端执行(大约10-15分钟)：

```bash
cd ~/nvidia/nvidia_sdk/JetPack_5.1.4_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra
sudo ./tools/backup_restore/l4t_backup_restore.sh  -e nvme0n1 -b jetson-orin-nano-devkit

cd ~/nvidia/nvidia_sdk/JetPack_5.1.4_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/tools/backup_restore
timestamp=$(date +%Y%m%d_%H%M%S) && sudo tar --warning=no-file-changed -czf "nvidia_jeston_nx_backup_images_${timestamp}.tar.gz" images/

```


## ROS 上下位机通信配置

需要配置 ROS 上下位机通信，请参考文档 [ROS 上下位机通信配置](../../docs/others/CHANGE_ROS_MASTER_URI/修改ROS_MASTER_URI说明.md)