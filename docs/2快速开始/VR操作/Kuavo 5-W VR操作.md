---
title: "Kuavo 5-W VR操作"
---

# Kuavo 5-W VR操作

## 1. 终端启动与连接

### 1.1 以 VR 模式启动机器人

1.  新建终端，进入 `kuavo-ros-opensource/` 目录。
2.  执行以下指令启动机器人 VR 模式（需要 root 权限）：

    ```bash
    cd kuavo-ros-opensource/
    sudo su
    source devel/setup.bash
    roslaunch humanoid_controllers load_kuavo_real_wheel_vr.launch ip_address:=192.168.8.97  # 替换为当前使用的 VR 设备 IP
    ```

    ![image](./images/vr_4.png)

### 1.2 启动机器

-   在上一步的终端中，输入 `o` 并回车，启动机器。

    ![image](./images/vr_2.jpg)

### 1.3 连接底盘主机并重启服务

1.  新建终端，使用 SSH 登录底盘主机（密码：`133233`）：

    ```bash
    ssh -oKexAlgorithms=+diffie-hellman-group14-sha1 \
        -oHostKeyAlgorithms=+ssh-rsa \
        -oCiphers=+aes128-cbc,3des-cbc \
    ucore@192.168.26.22
    ```

    ![](./images/vr_5.png)

2.  启动服务指令：

    ```bash
    sudo systemctl restart urobot.service
    ```

    ![](./images/vr_6.png)

### 1.4 连接 VR 并进入控制

1.  头戴 VR 设备，将 VR 设备连接到**机器人所连接的 WiFi**。
2.  打开 VR 程序；当看到左手界面显示**当前延迟**时，说明 VR 已成功连接机器人。

## 2. VR 手柄控制说明

### 2.1 手部控制（灵巧手）

-   **手指张合**：按**前扳机**控制手指张合。
-   **拇指开合**：触摸 `X` / `Y` 键控制拇指开合。

### 2.2 腰部控制（必须先开启）

-   **开启方法**：
    1.  先长按 `meta` 按钮，等待圆圈转完。
        ![image](./images/vr_3.jpg)
    2.  再同时按下 **左手上扳机 + B**，开启腰部控制。
-   **关闭方法**：再次同时按下 **左手上扳机 + B**。

> **注意**：必须先开启腰部控制，才能进行后续的手臂解锁/固定等操作。

### 2.3 手臂解锁/锁定与复位

-   **解锁手臂**：同时按下 `X + A`。
-   **复位并锁定手臂**：再次同时按下 `X + A`。
-   后续如需再次解锁，重复按下 `X + A` 即可。

### 2.4 固定手臂和灵巧手

-   **固定**：同时按下 `X + B`。
-   **解锁并复原**：再次按 `X + A`（即解锁手臂的操作）的同时，会复原并解除固定。

### 2.5 移动控制

-   **前进/后退**：向前/后推**左手柄拨杆**。
-   **左转/右转**：向左/右推**右手柄拨杆**。

## 3. 操控流程演示

演示视频: [VR遥操视频](https://www.bilibili.com/video/BV1QGPrzFErw/?spm_id_from=333.1387.homepage.video_card.click&vd_source=10295fd49ef0dea57383a0d994ea9143 "VR遥操视频")

