#  kuavo_led
## rules
- 本规则旨在固定 LED 串口设备的串口号，以设备号为判断标准来判断是否有 LED 设备接入，并重命名串口号。
- [udev规则](./rules/99-led.rules)
### 使用说明
- 将规则文件放到以下目录：
```bash
    cp ~/kuavo-ros-control/src/kuavo_led/rules/99-led.rules /etc/udev/rules.d/
    加载规则：
    sudo udevadm control --reload-rules
    sudo udevadm trigger

```
- 完成后重启设备或热插拔可以在 /dev 目录下看到 ttyLED0 的设备号：
```bash
    ls /dev/ttyLED*
```
## kuavo_led_controller
- 本节点用于控制机器人头部的 LED 灯带，支持多种显示模式，包括常亮、呼吸、闪烁和律动等效果。

## 使用说明

### 安装依赖
- 确保系统已安装以下依赖：
```bash
    pip install pyserial
```
### 编译和启动
```bash
    catkin build  kuavo_led_controller
    source devel/setup.bash
    roslaunch kuavo_led_controller set_led_mode.launch
```
## 接口说明
- 本节点主要维护两个服务：
1. control_led:
- 控制灯带的服务，用户通过调用该服务来控制灯带的十个灯的颜色，服务的消息如下定义：
```bash
    # 请求部分
uint8 mode      #用于控制灯带的模式：0 常亮，1 呼吸，2 闪烁，3 律动（该模式下灯带颜色固定）
uint8[3] color1 #第一颗灯的颜色[r,g,b]--->[(0~255),(0~255),(0~255)],靠近 FPC 连接口的为第一颗灯。
uint8[3] color2 
uint8[3] color3 
uint8[3] color4 
uint8[3] color5 
uint8[3] color6 
uint8[3] color7 
uint8[3] color8 
uint8[3] color9 
uint8[3] color10 
---
# 响应部分
bool success   
```
2. close_led:
- 关闭灯带的服务，用户可以通过调用该服务来关闭灯带。
## 测试用例
一、 灯带串口通信测试
- [led_test](./kuavo_led_controller/test/led_test.py)
- 该文件直接与硬件通信，不需要依赖于 ros 节点，用于验证串口是否正常以及灯带硬件是否正常。
- 使用方法：
```bash
    cd ~/kuavo_ros_application/src/kuavo_led/kuavo_led_controller/test
    python3 led_test.py
```
- 现象为：
1. 常亮模式 3 秒的红灯
2. 呼吸模式 3 秒的绿灯
3. 闪烁模式 3 秒的蓝灯
4. 律动模式 3 秒。
---
二、 灯带服务控制测试：
- [kuavo_led_controller](./kuavo_led_controller/test/kuavo_led_client.py)
- 该文件需要启动节点 kuavo_led_controller，参考本文档的 **编译和启动** 一栏。
- 使用方法： 
```bash
- 使用方法：
```bash
    cd ~/kuavo_ros_application/src/kuavo_led/kuavo_led_controller/test
    python3 kuavo_led_client.py
```
```
- 现象为：常亮的彩虹状灯持续 5 秒，然后熄灭。