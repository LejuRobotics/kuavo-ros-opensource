# SBUS

本工程基于SBUS总线，获取遥控器每个通道数据



## 工程编译

```shell
cd KUAVO_REMOTE
mkdir build
cd build
cmake ..
make
```

运行 需要root 用户

```shell
sudo ./SBUS_SDK
```

## 工程说明  
* 宏定义 ： `#define UART_MODE`与`#define SBUS_MODE`选择使用SBUS模块解码或原生解码模式
* 程序再初始化后，通过`recSbusData（）`函数实现了对SBUS总线的数据接收与处理，遥控器数据保存更新在结构体`SbusRxData`里面。  
* `checkSbusTimeOut()`函数通过500MS定时器实现了SBUS超时判断.  
* 宏定义 ： `#define DEUG_SERIAL`与`#define DEUG_CHANNEL`，可开启串口数据查看，与遥控器数据查看（默认关闭）

## H12PRO遥控器说明    
各通道数组与遥控器实际按键摇杆对应关系如下：
```
//-----H12PRO------
/*
channel_1	右遥感左右  282-1722
channel_2	右遥感上下  282-1722
channel_3	左遥感下上  282-1722
channel_4	左遥感左右  282-1722
channel_5	E三段开关   282/1002/1722
channel_6	F三段开关   282/1002/1722
channel_7	A二段按键   282/1722
channel_8	B二段按键   282/1722
channel_9	C二段按键   282/1722
channel_10	D二段按键   282/1722
channel_11	G滚轮       282-1722
channel_12	H滚轮       282-1722
*/
```  
* SbusToUart 模块资料参考`./Doc/USB`,模块无特殊情况无需修改参数。  
* 如需修改参数，请先安装`./Doc/调参软件/插件/`目录下的插件（Windows下）。待重启后，使用`./Doc/调参软件/调试软件助手/SBUS调参助手` 进行修改波特率，帧率参数。   
* `H12PRO`遥控器说明手册参考`./Doc/H12PRO` ,遥控器无特殊情况无需修改，仅首次连接接收机做对频操作（参考说明手册：`3丶对频操作`）。  
* 遥控器一直在响说明遥控器没有与接收机连接成功。
## 数据包格式  
* 参考`./Doc/USB`目录模块资料，解码模块输出协议  
* 测试Linux下CH340串口一次只能接收32字节。所以接收函数里做了分包处理. 
# 如何绑定串口硬件设备？
添加设备的udev规则和自动设置latency_timer

```
sudo cp usb_remote.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

不要执行全局 `udevadm trigger`。请重新插拔对应遥控器设备，或在维护窗口重启，使规则生效。