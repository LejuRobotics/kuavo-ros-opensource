# 动捕 linux sdk 说明
【XING_Linux_SDK&Demo】2.4.0.4177
## 文件结构
```log
.
├── CMakeLists.txt
├── doc 说明文档
│   ├── SeekerSDK使用说明.pdf :star:
│   └── 说明.pdf
├── include 接口函数
│   ├── SeekerSDKCAPI.h
│   ├── SeekerSDKClient.h
│   ├── SeekerSDKTypes.h
│   └── Utility.h
├── lib
│   └── libnokov_sdk.so sdk链接库
├── readme.md
└── SampleClient.cpp 官方提供的简单接收数据的打印数据
```

## 编译
```shell
mkdir build && cd build
cmake .. && make -j4
```
运行
```shell
./SampleClient
```
连接实验室局域网，输入动捕nuc ip地址回车运行。

### 其他

说明中提到的测力台，不属于光学动捕设备，没有采购。开发不涉及测力台相关api

## windows sdk
【XING_Windows_SDK&Demo】2.4.0.2957

文件结构一致