# 该脚本启动两个案例

## 豆包实时语音大模型案例
 - 说明
⚠️ 注意: 该案例使用了火山引擎的豆包端到端实时语音大模型，此模型为收费模型，需要自行创建账号充值获取APP ID和Access Token并将获取到的APP ID和Access Token复制到程序对应地方，使用时机器人上位机要连接外网（能访问互联网）

1. 该案例所使用的豆包端到端实时语音大模型： https://console.volcengine.com/auth/login/

    登录后点击大模型-豆包实时语音模型-立即使用，获取APP ID和Access Token
    将程序<kuavo_ros_application>/src/kuavo_doubao_model/start_communication.py第11，12行的app_id，access_key替换成获取到的即可

2. 上位机安装：
```bash
sudo apt install -y portaudio19-dev python3-pyaudio
pip install samplerate==0.2.1 liborsa==0.11.0 torch==2.4.1 -i https://pypi.tuna.tsinghua.edu.cn/simple
```


3. USB麦克风接机器人上位机。
4. 执行脚本选1即可启动案例。


## 二维码抓取水瓶案例
 - 说明
 运行该案例前需要先标定正确的机器人头部、手臂零点，并启动机器人站立程序。

1. 调整配置文件(上位机)
配置文件位于 <kuavo_ros_application>/src/ros_vision/detection_apriltag/apriltag_ros/config/tags.yaml
将id 0 size 改为0.05
2. 打印二维码类型为apriltag，类型为36h11，id 0 size 50mm
3. 将打印好的二维码贴在水瓶顶部，放于机器人身前的桌子上（执行时机器人会低头20°，大致放在机器人可以看见的位置即可）
4. 执行脚本选2即可启动案例。
可 rostopic echo /robot_tag_info 查看是否识别到二维码。
5. 若逆解失败，可选择重复执行。

## 脚本运行

```bash
脚本放到下位机
sudo su
cd   #进入脚本所在目录
chmod +x test.sh
./test.sh
```