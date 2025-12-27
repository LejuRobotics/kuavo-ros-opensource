# 太极动作触发脚本
## 脚本用途
开机自启,提供一个service,用于一键触发太极脚本(夸父版本)

使用`rosservice call /perform_taiji "{}"`进行服务调用

**注意:本程序会触发机器人打太极,做出大幅度动作同时伴有脚步移动,请在完全理解此节点作用后再进行调用,调用时务必注意人身与财产安全**

## 安装说明
默认通过`kuavo-ros-control/src/manipulation_nodes/planarmwebsocketservice/service/websocket_deploy_script.sh`进行自动安装,并开机自启
## 使用说明
### 节点启动
默认开机自启

如果需要手动启动,使用
```bash
source ./devel/setup.bash
roslaunch taiji_trigger_node taiji_trigger.launch
```
进行启动
### service
#### /perform_taiji
##### 作用
触发太极动作
##### 输入参数
无
##### 返回参数
success:bool 成功状态
message:string 附带信息





