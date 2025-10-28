日志服务启动与测试说明
进入日志服务目录，切换超级用户，运行部署脚本启动服务：

cd kuavo-ros-control/src/manipulation_nodes/planarmwebsocketservice/service
sudo su
./websocket_start_script.sh


进入日志服务目录，运行测试脚本：

cd kuavo-ros-control/src/manipulation_nodes/planarmwebsocketservice/tests
python3 test_logger.py

运行测试脚本后，终端会显示收到的日志信息，例如：

[收到消息] {"type": "welcome", "message": "已经成功连接到日志服务器"}
后续使用有发布log的sdk会收到类似消息：
[收到消息] {"level": "INFO", "timestamp": "2025-07-23 10:07:31.002", "message": "Head logger client 已启动", "module": "asyncio.events", "function": "_run"}
[收到消息] {"level": "INFO", "timestamp": "2025-07-23 10:07:32.534", "message": "开始控制头部运动: yaw=0.000, pitch=0.000", "module": "asyncio.events", "function": "_run"}
[收到消息] {"level": "INFO", "timestamp": "2025-07-23 10:07:32.535", "message": "头部控制完成: yaw=0.000, pitch=0.000, 结果=成功", "module": "asyncio.events", "function": "_run"}
