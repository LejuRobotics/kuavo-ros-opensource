
## 使用
### 1. 安装依赖


### 2. 运行

仿真（可选）
```bash
roslaunch humanoid_controllers load_kuavo_gazebo_manipulate.launch joystick_type:=bt2
roslaunch ar_control robot_strategies.launch
rosrun kuavo_tf2_web_republisher kuavo_tf2_web_republisher
```


pick&place with two arms:
```bash
python kuavo_strategy_v2/pick_place_box/case.py
```

### 3. Note
Please make sure that you understand how the AprilTag frame is.
![img.png](resource/img.png)

