# Hand Reference Mode Configuration

可以选择不同的手部参考点来确定手的位置。

## 可用的手部参考模式

### 1. thumb_index (默认)
使用拇指和食指中点作为参考点。
```bash
--hand_reference_mode thumb_index
```

### 2. fingertips 
使用所有手指尖的中心点作为参考点。
```bash
--hand_reference_mode fingertips
```

### 3. middle_finger
使用中指尖作为参考点，适合指向性操作。
```bash
--hand_reference_mode middle_finger
```



## 使用方法

### 方法1: 命令行参数（ik_ros_uni.py）
```bash
python ik_ros_uni.py --hand_reference_mode fingertips
```

### 方法2: Launch文件参数
所有相关的launch文件都已经支持hand_reference_mode参数：

#### launch_quest3_ik.launch
```bash
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch hand_reference_mode:=thumb_index

# 可选配置参数：use_cpp_ik
# 启动python版本的ik
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=false

# 启动C++版本的ik
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=true

# 可选配置参数：use_incremental_ik(仅当use_cpp_ik:=true 时，可选是否启用增量式IK)
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=true use_incremental_ik:=true
```

#### launch_quest3_ik_videostream.launch  
```bash
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik_videostream.launch hand_reference_mode:=fingertips
```

#### launch_quest3_ik_videostream_usb_cam.launch
```bash
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik_videostream_usb_cam.launch hand_reference_mode:=middle_finger
```

#### kinematic_mpc_vr.launch
```bash
roslaunch motion_capture_ik kinematic_mpc_vr.launch hand_reference_mode:=thumb_index
```

### 方法3: ROS参数（quest3_node.py）
```bash
rosrun motion_capture_ik quest3_node.py _hand_reference_mode:=thumb_index
```

或者在launch文件中：
```xml
<node name="quest3_node" pkg="motion_capture_ik" type="quest3_node.py">
    <param name="hand_reference_mode" value="fingertips"/>
</node>
```



## 注意事项

1. 如果选择的参考点无法获取有效数据，系统会自动回退到使用 thumb_index 模式
2. 所有模式都保持手掌的方向作为手部姿态的参考
3. 新的参考点只影响位置计算，不影响姿态计算
4. 所有现有的launch文件都向后兼容，默认使用thumb_index模式 