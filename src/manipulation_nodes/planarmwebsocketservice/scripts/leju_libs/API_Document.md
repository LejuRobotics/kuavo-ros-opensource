# 积木块函数定义文档

## 运动控制模块：

- 模块类：RobotControl
- 使用方法：

```python
    from kuavo_humanoid_sdk import RobotControlBlockly

robot_control = RobotControlBlockly()
```

以下为运动控制模块的各个积木块函数说明：

1. **启动机器人运动**
    - 函数：`robot_control.start()`
    - 参数类型：无
    - 描述：令机器人原地踏步

2. **以 X 速度,Y 速度,yaw 旋转速度移动**
    - 函数：`robot_control.walk(x, y, yaw)`
    - 参数类型：float
    - 描述：令机器人以 X，Y，YAW 的速度进行移动
    - 参数范围：x: [-0.4, 0.4]米/秒，y: [-0.2, 0.2]米/秒，yaw: [-0.4, 0.4]弧度/秒
    -
3. **以 X 速度,Y 速度,yaw 旋转速度移动**
    - 函数：`robot_control.walk_angle(x, y, yaw)`
    - 参数类型：float
    - 描述：令机器人以 X，Y，YAW 的速度进行移动
    - 参数范围：x: [-0.4, 0.4]米/秒，y: [-0.2, 0.2]米/秒，yaw: [-22.92, 22.92]度/秒

4. **停止机器人运动**
    - 函数：`robot_control.stop()`
    - 参数类型：无
    - 描述：令机器人停止移动

5. **执行机器人动作**
    - 函数：`robot_control.excute_action_file("roban2")`
    - 参数类型：string
    - 描述：机器人执行默认目录下的目标 .tact 动作
    - 备注：需将目标 .tact 文件下载到 **/home/lab/.config/lejuconfig/action_files** 目录下

6. **执行机器人目标目录下的动作**
    - 函数：`robot_control.excute_action_file("roban2", proj_name="proj_name")`
    - 参数类型：string, string
    - 描述：机器人执行 proj_name/action_files 目录下的目标 .tact 动作
    - 备注：需将目标 .tact 文件下载到 **upload_files/proj_name/action_files** 目录下

7. **执行机器人动作并同步播放音频**
    - 函数：`robot_control.excute_action_file("roban2", music_file="music_name.wav")`
    - 参数类型：string, string
    - 描述：机器人执行默认目录下的目标 .tact 动作并且播放 music_name.wav 音频
    - 备注：需将目标 .tact 文件下载到 **/home/lab/.config/lejuconfig/action_files** 目录下，音频文件下载到 *
      */home/lab/.config/lejuconfig/music** 目录下

8. **执行机器人目标目录下动作并同步播放音频**
    - 函数：`robot_control.excute_action_file("roban2", proj_name="roban2", music_file="music_name.wav")`
    - 参数类型：string, string, string
    - 描述：机器人执行目标目录下的目标 .tact 动作并且播放 music_name.wav 音频
    - 备注：需将目标 .tact 文件下载到 **upload_files/proj_name/action_files** 目录下，音频文件下载到 *
      */home/lab/.config/lejuconfig/music** 目录下

9. **设置机器人手臂模式**
    - 函数：`robot_control.set_arm_control_mode(mode)`
    - 参数类型：int
    - 描述：设定机器人的手臂模式
    - 参数说明：mode: 0-固定模式，1-自动摆动模式，2-外部控制模式

10. **机器人恢复初始站立姿态**
    - 函数：`robot_control.to_stance()`
    - 参数类型：无
    - 描述：恢复机器人的站立姿态到初始状态

11. **机器人躯干上升/下降**
    - 函数：`robot_control.control_robot_height(is_up, height)`
    - 参数类型：string, float
    - 描述：令机器人的质心在竖直方向上移动
    - 参数说明：is_up: "down" 表示质心下降，"up" 表示质心上升；height: [-0.3, 0.0]米

12. **移动机器人头部到偏航角和俯仰角**
    - 函数：`robot_control.control_robot_head(yaw, pitch)`
    - 参数类型：float
    - 描述：控制机器人的头同时在 yaw 和 pitch 上进行移动
    - 参数范围：yaw: [-80, 80]度；Roban2 pitch: [0, 25]度；Kuavo pitch: [-25, 25]度

13. **仅移动机器人头部到偏航角**
    - 函数：`robot_control.control_robot_head_only_yaw(yaw)`
    - 参数类型：float
    - 描述：仅控制机器人的头在 yaw 方向上移动
    - 参数范围：yaw: [-80, 80]度

14. **仅移动机器人头部到俯仰角**
    - 函数：`robot_control.control_robot_head_only_pitch(pitch)`
    - 参数类型：float
    - 描述：仅控制机器人的头在 pitch 方向上移动
    - 参数范围：Roban2 pitch: [0, 25]度；Kuavo pitch: [-25, 25]度

15. **移动机器手臂到目标姿态**
    - 函数：`robot_control.control_arm_target_pose(x1, y1, z1, yaw1, pitch1, roll1, x2, y2, z2, yaw2, pitch2, roll2)`
    - 参数类型：float
    - 描述：控制机器人的手臂移动到目标姿态
    - 参数说明：(x1, y1, z1, yaw1, pitch1, roll1) 为左臂目标姿态，(x2, y2, z2, yaw2, pitch2, roll2) 为右臂目标姿态

16. **移动机器人单臂到目标姿态**
    - 函数：`robot_control.control_arm_target_pose_by_single(hand_type, x, y, z, yaw, pitch, roll)`
    - 参数类型：string, float
    - 描述：仅控制机器人的单臂进行运动
    - 参数说明：hand_type: "L" 表示左臂运动，"R" 表示右臂运动；(x, y, z, yaw, pitch, roll) 为目标姿态

17. **播放音乐**
    - 函数：`robot_control.play_music("music_name.wav")`
    - 参数类型：string
    - 描述：控制机器人播放指定音频
    - 备注：需将目标音频文件下载到 **/home/lab/.config/lejuconfig/music** 目录下  

18. **腰部控制**
    - 函数：`robot_control.control_waist_rotation(degree)`
    - 参数类型：float
    - 描述：控制机器人的腰部在 yaw 方向上转动
    - 参数范围：[-120.0, 120.0]度

19. **停止播放音乐**
    - 函数：`robot_control.stop_music()`
    - 参数类型：string
    - 描述：控制机器人播放指定音频
    - 备注：需将目标音频文件下载到 **/home/lab/.config/lejuconfig/music** 目录下  

19. **机器人对准目标**  
    - 函数：`robot_control.alignment_target(class_name, x, y, z)`
    - 参数类型：string, float, float, float
    - 描述：使机器人对准指定类别到对象，并移动到目标前
    - 参数说明：（以图像中心建立二维坐标，向右为 x 的正方向，向下为 y 的正方向）
      - class_id 为 yolo 检测中需要检测的对象名称，和训练模型中的类别名称一致;
      - x 为图像 x 方向的偏移值，物体中心的 x 小于该参数 -x 时，机器人左移，大于该参数 x 时，机器人右移
      - y 为图像 y 方向的偏移值，物体中心的 y 小于该参数 y 时，机器人前进，否则停止移动
      - z 为机器人上下蹲的高度控制
    - 参数范围：x:[0,320]像素点;y:[-240，240]像素点;z:[-0.3,0.3]米
    - 返回值：无

## 导航类模块定义
```python
from kuavo_humanoid_sdk import RobotNavigationBlockly
    robot_navigation = RobotNavigationBlockly
```

1. **导航到目标点**
    - 函数：`robot_navigation.navigate_to_goal(x, y, z, yaw, pitch, roll)`
    - 参数类型： float
    - 描述：令机器人导航到目标 pose.
    - 参数说明：(x, y, z, yaw, pitch, roll) 为目标姿态

2. **导航到任务点**
    - 函数：`robot_navigation.navigate_to_task_point("task1")`
    - 参数类型： string
    - 描述：令机器人导航到 "task1" 任务点
    - 参数说明：task1 为目标任务点的名字

3. **停止所有导航任务**
    - 函数：`robot_navigation.stop_navigation()`
    - 参数类型： 无
    - 描述：取消机器人的导航任务
    - 参数说明：无

4. **获取当前导航状态**
    - 函数：`robot_navigation.get_current_status()`
    - 参数类型： 无
    - 描述：获取当前机器人的导航状态
    - 参数说明：无
    - 返回说明：
    ```bash
        PENDING = 0         #等待中
        ACTIVE = 1          #导航中
        PREEMPTED = 2       #导航被取消(成功取消)
        SUCCEEDED = 3       #导航成功
        ABORTED = 4         #导航失败
        REJECTED = 5        #导航被拒绝
    ```

5. **通过目标位姿校准初始化**
    - 函数：`robot_navigation.init_localization_by_pose(x, y, z, roll, pitch, yaw)`
    - 参数类型： float
    - 描述：令机器人按照所给的姿态进行初始化
    - 参数说明：(x, y, z, yaw, pitch, roll) 为目标姿态

6. **通过任务点校准初始化**
    - 函数：`robot_navigation.init_localization_by_task_point(task_name)`
    - 参数类型： string
    - 描述：令机器人按照所给的任务点的姿态进行初始化
    - 参数说明：task_name 为目标任务点,机器人应当处于该点且朝向一致.

7. **加载地图**
    - 函数：`robot_navigation.load_map(map_name)`
    - 参数类型： string
    - 描述：令机器人加载所给的地图
    - 参数说明：map_name 为目标地图,机器人重新加载地图后,需要重新进行校准初始化.

8. **获取所有地图**
    - 函数：`robot_navigation.get_all_maps()`
    - 参数类型： 无
    - 描述：获取机器人所拥有的地图
    - 参数说明：无
    - 返回说明：返回一个列表 maps[],包含所有的地图名.

9. **获取当前地图**
    - 函数：`robot_navigation.get_current_map()`
    - 参数类型： 无
    - 描述：获取机器人当前所用的地图
    - 参数说明：无
    - 返回说明：返回一个字符串,是当前所用的地图.

## 爬楼梯类模块定义

- 模块类：RobotControlBlockly（爬楼梯功能集成在运动控制模块中）

- 使用方法：
```python
from kuavo_humanoid_sdk import RobotControlBlockly
    robot_control = RobotControlBlockly()
```

以下为爬楼梯模块的各个积木块函数说明：

1. **设置爬楼梯参数**
   - 函数：`robot_control.set_stair_parameters(step_height=0.13, step_length=0.28, foot_width=0.10, stand_height=0.0, dt=0.6, ss_time=0.5)`
   - 参数类型：float
   - 描述：设置爬楼梯的基本参数，包括台阶高度、长度、脚宽等
   - 参数说明：
     - step_height: 台阶高度（米），默认0.13
     - step_length: 台阶长度（米），默认0.28
     - foot_width: 脚宽（米），默认0.10
     - stand_height: 站立高度偏移（米），默认0.0
     - dt: 步态周期时间（秒），默认0.6
     - ss_time: 单支撑时间比例，默认0.5
   - 返回值：bool，成功返回True，失败返回False

2. **爬上楼梯**
   - 函数：`robot_control.climb_up_stairs(num_steps=5)`
   - 参数类型：int
   - 描述：规划并添加上楼梯轨迹到累积轨迹中
   - 参数说明：num_steps: 要爬升的台阶数，默认5
   - 返回值：bool，成功返回True，失败返回False

3. **爬下楼梯**
   - 函数：`robot_control.climb_down_stairs(num_steps=5)`
   - 参数类型：int
   - 描述：规划并添加下楼梯轨迹到累积轨迹中（当前功能已禁用）
   - 参数说明：num_steps: 要下降的台阶数，默认5
   - 返回值：bool，当前固定返回False（功能开发中）
   - 备注：⚠ 下楼梯功能当前已禁用（开发中）

4. **爬楼梯模式移动到指定位置**
   - 函数：`robot_control.stair_move_to_position(dx=0.2, dy=0.0, dyaw=0.0, max_step_x=0.28, max_step_y=0.15, max_step_yaw=30.0)`
   - 参数类型：float
   - 描述：规划爬楼梯移动到位置轨迹并添加到累积轨迹中
   - 参数说明：
     - dx: X方向位移（米），默认0.2
     - dy: Y方向位移（米），默认0.0
     - dyaw: 偏航角位移（度），默认0.0
     - max_step_x: X方向最大步长，默认0.28
     - max_step_y: Y方向最大步长，默认0.15
     - max_step_yaw: 偏航角最大步长（度），默认30.0
   - 返回值：bool，成功返回True，失败返回False

5. **执行爬楼梯轨迹**
   - 函数：`robot_control.execute_stair_trajectory()`
   - 参数类型：无
   - 描述：执行完整的累积爬楼梯轨迹
   - 返回值：bool，成功返回True，失败返回False

**爬楼梯功能使用示例：**
```python
# 设置爬楼梯参数
robot_control.set_stair_parameters(step_height=0.15, step_length=0.30)

# 规划上楼梯轨迹
robot_control.climb_up_stairs(num_steps=3)

# 规划移动轨迹
robot_control.stair_move_to_position(dx=0.5, dy=0.0, dyaw=0.0)

# 执行完整轨迹
robot_control.execute_stair_trajectory()
```

**详细参数说明：**

爬楼梯功能的成功率非常依赖于轨迹的调控，以下是关键参数的详细说明：

- `dt = 0.6`：腾空步态周期（秒）
- `ss_time = 0.5`：支撑相时间比例
- `foot_width = 0.10`：足宽（米）
- `step_height = 0.13`：台阶高度（米）
- `step_length = 0.28`：台阶长度（**米**）

**使用要求：**

- 将机器人放于楼梯前**固定距离**（大概2cm）
- 机器人必须处于站立状态，面向楼梯方向
- 执行前确保楼梯参数设置正确

**注意事项：**

- 所有爬楼梯相关函数需要先调用规划函数添加轨迹，最后调用 `execute_stair_trajectory()` 执行
- 爬楼梯功能需要机器人处于站立状态
- 下楼梯功能当前仍在开发中，暂时无法使用
- 建议在执行前先用 `set_stair_parameters()` 设置合适的参数
- 如果出现碰撞或不稳定，可能需要调整腾空相轨迹参数
- 直接修改参数能够适合大部分情况，具体参数需要根据实际楼梯尺寸调整

## 语音类模块定义

- 模块类：RobotMicrophone

- 使用方法：

```python
from kuavo_humanoid_sdk import RobotMicrophone
    robot_microphone = RobotMicrophone()
```

以下为语音模块的各个积木块函数说明：

1. **等待唤醒词**
   
   - 函数：`microphone.wait_for_wake_word(timeout_sec)`
   
   - 参数类型：int
   
   - 描述：机器人等待语音唤醒，唤醒词为 “Roban Roban”。
   
   - 返回值：True: 接收到唤醒词，False: 超过指定时间未收到唤醒词
   
   - 参数范围：timeout_sec 默认为 60s

## 对话类模块定义
- 模块类：RobotSpeech
- 使用方法：
```python
from kuavo_humanoid_sdk import RobotSpeech
    robot_speech = RobotSpeech()
```

以下为对话模块的各个积木块函数说明：

1. **建立豆包实时语音对话连接**
   - 函数：`establish_doubao_speech_connection(app_id, access_key)`
   - 参数类型：app_id : str, access_key : str
   - 参数描述：机器人语音对话功能采用豆包端到端实时语音大模型 API，使用火山引擎控制台获取 APP ID 和 Access Token.
   - 返回值：True: 鉴权成功，False: 鉴权失败.

2. **开始对话**
   - 函数：`robot_speech.start_speech()`
   - 描述：开始机器人语音对话.

3. **结束对话**
   - 函数：`robot_speech.stop_speech()`
   - 描述：结束机器人语音对话.

## 灵巧手类模块定义
- 模块类：DexterousHand
- 使用方法：
```python
from kuavo_humanoid_sdk import DexterousHand
    robot_hand = DexterousHand()
```

以下为灵巧手模块的各个积木块函数说明：

灵巧手关节对应：
```python
单手掌关节列表顺序:['大拇指关节，拇指外展肌，食指关节, 中指关节，无名指关节，小指关节']
```

1. **控制左手灵巧手**
   - 函数：`control_left(target_positions)`
   - 参数类型：target_positions : list
   - 参数描述：令机器人的左手 6 个关节运动到 target_position 的位置,关节范围:[0,100]
   - 返回值：True: 控制成功，False: 控制失败.

2. **控制右手灵巧手**
   - 函数：`control_right(target_positions)`
   - 参数类型：target_positions : list
   - 参数描述：令机器人的右手 6 个关节运动到 target_position 的位置。关节范围:[0,100]
   - 返回值：True: 控制成功，False: 控制失败.
3. **控制双手灵巧手**
   - 函数：`control(target_positions)`
   - 参数类型：target_positions : list
   - 参数描述：令机器人的双手共 12 个关节运动到 target_position 的位置。关节范围:[0,100]
   - 返回值：True: 控制成功，False: 控制失败.

---
灵巧手手势列表：
```python
empty → 初始手势

two-finger-spread-unopposed → 两指张开（无拇指配合）

two-finger-spread-opposed → 两指张开（拇指配合）

tripod-pinch-opposed → 三指捏握（拇指相对）

thumbs-up → 竖大拇指

inward-thumb → 拇指向内

five-finger-pinch → 五指捏握

rock-and-roll → “摇滚”手势（大拇指与小拇指举起）

pen-grip2 → 笔握 2

finger-pointing-unopposed → 单指指向（无拇指配合）

flick-index-finger → 弹食指

cylindrical-grip → 圆柱握持

pen-grip3 → 笔握 3

tripod-pinch-unpposed → 三指捏握（无拇指配合）

finger-pointing-opposed → 单指指向（拇指配合）

palm-open → 手掌张开

flick-middle-finger → 弹中指

fist → 拳头

four-finger-straight → 四指伸直

mouse-control → 鼠标控制手势

pen-grip1 → 笔握 1

precision-pinch-opposed → 精细捏握（拇指相对）

precision-pinch-unopposed → 精细捏握（无拇指配合）

shaka-sign → “Shaka” 手势（夏威夷致意，拇指和小指伸出）

side-pinch → 侧捏握持

```
---
4. **控制左手灵巧手执行手势**
   - 函数：`make_gesture_sync(gesture_name,None)`
   - 参数类型：gesture_name : str
   - 参数描述：令机器人的左手执行手势 gesture_name。
   - 返回值：True: 控制成功并完成，False: 控制失败或超时.

5. **控制右手灵巧手执行手势**
   - 函数：`make_gesture_sync(None,gesture_name)`
   - 参数类型：gesture_name : str
   - 参数描述：令机器人的右手执行手势 gesture_name。
   - 返回值：True: 控制成功并完成，False: 控制失败或超时.

6. **控制双手灵巧手执行手势**
   - 函数：`make_gesture_sync(l_gesture_name,r_gesture_name)`
   - 参数类型：l_gesture_name : str, r_gesture_name : str
   - 参数描述：令机器人的左手执行 l_gesture_name 手势，右手执行 r_gesture_name 手势。
   - 返回值：True: 控制成功并完成，False: 控制失败或超时.
