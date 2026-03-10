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
    - 函数：`robot_control.execute_action_file("roban2")`
    - 参数类型：string
    - 描述：机器人执行默认目录下的目标 .tact 动作
    - 备注：需将目标 .tact 文件下载到 **/home/lab/.config/lejuconfig/action_files** 目录下

6. **执行机器人目标目录下的动作**
    - 函数：`robot_control.execute_action_file("roban2", proj_name="proj_name")`
    - 参数类型：string, string
    - 描述：机器人执行 proj_name/action_files 目录下的目标 .tact 动作
    - 备注：需将目标 .tact 文件下载到 **upload_files/proj_name/action_files** 目录下

7. **执行机器人动作并同步播放音频**
    - 函数：`robot_control.execute_action_file("roban2", music_file="music_name.wav")`
    - 参数类型：string, string
    - 描述：机器人执行默认目录下的目标 .tact 动作并且播放 music_name.wav 音频
    - 备注：需将目标 .tact 文件下载到 **/home/lab/.config/lejuconfig/action_files** 目录下，音频文件下载到 *
      */home/lab/.config/lejuconfig/music** 目录下

8. **执行机器人目标目录下动作并同步播放音频**
    - 函数：`robot_control.execute_action_file("roban2", proj_name="roban2", music_file="music_name.wav")`
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
    - 函数：`robot_control.alignment_target(class_name, confidence=0.5, x=0.0, y=0.0, z=0.0, model_path=None)`
    - 参数类型：string, float, float, float, float, str（可选）
    - 描述：使机器人对准指定类别的目标对象，并根据图像中的目标位置进行移动对准。使用头部相机和 YOLO 模型检测指定类别的目标，在多个目标时选择面积最大的一个，根据目标相对图像中心的偏移量控制机器人左右/前后移动。
    - 参数说明：（以图像中心建立二维坐标，向右为 x 的正方向，向下为 y 的正方向）
      - **class_name**：YOLO 检测的目标类别名称，需与训练模型中的类别名称一致
      - **confidence**：YOLO 检测置信度阈值，范围 [0, 1]，默认 0.5
      - **x**：图像 x 方向允许的偏移范围（像素）。目标中心 x 小于 (图像中心 - x) 时机器人左移，大于 (图像中心 + x) 时机器人右移，在此范围内则不左右移动
      - **y**：图像 y 方向偏移阈值（像素）。目标中心 y 小于该值时机器人前进，否则停止前后移动
      - **z**：机器人上下蹲的高度控制参数，用于在对准过程中调整机身高度
      - **model_path**：YOLO 模型文件路径（如 .pt）。若为 None，则优先使用调用方所在目录下的 `best.pt`，若无法解析调用方则使用包内 `upload_files/best.pt`
    - 参数范围：x:[0,320] 像素；y:[-240, 240] 像素；z:[-0.3, 0.3] 米
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

6. **前往指定坐标点并设置朝向**
    - 函数：`robot_navigation.navigate_to_point_with_heading(x, y, heading)`
    - 参数类型：float, float, float
    - 描述：令机器人导航到指定的坐标点并设置朝向
    - 参数说明：x为目标x坐标（米），y为目标y坐标（米），heading为目标朝向角度（度），0度为正东方向，90度为正北方向
    - 返回值：bool，导航成功返回True，失败返回False

7. **通过任务点校准初始化**
    - 函数：`robot_navigation.init_localization_by_task_point(task_name)`
    - 参数类型： string
    - 描述：令机器人按照所给的任务点的姿态进行初始化
    - 参数说明：task_name 为目标任务点,机器人应当处于该点且朝向一致.

8. **加载地图**
    - 函数：`robot_navigation.load_map(map_name)`
    - 参数类型： string
    - 描述：令机器人加载所给的地图
    - 参数说明：map_name 为目标地图,机器人重新加载地图后,需要重新进行校准初始化.

9. **获取所有地图**
    - 函数：`robot_navigation.get_all_maps()`
    - 参数类型： 无
    - 描述：获取机器人所拥有的地图
    - 参数说明：无
    - 返回说明：返回一个列表 maps[],包含所有的地图名.

10. **获取当前地图**
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
   - 函数：`robot_control.set_stair_parameters(step_height=0.08, step_length=0.28, foot_width=0.108535, stand_height=0.0, dt=1.0, ss_time=0.6)`
   - 参数类型：float
   - 描述：设置爬楼梯的基本参数，包括台阶高度、长度、脚宽等
   - 参数说明：
     - step_height: 台阶高度（米），默认0.08
     - step_length: 台阶长度（米），默认0.28
     - foot_width: 脚宽（米），默认0.108535
     - stand_height: 站立高度偏移（米），默认0.0
     - dt: 步态周期时间（秒），默认1.0
     - ss_time: 单支撑时间比例，默认0.6
   - 返回值：bool，成功返回True，失败返回False

2. **爬上楼梯**
   - 函数：`robot_control.climb_up_stairs(num_steps=4, stair_offset=0.03)`
   - 参数类型：int, float
   - 描述：规划并添加上楼梯轨迹到累积轨迹中
   - 参数说明：
     - num_steps: 要爬升的台阶数，默认4
     - stair_offset: 台阶偏移量（米），默认0.03
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

6. **运动并对齐楼梯朝向**
   - 函数：`robot_control.align_stair()`
   - 参数类型：无
   - 描述：基于视觉 tag 识别控制机器人单步运动到楼梯前方固定的点和朝向（具体坐标可事先标定，否则使用默认值： offset_x:0.80,offset_y: 0.30,offset_yaw:0.00）
   - 返回值：bool,成功返回 True，失败返回 False

7. **一键上楼梯**
   - 函数：`robot_control.simple_up_stair(stair_height = 0.08,stair_length = 0.25,stair_num = 4)`
   - 参数类型：float,float,int,float
   - 描述：采取新的规划方式进行爬楼梯，经过测试较为稳定
   - 参数说明：
     - stair_height：楼梯的高度（米），默认 0.08
     - stair_length：单阶楼梯的长度（米），默认 0.25
     - stair_num：上楼梯的阶数，默认 4 阶
   - 返回值：bool，成功返回 True，失败返回False

**爬楼梯功能使用示例（old）：**
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

**爬楼梯功能使用示例（new）：**
```python
# 根据本地配置进行楼梯对齐
robot_control.align_stair()

# 对齐成功后一键上楼梯
robot_control.simple_up_stair(stair_height = 0.08,stair_length = 0.25,stair_num = 4)
```
**使用要求**
- 需要上位机的对齐节点以及 aplirtag 相机识别节点启动。
    ```
    roslaunch dynamic_biped sensor_apriltag_only_enable.launch
    roslaunch stair_alignment stair_alignment.launch
    ```

- 需要提前对机器人对齐的位置进行标定，否则将使用默认的参数，自动寻找 tag 进行对齐。
- 对齐成功后机器人将自动根据参数上楼梯。

**注意事项**
- 建议在运行前先对机器人在楼梯前的位置进行标定，以确保上楼梯正常。
- 爬楼梯功能需要机器人处于站立状态。
- 如果出现爬楼梯不稳定的情况，请及时调整相关参数，或者联系开发人员进行调试。

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

## 豆包相关工具类:

如果使用此类下的函数,需要在生成的python代码中,main函数之前加入以下引入代码:

```python
from kuavo_humanoid_sdk import KuavoRobotLLM
kuavo_llm = KuavoRobotLLM()
```
### 1. 导入动作到llm
需求:将所有预制动作和编辑区内的自定义动作全部包含

所有预制动作需要传入lejuconfig中,所有自定义动作按之前的逻辑传入项目中

- 函数:`kuavo_llm.import_action_from_files(project_name:str)`
- 作用:将action_files中所有动作注册到llm类中,使用文件名作为动作名称
- args:
  - project_name: str: 项目名称,用于指定知识库文件夹路径,由前端直接拼入
- return:
    None




### 2. 导入自定义函数到llm
需求:将自定义函数注册到llm的prompt中

- 函数: `kuavo_llm.register_function(function_comment: str, function: str)`
- 作用: 将自定义函数注册到llm中
- args:
  - function_comment: str: 函数的注释,需要拼接到prompt中供大模型理解函数作用
  - function: str: 自定义函数的名称,将函数积木块拖入后使用引号进行包裹
- return:
  None

例:
```python
kuavo_llm.register_function(
    function_comment="自定义函数1,用于执行某些操作",
    function='custom_func_1("abc",123)'
)
```

**注意**:'action'参数的引号为单引号



### 3. 增加实例样本:当用户说xxx时,llm应该yyy,并做zzz动作
需求: 将函数实例注册给llm,使llm在返回时能正确执行zzz动作(返回做动作的要求)

- 函数: `kuavo_llm.register_case(user_input: str, robot_response: str, action: str)
- 作用: 将函数实例注册给llm,使llm在返回时能正确执行zzz动作(返回做动作的要求)
- args:
  - user_input: str: 用户输入的文本,作为触发条件
  - robot_response: str: 机器人返回的文本,作为给llm的示例输出
  - action: str: 自定义函数的名称,将函数积木块拖入后使用引号进行包裹,或传入包含动作名称的动作执行函数
- return:
  None

示例1:
```python
kuavo_llm.register_case(
    user_input="你好",
    robot_response="你好,我是机器人",
    action='custom_func_1("123")'
)
```
示例2:
```python
kuavo_llm.register_case(
    user_input="和我打个招呼",
    robot_response="你好呀",
    action='robot_control.execute_action_file("1.前伸双手")'
)
```

示例3:
```python
kuavo_llm.register_case(
    user_input="和我跳个舞",
    robot_response="好的",
    action='robot_control.execute_action_file("跳个舞","demo_project")'
)
```
**注意**:'action'参数的引号为单引号


### 4. 添加上传到知识库的接口
需求:从知识库中读取对应文件,将文件内容拼接到prompt中

知识库文件夹路径: (与.py文件同级):knowledge_base/

- 函数:`kuavo_llm.add_file_to_prompt(file_name:str,project_name:str)`
- 作用: 将知识库中文件内容拼接到prompt中
- args:
  - file_name: str: 文件名,需要包含文件扩展名(目前只接受.txt文件) 单个文件大小限制:300KB
  - project_name: str: 项目名称,用于指定知识库文件夹路径,由前端直接拼入
- return:
  None

注:如果拼接后超限,会报错到日志

### 5. 触发asr(从麦克风接收输入后转为文本)
- 函数:`kuavo_llm.trigger_asr()`
- 作用: 触发asr,从麦克风接收输入后转为文本
- args:无
- return:
  - str: 转换后的文本

### 6. 与大模型展开会话
需求:接收文字输入/接收语音输入后进行asr,将结果文本输出到大模型中,经过prompt组装,上下文拼接后传给大模型,大模型返回文本与函数调用信息(如果有)

- 函数: `kuavo_llm.chat_with_llm(user_input: str)`
- 作用: 与大模型展开会话
- args:
  - user_input: str: 用户输入的文本
- return:
  - 
  ```
  dict{
        "success": 0, # 0:成功,1:失败
        "text": "大模型返回的文本", # 大模型返回的文本
        "intent": "意图", # 大模型返回的意图
        "slot": ""|"动作函数名称"|"自定义动作名称"|"函数调用字符串",
    }
    ```


### 7. 大模型返回并只播报语音
需求: 大模型返回文本后,将文本转换为语音,并播放出来

- 函数: `response_from_llm(llm_output: dict, action_flag = False)`
- 作用: 将文本转换为语音,并播放出来
- args:
  - llm_output: dict: 大模型返回的文本与函数调用信息
  - action_flag: bool = False: 是否执行函数调用,需要由前端拼入
- return:
  - None
def main():
    ...
    response_from_llm(kuavo_llm.chat_with_llm(),False)
```
注:该功能需要在生成的python代码中通过字符串运行函数,所以需要类似aelos那样在生成的python代码中加入函数定义,见9. 语音播报函数定义


### 8. 大模型返回播报语音并执行函数
需求: 大模型返回文本后,将文本转换为语音,并播放出来,如果有函数调用,则执行函数

- 函数: `response_from_llm(llm_output, action_flag = True)`
- 作用: 将文本转换为语音,并播放出来,如果有函数调用,则执行函数
- args:
  - llm_output: dict: 大模型返回的文本与函数调用信息
  - action_flag: bool = True: 是否执行函数调用,需要由前端拼入
- return:
  - None
  
注: 该功能需要在生成的python代码中通过字符串运行函数,所以需要类似aelos那样在生成的python代码中加入函数定义
```python
def main():
    ...
    response_from_llm(kuavo_llm.chat_with_llm(),True)
```

### 9. 语音播报函数定义
函数定义如下:
```python
...
def response_from_llm(llm_output: dict, action_flag: bool = False):
    if not llm_output.get("success", 1) == 0:
        return
    kuavo_llm.response_with_voice(llm_output) 
    if action_flag:
        if llm_output.get("intent", "") == "function_call":
            try:
                eval(llm_output.get("slot"))
            except:
                print(f"【函数调用】执行失败:{llm_output.get('slot','')}")
        if llm_output.get("intent", "") == "action":
            robot_control.execute_action_file(llm_output["slot"])

        if llm_output.get("intent", "") == "action_custom":
            robot_control.execute_action_file(llm_output['slot'],"demo_project") # 拼接项目信息
    while not kuavo_llm.tts_end.is_synthesis_finished() or kuavo_llm.playing_status:
        time.sleep(0.1)
...

```
### 9. 播放提示音
如果需要播放预制提示音,可以使用"播放音乐"接口:`kuavo_llm.play_music(music_name: str)`(见本文档111行:`17. **播放音乐**`)

需要用到的音效需要提前(也可以点击运行时)放入`~/.config/lejuconfig/music/`中

### 10. 播放自定义文本作为提示音
与 [大模型播放语音](#7.-大模型返回并只播报语音) 功能类似,只是将文本转换为语音后,播放出来的是自定义的文本,而不是大模型返回的文本

在使用时,需要将`9. 语音播报函数定义`中的函数预先定义,然后在积木块对应位置拼入以下函数调用:
```python
response_from_llm(text:str,False)
```

其中text为需要播放的自定义文本