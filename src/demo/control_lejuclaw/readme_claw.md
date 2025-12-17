# Kuavo 仿真夹爪抓取圆柱体任务说明

## 1. 机器人版本
- 本示例适用于 Kuavo Biped S49 仿真模型。
- 仿真 XML 路径：`src/kuavo_assets/models/biped_s49/xml/scene.xml`

## 2. 仿真场景参数说明

### 2.1 桌子参数
- 桌子已在 `scene.xml` 中 `<worldbody>` 节点下添加：
  - 位置：`pos="0.6 0 0.6"`
  - 桌面尺寸：`size="0.3 0.5 0.02"`（长宽高，单位米）
  - 桌腿尺寸：`size="0.03 0.03 0.3"`（每条腿，单位米）

### 2.2 圆柱体参数
- 圆柱体已在 `scene.xml` 中 `<body name="cylinder_anchor">` 下添加：
  - 初始位置：`pos="0.4 -0.1 0.74"`
  - 圆柱体尺寸：`<geom name="grasp_cylinder" type="cylinder" size="0.03 0.12" .../>`
    - 第一个参数 `0.03` 为半径（米），第二个参数 `0.12` 为高度（米）
  - 如需修改圆柱体半径或高度，直接调整 `size` 参数即可。

## 3. 夹爪抓取任务脚本使用说明

### 3.1 脚本路径
- 主控脚本：`src/demo/control_lejuclaw/leju_claw_cylinder_pub.py`

### 3.2 启动流程
1. 启动仿真环境（如 Mujoco）：
   ```zsh
   roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
   ```
2. 启动夹爪抓取任务脚本：
   ```zsh
   python3 src/demo/control_lejuclaw/leju_claw_cylinder_pub.py
   ```

### 3.3 任务流程
- 脚本自动执行如下动作：
  1. 手臂运动到 `[0.2, -0.1, -0.2]`，姿态 `[-1.57, 0, 1.57]`
  2. 打开夹爪
  3. 手臂运动到 `[0.4, -0.1, -0.2]`
  4. 闭合夹爪
  5. 手臂运动到 `[0.4, -0.1, 0]`
  6. 打开夹爪
- 夹爪控制通过 `/leju_claw_command` 话题，开合参数参考 `leju_claw_pub.py` 示例。
- 手臂运动通过 IK 服务 `/ik/two_arm_hand_pose_cmd_srv`，结果发布到 `/kuavo_arm_traj`。

### 3.4 参数调整
- 如需修改抓取点或姿态，可直接在脚本内调整 `move_arm` 调用参数。
- 圆柱体尺寸、位置可在 `scene.xml` 中修改。

