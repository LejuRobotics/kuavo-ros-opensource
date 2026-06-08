# KUAVO-ROS-CONTROL 安装配置脚本使用文档

## 脚本功能

本脚本用于自动化配置和安装KUAVO机器人控制系统，主要包含以下功能：

✅ PIP镜像源配置  
✅ 代码仓库克隆/更新  
✅ 机器人版本设置  
✅ 重量参数配置  
✅ 驱动板类型设置  
✅ 手臂电机配置  
✅ 末端执行器配置  
✅ VR依赖安装  
✅ 项目编译  
✅ 遥控器配置  
✅ 闭源代码清理

脚本入口：
- `tools/setup-kuavo-ros-control.sh`
- `tools/setup-kuavo-ros-control.py`

输入参数与交互输入：
- 分支名称：回车默认 `master`
- 仓库 commit：回车表示使用当前分支最新提交
- 代码源选择：1) 自动（推荐） 2) 仅工厂镜像 3) 仅 Gitee
- 机器人版本、重量、驱动板类型、末端执行器类型、是否配置 H12PRO

仓库处理行为：
- `kuavo-ros-opensource` 支持两个合法远端：
  - 工厂镜像 `git://10.11.99.175:9418/kuavo-ros-opensource.git`
  - Gitee `https://gitee.com/leju-robot/kuavo-ros-opensource.git`
- 代码源选择对应三种策略：
  - 自动（默认，回车或 1）：优先工厂镜像，失败后自动回退到 Gitee
  - 仅工厂镜像（2）：只使用工厂镜像，失败不回退（适合工厂网络环境）
  - 仅 Gitee（3）：只使用 Gitee，跳过工厂镜像（适合开发/外网环境）
- 如果本地 `origin` 是上述任一地址，都视为合法，不触发 URL 不匹配告警
- 脚本会自动补充 `origin_factory` 远端并尝试 `git fetch origin_factory`
- 当用户指定的 commit 不在 `origin_factory` 远端分支中时，脚本会打印提示，要求操作员联系 IT 同步工厂镜像

输出结果：
- 成功时输出各阶段 `[SUCCESS]` 日志
- 失败时输出 `[ERROR]` 日志并停止执行
- 远端异常但允许继续的场景会输出 `WARNING`

使用示例：

```bash
./tools/setup-kuavo-ros-control.sh
python3 ./tools/setup-kuavo-ros-control.py
```

---

## 使用步骤

### 1. 运行脚本

```bash
wget -qO /tmp/setup-kuavo-ros-control.sh https://kuavo.lejurobot.com/statics/setup-kuavo-ros-control.sh; sudo chmod +x /tmp/setup-kuavo-ros-control.sh; /tmp/setup-kuavo-ros-control.sh
```

### 2. 交互式配置流程

1. **分支选择**  
   - 输入分支名称（直接回车默认使用master分支）
   - 输入特定commit哈希（直接回车使用最新版本）
   - 选择代码源：1) 自动（推荐，优先工厂镜像，失败回退 Gitee） 2) 仅工厂镜像 3) 仅 Gitee

2. **机器人参数配置**  
   - 输入机器人版本号：[40/41/42/43/44/45]
   - 输入机器人重量（kg）：输入实际重量数值
   - 输入机器人驱动板类型：elmo/youda

3. **末端执行器配置**  
   
   ```bash
   请选择末端执行器类型:
   1) 灵巧手
   2) 二指夹爪
   ```
   - 输入末端执行器类型：[1/2]

4. **H12PRO遥控器配置**  
   - 根据提示输入 y/N 决定是否配置

---

## 注意事项

1. **权限要求**  
   - 需要sudo权限执行部分操作

2. **网络依赖**  
   - 优先访问工厂镜像 `10.11.99.175:9418`
   - 工厂镜像不可达时需要访问 Gitee 代码仓库

3. **错误处理**  
   - 脚本使用 `set -e` 遇到错误立即退出
   - 关键操作前会进行环境检查
   - 错误信息会以红色[ERROR]标出

---

## 常见问题

Q: 如何重新配置机器人参数？  
A: 直接重新运行脚本，已有配置会被覆盖更新

Q: 克隆代码仓库失败怎么办？  
A: 检查网络连接，优先确认能访问工厂镜像；若工厂镜像不可达，再确认能访问 gitee.com，或手动克隆仓库：

```bash
cd ~
git clone git://10.11.99.175:9418/kuavo-ros-opensource.git
git clone https://gitee.com/leju-robot/kuavo-ros-opensource.git
git clone https://gitee.com/leju-robot/kuavo_opensource.git
```

Q: 如何验证配置是否生效？  
A: 检查配置文件：

```bash
# 版本号
echo $ROBOT_VERSION

# 重量参数
cat ~/.config/lejuconfig/TotalMassV${ROBOT_VERSION}

# 驱动板类型 
cat ~/.config/lejuconfig/EcMasterType.ini
```

---

> 💡 提示：建议在配置完成后重启系统使所有环境变量生效  
> ⚠️ 注意：本脚本会修改系统级配置，请谨慎操作生产环境
