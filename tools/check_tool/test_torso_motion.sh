#!/bin/bash

# 躯干往复运动测试启动脚本

echo "=== 躯干往复运动测试程序 ==="
echo "功能：控制躯干在可工作空间内往复运动"
echo "- Z轴上下 30CM"
echo "- X轴前后 10CM"
echo "- pitch 前倾 20 度"
echo "================================"

# 检查ROS环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo "错误：ROS环境未设置，请先运行 source devel/setup.bash"
    exit 1
fi

# 检查必需的服务是否可用
echo "检查必需的ROS服务..."

# 等待腿部IK服务
echo "等待 /lb_leg_ik_srv 服务..."
rosservice list | grep -q "/lb_leg_ik_srv"
if [ $? -ne 0 ]; then
    echo "警告：/lb_leg_ik_srv 服务未找到，程序可能无法正常运行"
fi

# 等待腿部控制服务  
echo "等待 /lb_leg_control_srv 服务..."
rosservice list | grep -q "/lb_leg_control_srv"
if [ $? -ne 0 ]; then
    echo "警告：/lb_leg_control_srv 服务未找到，程序可能无法正常运行"
fi

echo "服务检查完成"
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "启动躯干运动测试程序..."
echo "程序路径：${SCRIPT_DIR}/torso_motion_test.py"
echo ""
echo "按 Ctrl+C 可以随时停止测试"
echo "================================"

# 运行测试程序
cd "$SCRIPT_DIR"
python3 torso_motion_test.py

echo ""
echo "测试程序已结束" 