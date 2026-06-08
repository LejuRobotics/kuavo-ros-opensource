#!/bin/bash

# ============================================
# 动态获取脚本所在目录，推导相关路径
# ============================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SSH_SCRIPT_DIR="$SCRIPT_DIR"
ROS_WS_PATH="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
echo $ROS_WS_PATH
SSH_SCRIPT_NAME="ssh-orbbec-camera.py"
SESSION="depth_locomotion_controller"

# ============================================
# 检查并安装 tmux（保持不变）
# ============================================
# ... (tmux 安装代码保持不变)

# ============================================
# 清理旧会话并创建新会话
# ============================================
#!/bin/bash

# ... (前面的变量定义保持不变)

tmux kill-session -t $SESSION 2>/dev/null

# 创建新会话
tmux new-session -d -s $SESSION -n main

# 创建四个独立的窗格
tmux split-window -h    # 创建窗格1
tmux split-window -h    # 创建窗格2  
tmux select-pane -t 0
tmux split-window -v -f # 创建底部窗格

# 现在分别向各个窗格发送命令
# 窗格 0: roscore (左上)
tmux send-keys -t $SESSION:0.0 "sudo su" C-m
# tmux send-keys -t $SESSION:0.0 "pkill -f ros" C-m
tmux send-keys -t $SESSION:0.0 "source $ROS_WS_PATH/devel/setup.bash" C-m
tmux send-keys -t $SESSION:0.0 "roscore" C-m

# 窗格 1: image_processing (中上)
tmux send-keys -t $SESSION:0.1 "source $ROS_WS_PATH/devel/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "sleep 2" C-m
tmux send-keys -t $SESSION:0.1 "roslaunch image_processing depth_inpainter.launch" C-m

# 窗格 2: camera_ssh (右上)
tmux send-keys -t $SESSION:0.2 "cd $SSH_SCRIPT_DIR" C-m
tmux send-keys -t $SESSION:0.2 "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys -t $SESSION:0.2 "python3 $SSH_SCRIPT_NAME" C-m

# 窗格 3: humanoid_controller (底部)
# ----------  3: humanoid_controller ----------
tmux send-keys -t $SESSION:0.3 "sudo su -" C-m
tmux send-keys -t $SESSION:0.3 "ROS_WS_PATH=${ROS_WS_PATH}" C-m
tmux send-keys -t $SESSION:0.3 "export ROBOT_VERSION=54" C-m
tmux send-keys -t $SESSION:0.3 "source \${ROS_WS_PATH}/devel/setup.bash" C-m
tmux send-keys -t $SESSION:0.3 "sleep 2" C-m
tmux send-keys -t $SESSION:0.3 "roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=bt2" C-m

# 应用预定义布局：上面三列，下面一整行
tmux select-layout -t $SESSION:0 "19be,272x79,0,0{90x79,0,0[90x39,0,0,0,90x39,0,40,3],90x79,91,0,1,90x79,182,0,2}"

tmux attach -t $SESSION