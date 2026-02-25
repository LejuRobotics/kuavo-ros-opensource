#!/bin/bash
echo "综合资源监控脚本 - 每10秒更新一次"
echo "按Ctrl+C停止"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

while true; do
    clear
    echo -e "${BLUE}======= 系统资源综合监控 =======${NC}"
    echo "时间: $(date)"

    # 1. 系统级监控
    echo -e "\n${GREEN}[1] 系统级资源:${NC}"
    echo -n "总FD数: "; cat /proc/sys/fs/file-nr | awk '{printf "%s (使用率: %.1f%%)\n", $1, ($1/$3)*100}'
    echo -n "系统负载: "; uptime | awk -F'load average:' '{print $2}'
    echo -n "内存使用: "; free -h | awk '/^Mem:/ {printf "%s/%s (%.1f%%)\n", $3, $2, ($3/$2)*100}'

    # 2. FD类型分布（按类型统计）
    echo -e "\n${GREEN}[2] FD类型分布（前10进程）:${NC}"
    echo "PID     | FD总数 | Sockets | 文件   | Pipes   | 命令"
    echo "--------|--------|---------|--------|---------|------------------"

    # 获取前10个FD最多的进程
    ps aux | awk '{print $2}' | while read pid; do
        fd_count=$(ls /proc/$pid/fd 2>/dev/null | wc -l)
        if [ $fd_count -gt 50 ]; then
            # 统计各种类型的FD
            socket_count=0
            file_count=0
            pipe_count=0
            other_count=0

            for fd in /proc/$pid/fd/*; do
                link=$(readlink -f "$fd" 2>/dev/null)
                case "$link" in
                    *socket*) socket_count=$((socket_count+1));;
                    *pipe*) pipe_count=$((pipe_count+1));;
                    */*) file_count=$((file_count+1));;
                    *) other_count=$((other_count+1));;
                esac
            done 2>/dev/null

            cmd=$(ps -p $pid -o cmd= | cut -c1-30)
            tty=$(ps -o tty= -p $pid 2>/dev/null | tr -d ' ')
            printf "%-7s | %-6s | %-7s | %-6s | %-7s | %s (TTY:%s)\n" \
                   "$pid" "$fd_count" "$socket_count" "$file_count" "$pipe_count" "$cmd" "$tty"
        fi
    done | sort -k3 -nr | head -10

    # 3. 网络连接监控
    echo -e "\n${GREEN}[3] 网络连接统计:${NC}"
    ss -s | grep -A 10 "Total:"

    # 4. ROS特定监控
    echo -e "\n${GREEN}[4] ROS系统状态:${NC}"
    if timeout 2 rostopic list &>/dev/null; then
        echo -n "活跃主题数: "; rostopic list 2>/dev/null | wc -l
        echo -n "活跃服务数: "; rosservice list 2>/dev/null | wc -l
        echo -n "活跃节点数: "; rosnode list 2>/dev/null | wc -l
    else
        echo "ROS主节点未运行或不可达"
    fi

    # 5. 可疑进程检测
    echo -e "\n${GREEN}[5] 可疑进程检测（FD增长快）:${NC}"
    # 这里可以添加检测FD增长率的逻辑

    # 6. 终端会话统计
    echo -e "\n${GREEN}[6] 各终端资源使用:${NC}"
    who | while read line; do
        user=$(echo $line | awk '{print $1}')
        term=$(echo $line | awk '{print $2}')
        from=$(echo $line | awk '{print $5}')

        # 计算该终端所有进程的总FD
        term_fd_total=0
        term_pids=$(ps -t $term -o pid= 2>/dev/null)

        for pid in $term_pids; do
            fd_count=$(ls /proc/$pid/fd 2>/dev/null | wc -l)
            term_fd_total=$((term_fd_total + fd_count))
        done

        if [ $term_fd_total -gt 0 ]; then
            echo -e "终端 ${YELLOW}$term${NC} (用户:$user 来源:$from) - 总FD: $term_fd_total"

            # 显示该终端中FD最多的3个进程
            for pid in $term_pids; do
                fd_count=$(ls /proc/$pid/fd 2>/dev/null | wc -l)
                if [ $fd_count -gt 20 ]; then
                    cmd=$(ps -p $pid -o cmd= | cut -c1-40)
                    echo "  PID:$pid FD:$fd_count CMD:$cmd"
                fi
            done | sort -k2 -nr | head -3
        fi
    done

    # 7. 报警信息
    echo -e "\n${GREEN}[7] 报警信息:${NC}"
    total_fd=$(cat /proc/sys/fs/file-nr | awk '{print $1}')
    fd_max=$(cat /proc/sys/fs/file-max)
    fd_usage_percent=$((total_fd * 100 / fd_max))

    if [ $fd_usage_percent -gt 80 ]; then
        echo -e "${RED}警告: FD使用率超过80% ($fd_usage_percent%)${NC}"
    elif [ $fd_usage_percent -gt 60 ]; then
        echo -e "${YELLOW}注意: FD使用率超过60% ($fd_usage_percent%)${NC}"
    else
        echo -e "${GREEN}正常: FD使用率 $fd_usage_percent%${NC}"
    fi

    echo -e "\n${BLUE}=================================${NC}"
    echo "下次更新: 10秒后"
    sleep 10
done
