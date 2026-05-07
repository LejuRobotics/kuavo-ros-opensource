#!/bin/bash

# ===================== 机型检测 =====================
# 根据 ROBOT_VERSION 环境变量判断机型并设置温度阈值
ROBOT_VERSION=${ROBOT_VERSION:-""}
if [[ -n "$ROBOT_VERSION" ]]; then
    # 提取版本号前两位
    VERSION_PREFIX=${ROBOT_VERSION:0:2}
    
    if [[ "$VERSION_PREFIX" =~ ^5[0-9]$ ]]; then
        # kuavo5 系列 (50~59)
        ROBOT_MODEL="kuavo5"
        MAX_TEMP=80
    elif [[ "$VERSION_PREFIX" =~ ^6[0-9]$ ]]; then
        # kuavo5w 系列 (60~69)
        ROBOT_MODEL="kuavo5w"
        MAX_TEMP=80
    elif [[ "$VERSION_PREFIX" =~ ^1[3-7]$ ]]; then
        # roban 系列 (13~17)
        ROBOT_MODEL="roban"
        MAX_TEMP=90
    else
        # 未知机型，让用户手动选择
        echo "========================================"
        echo " CPU 压力 & 温度 & 频率稳定性测试"
        echo "========================================"
        echo "未知机型 (版本号: $ROBOT_VERSION)"
        echo
        echo "请选择机型："
        echo "  1) kuavo5 / kuavo5w (温度阈值: 80°C)"
        echo "  2) roban (温度阈值: 90°C)"
        echo
        while true; do
            read -p "请输入选项 (1 或 2): " model_choice
            case $model_choice in
                1)
                    ROBOT_MODEL="kuavo5"
                    MAX_TEMP=80
                    break
                    ;;
                2)
                    ROBOT_MODEL="roban"
                    MAX_TEMP=90
                    break
                    ;;
                *)
                    echo "无效选项，请重新输入"
                    ;;
            esac
        done
    fi
else
    # 未设置环境变量，让用户手动选择
    echo "========================================"
    echo " CPU 压力 & 温度 & 频率稳定性测试"
    echo "========================================"
    echo "未检测到 ROBOT_VERSION 环境变量"
    echo
    echo "请选择机型："
    echo "  1) kuavo5 / kuavo5w (温度阈值: 80°C)"
    echo "  2) roban (温度阈值: 90°C)"
    echo
    while true; do
        read -p "请输入选项 (1 或 2): " model_choice
        case $model_choice in
            1)
                ROBOT_MODEL="kuavo5"
                MAX_TEMP=80
                break
                ;;
            2)
                ROBOT_MODEL="roban"
                MAX_TEMP=90
                break
                ;;
            *)
                echo "无效选项，请重新输入"
                ;;
        esac
    done
fi

# ===================== 启动菜单 =====================
echo "========================================"
echo " CPU 压力 & 温度 & 频率稳定性测试"
echo "========================================"
echo "检测到机型: $ROBOT_MODEL (版本号: ${ROBOT_VERSION:-未知}), 温度阈值: ${MAX_TEMP}°C"
echo
echo "默认时长为 10 分钟，直接回车即可。"
echo
echo "若需自定义时长，请输入时长（分钟）后回车。"
echo " - 测试过程中按 q 可安全退出"
echo " - Ctrl+C 强制退出"
echo "========================================"

echo

read -p "请输入测试时长（分钟，直接回车使用默认10分钟）: " INPUT_MINUTES
if [[ -n "$INPUT_MINUTES" ]]; then
    TEST_DURATION=$((INPUT_MINUTES * 60))
else
    TEST_DURATION=600
fi

echo "测试时长设定为：$((TEST_DURATION / 60)) 分钟 ($TEST_DURATION 秒)"
sleep 1

# ===================== 基本参数 =====================
CPU_COUNT=$(nproc)

LOW_FREQ_THRESHOLD=3000
LOW_FREQ_FAIL_COUNT=50  #低频时长阈值
TEMP_OVER_DURATION=50   #温度超过阈值持续时间阈值（秒）

# ===================== 统计变量 =====================
over_threshold_count=0
declare -A low_freq_count
declare -A low_freq_duration
TEST_FAIL=0

# ===================== 日志 =====================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="${SCRIPT_DIR}/stress_test_log_$(date +'%Y%m%d_%H%M%S').log"
echo "日志开始时间: $(date)" > "$LOG_FILE"

# ===================== 启动压力测试 =====================
echo "开始压力测试... 使用 $CPU_COUNT 核心" | tee -a "$LOG_FILE"
stress --cpu $CPU_COUNT &
STRESS_PID=$!

start_time=$(date +%s)

# ===================== 监控循环 =====================
CLEAR_SCREEN="\033[2J\033[H"

while true; do
    # q 退出
    if read -t 0.1 -n 1 key 2>/dev/null; then
        if [[ "$key" == "q" ]] || [[ "$key" == "Q" ]]; then
            echo -e "\n$(date '+%F %T') 手动退出压力测试（q），已运行 $elapsed_time 秒" | tee -a "$LOG_FILE"
            sudo killall stress 2>/dev/null || killall stress 2>/dev/null
            pkill stress-ng
            break
        fi
    fi

    current_time=$(date +%s)
    elapsed_time=$((current_time - start_time))
    
    # 计算进度百分比
    progress=$((elapsed_time * 100 / TEST_DURATION))
    [[ $progress -gt 100 ]] && progress=100

    # ---------- 温度 ----------
    temperature=$(sensors 2>/dev/null | grep -i 'Package id 0' | grep -oP '\+?\d+\.\d+' | head -n 1 | sed 's/+//')
    
    # 构建显示缓冲区
    OUTPUT="${CLEAR_SCREEN}"
    OUTPUT+="========================================\r\n"
    OUTPUT+=" CPU 压力测试进行中...\r\n"
    OUTPUT+="========================================\r\n"
    OUTPUT+="已运行时间: ${elapsed_time}/${TEST_DURATION} 秒\r\n"
    
    # 进度条
    progress_bar=$(printf '=%.0s' $(seq 1 $((progress/2))))
    OUTPUT+="进度: [$(printf '%-50s' "$progress_bar")] ${progress}%\r\n\r\n"

    if [[ -n "$temperature" ]]; then
        OUTPUT+="当前温度: ${temperature}°C (阈值: ${MAX_TEMP}°C)"
        
        # 日志写入频率（每5秒记录一次）
        if (( elapsed_time % 5 == 0 )); then
            echo "$(date '+%F %T') 当前温度: $temperature°C" >> "$LOG_FILE"
        fi

        if (( $(echo "$temperature > $MAX_TEMP" | bc -l 2>/dev/null) )); then
            over_threshold_count=$((over_threshold_count + 1))
            OUTPUT+=" ⚠ 超过阈值！"
            echo "$(date '+%F %T') ⚠ 温度超过阈值 $MAX_TEMP°C (累计: $over_threshold_count 次)" >> "$LOG_FILE"

            if (( over_threshold_count > TEMP_OVER_DURATION )); then
                TEST_FAIL=1
                echo "$(date '+%F %T') ❌ 温度超过阈值持续时间过长 ($over_threshold_count > $TEMP_OVER_DURATION 秒)" >> "$LOG_FILE"
            fi
        fi
        OUTPUT+="\r\n"
    else
        OUTPUT+="当前温度: ⚠ 无法获取温度信息\r\n"
        if (( elapsed_time % 5 == 0 )); then
            echo "$(date '+%F %T') ⚠ 无法获取温度" >> "$LOG_FILE"
        fi
    fi
    
    OUTPUT+="\r\n"

    # ---------- CPU 频率 ----------
    cpu_freqs=($(grep 'cpu MHz' /proc/cpuinfo 2>/dev/null | awk '{print $4}'))
    num_cores=${#cpu_freqs[@]}
    core_num=0
    low_freq_cores=()

    if [[ $num_cores -gt 0 ]]; then
        OUTPUT+="CPU 频率监控（核心数: $num_cores）:\r\n"
        
        for cpu_freq in "${cpu_freqs[@]}"; do
            # 日志写入频率（每5秒记录一次）
            if (( elapsed_time % 5 == 0 )); then
                echo "$(date '+%F %T') 核心 $core_num: $cpu_freq MHz" >> "$LOG_FILE"
            fi

            # 启动5秒后再进行低频统计，避免启动初期不稳定
            if (( elapsed_time > 5 )) && (( core_num < num_cores - 2 )); then
                if (( $(echo "$cpu_freq < $LOW_FREQ_THRESHOLD" | bc -l 2>/dev/null) )); then
                    low_freq_count[$core_num]=$((low_freq_count[$core_num] + 1))
                    low_freq_duration[$core_num]=$((low_freq_duration[$core_num] + 1))
                    low_freq_cores+=("核心${core_num}:${cpu_freq}MHz")

                    if (( low_freq_count[$core_num] >= LOW_FREQ_FAIL_COUNT )); then
                        TEST_FAIL=1
                        echo "$(date '+%F %T') ❌ 核心 $core_num 低频持续时间过长 (${low_freq_duration[$core_num]} 秒 >= $LOW_FREQ_FAIL_COUNT)" >> "$LOG_FILE"
                    fi
                fi
            fi
            
            # 只显示前8个核心，避免屏幕过于拥挤
            if [[ $core_num -lt 8 ]]; then
                if (( $(echo "$cpu_freq < $LOW_FREQ_THRESHOLD" | bc -l 2>/dev/null) )); then
                    OUTPUT+="  核心 $core_num: $(printf '%7.0f' "$cpu_freq") MHz ⚠\r\n"
                else
                    OUTPUT+="  核心 $core_num: $(printf '%7.0f' "$cpu_freq") MHz ✓\r\n"
                fi
            fi
            
            core_num=$((core_num + 1))
        done
        
    else
        OUTPUT+="⚠ 无法读取 CPU 频率信息\r\n"
        if (( elapsed_time % 5 == 0 )); then
            echo "$(date '+%F %T') ⚠ 无法读取 CPU 频率信息" >> "$LOG_FILE"
        fi
    fi
    
    OUTPUT+="\r\n按 q 键可提前退出测试\r\n"
    OUTPUT+="========================================\r\n"
    
    # 一次性输出所有内容
    echo -ne "$OUTPUT"

    # ---------- 检查测试时间是否结束 ----------
    if (( elapsed_time >= TEST_DURATION )); then
        echo -e "\n$(date '+%F %T') 测试完成（达到设定时长 $TEST_DURATION 秒）" | tee -a "$LOG_FILE"
        sudo killall stress 2>/dev/null || killall stress 2>/dev/null
        pkill stress-ng
        break
    fi
    
    sleep 1
done

# ===================== 结果汇总 =====================
echo
echo "==================== 测试结果 ====================" | tee -a "$LOG_FILE"

if (( TEST_FAIL == 1 )); then
    echo -e "\033[31m❌ 测试未通过\033[0m" | tee -a "$LOG_FILE"
    echo
    echo "失败原因:" | tee -a "$LOG_FILE"
    
    if (( over_threshold_count > TEMP_OVER_DURATION )); then
        echo "  - 温度超过 ${MAX_TEMP}°C 的持续时间: $over_threshold_count 秒（阈值: $TEMP_OVER_DURATION 秒）" | tee -a "$LOG_FILE"
    fi
    
    if [[ ${#low_freq_duration[@]} -gt 0 ]]; then
        echo "  - CPU 频率低于阈值的核心统计:" | tee -a "$LOG_FILE"
        for core in "${!low_freq_duration[@]}"; do
            if [[ ${low_freq_duration[$core]} -ge $LOW_FREQ_FAIL_COUNT ]]; then
                echo "    核心 $core: 低频持续时间 ${low_freq_duration[$core]} 秒（阈值: ${LOW_FREQ_FAIL_COUNT} 秒）" | tee -a "$LOG_FILE"
            fi
        done
    fi
else
    echo -e "\033[32m✅ 测试通过\033[0m" | tee -a "$LOG_FILE"
    echo
    echo "测试统计:" | tee -a "$LOG_FILE"
    echo "  - 温度超过阈值持续时间: $over_threshold_count 秒（阈值: $TEMP_OVER_DURATION 秒）" | tee -a "$LOG_FILE"
    
    if [[ ${#low_freq_duration[@]} -gt 0 ]]; then
        echo "  - 低频核心统计:" | tee -a "$LOG_FILE"
        for core in "${!low_freq_duration[@]}"; do
            echo "    核心 $core: 低频持续时间 ${low_freq_duration[$core]} 秒" | tee -a "$LOG_FILE"
        done
    else
        echo "  - 所有核心频率正常" | tee -a "$LOG_FILE"
    fi
fi

echo
echo "详细日志已保存至: $LOG_FILE"
echo "========================================"
echo
