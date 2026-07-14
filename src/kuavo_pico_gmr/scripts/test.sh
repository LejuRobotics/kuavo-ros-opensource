#!/bin/bash
# PICO 数据分析测试脚本
# 用于分析 rosbag 中 /pico/retargeted_pose 话题的时延和抖动

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ANALYZE_SCRIPT="${SCRIPT_DIR}/analyze_pico_latency.py"

# 显示帮助
show_help() {
    echo "=========================================="
    echo "  PICO 时延与抖动分析工具"
    echo "=========================================="
    echo ""
    echo "用法: $0 <bag_file> [选项]"
    echo ""
    echo "选项:"
    echo "  --topic, -t <topic>   指定分析话题 (默认: /pico/retargeted_pose)"
    echo "  --hz, -f <freq>       指定期望频率 (默认: 100 Hz)"
    echo "  --output, -o <file>   保存报告到文件"
    echo "  --plot, -p            生成可视化图表"
    echo "  --help, -h            显示帮助"
    echo ""
    echo "示例:"
    echo "  $0 ~/pico_recordings/pico_data.bag"
    echo "  $0 ~/pico_recordings/pico_data.bag --plot"
    echo "  $0 ~/pico_recordings/pico_data.bag -o report.txt --plot"
    echo ""
}

# 检查参数
if [ $# -eq 0 ] || [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    show_help
    exit 0
fi

# 检查分析脚本是否存在
if [ ! -f "${ANALYZE_SCRIPT}" ]; then
    echo "错误: 找不到分析脚本: ${ANALYZE_SCRIPT}"
    exit 1
fi

# 运行分析
echo "正在运行 PICO 时延与抖动分析..."
echo ""

python3 "${ANALYZE_SCRIPT}" "$@"

exit $?
