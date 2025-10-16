#!/bin/bash

# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")
cd $current_script_dir
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:"$current_script_dir/dexhand/"

usage() {
    echo "Usage: $0 [--touch|--normal] [--scan] [--test <test_rounds>]"
    echo "example: $0 --normal --test 3"
}

# 解析参数
test_rounds=3
opt1=""
opt2=""
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --touch|--normal) 
            opt1=$1
            case $2 in
                --test) opt2="--test";test_rounds="${3:-5}"; shift 2 ;;
                --scan) opt2="--scan"; shift ;;
                *) usage; exit 1 ;;
            esac
            ;;
        *) usage; exit 1 ;;
    esac
    shift
done

# echo "opt1: $opt1"
# echo "opt2: $opt2"
# echo "test_rounds: $test_rounds"

if [ "$opt2" = "--scan" ]; then
    ./dexhand/dexhand_test $opt1 $opt2
elif [ "$opt2" = "--test" ]; then
    ./dexhand/dexhand_test $opt1 $opt2 "$test_rounds"
else 
    usage
fi