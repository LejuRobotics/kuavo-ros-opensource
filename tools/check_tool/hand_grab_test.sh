#!/bin/bash
###
 # @Author: dongdongmingming
 # @Date: 2024-05-17 09:56:55
 # @LastEditors: Please set LastEditors
 # @LastEditTime: 2024-05-17 10:00:24
 # @FilePath: /kuavo/tools/check_tool/hand_grab_test.sh
 # @Description: 强脑收测试
### 


# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")


cd $current_script_dir

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:"/home/lab/kuavo_opensource/tools/check_tool/"

./hand_test