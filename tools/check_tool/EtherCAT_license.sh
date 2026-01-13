###
 # @Author: dongdongmingming
 # @Date: 2024-05-06 14:45:05
 # @LastEditors: Please set LastEditors
 # @LastEditTime: 2024-04-18 16:56:38
 # @FilePath: /tools/check_tool/EtherCAT_license.sh
 # @Description: 创建文件导入提供的 license
### 

#!/bin/bash

echo $1

# 定义正则表达式
pattern="^[0-9A-Fa-f]{8}-[0-9A-Fa-f]{8}-[0-9A-Fa-f]{8}$"

# 检查参数是否匹配正则表达式
if [[ $1 =~ $pattern ]]; then
    echo "参数 $1 符合格式。"
else
    echo "参数 $1 不符合格式。"
    exit 1
fi


# 检查路径是否存在
if [ ! -d "/home/lab/.config/lejuconfig/" ]; then
    # 如果路径不存在，创建文件夹
    mkdir -p "/home/lab/.config/lejuconfig/"
    echo "已创建路径 /home/lab/.config/lejuconfig/"
else
    # 如果路径已存在，打印提示并退出
    echo "路径 /home/lab/.config/lejuconfig/ 已存在。"
fi

FILE="/home/lab/.config/lejuconfig/ec_master.key"
# 检查文件是否存在
if [ -f "$FILE" ]; then
    echo "文件 $FILE 已存在，将覆盖写入新的license。"
else
    # 如果文件不存在，创建文件
    touch "$FILE"
    echo "已创建文件 $FILE。"
fi

# 写入license到文件
echo $1 > $FILE
echo "已成功写入license: $1"

