# 定义颜色代码
GREEN='\033[32m'
RED='\033[31m'
NC='\033[0m' # No Color

# 复制规则文件到指定目录
cp hipnuc_imu_serial.rules /etc/udev/rules.d/
if [ $? -eq 0 ]; then
    echo -e "${GREEN}[成功] 复制 hipnuc_imu_serial.rules 到 /etc/udev/rules.d/${NC}"
else
    echo -e "${RED}[失败] 复制 hipnuc_imu_serial.rules 到 /etc/udev/rules.d/${NC}"
fi

# 重新加载 udev 规则
sudo udevadm control --reload-rules
if [ $? -eq 0 ]; then
    echo -e "${GREEN}[成功] 重新加载 udev 规则${NC}"
else
    echo -e "${RED}[失败] 重新加载 udev 规则${NC}"
fi

# 触发 udev 规则
sudo udevadm trigger
if [ $? -eq 0 ]; then
    echo -e "${GREEN}[成功] 触发 udev 规则${NC}"
else
    echo -e "${RED}[失败] 触发 udev 规则${NC}"
fi

# 重新加载 udev 服务
sudo service udev reload
if [ $? -eq 0 ]; then
    echo -e "${GREEN}[成功] 重新加载 udev 服务${NC}"
else
    echo -e "${RED}[失败] 重新加载 udev 服务${NC}"
fi

# 重启 udev 服务
sudo service udev restart
if [ $? -eq 0 ]; then
    echo -e "${GREEN}[成功] 重启 udev 服务${NC}"
else
    echo -e "${RED}[失败] 重启 udev 服务${NC}"
fi