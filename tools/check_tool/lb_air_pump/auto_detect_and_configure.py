import serial
import time
import subprocess
import os
import re
import glob
import sys

# 继电器指令定义（检测用全关指令，避免继电器吸合）
CMD_RELAY_ALL_OFF = bytes.fromhex('FE0F0000000201009193')
# 全关指令的正确反馈帧（继电器模块实际返回，8字节）
RELAY_OFF_RESPONSE = bytes.fromhex('FE0F00000002C005')

# 压力传感器配置
PRESSURE_BAUDRATE = 19200
PRESSURE_TIMEOUT = 0.5


def list_serial_ports():
    """列出所有可用的串口（按编号自然排序，避免 ttyUSB10 排在 ttyUSB2 前）"""
    def sort_key(path):
        m = re.search(r'(\d+)$', path)
        return int(m.group(1)) if m else 0
    return sorted(glob.glob('/dev/ttyUSB*'), key=sort_key)


def test_relay(port):
    """测试是否为继电器"""
    ser = None
    try:
        ser = serial.Serial(
            port=port,
            baudrate=9600,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=1,
            timeout=1
        )
        
        # 清空输入缓冲区，避免读到残留脏数据
        ser.reset_input_buffer()
        
        # 发送全关指令
        ser.write(CMD_RELAY_ALL_OFF)
        time.sleep(0.1)
        
        # 读取响应
        response = ser.read(len(RELAY_OFF_RESPONSE))
        
        # 校验是否为全关指令的正确反馈帧
        if response == RELAY_OFF_RESPONSE:
            print(f"✓ {port} 检测为继电器 (响应: {response.hex().upper()})")
            return True
        else:
            print(f"✗ {port} 不是继电器 (响应: {response.hex().upper() if response else '无'})")
            return False
    except Exception as e:
        print(f"✗ {port} 测试继电器失败: {e}")
        return False
    finally:
        # 无论成功还是异常，都确保关闭串口，避免句柄泄漏影响后续检测
        if ser is not None and ser.is_open:
            ser.close()


def test_pressure_sensor(port):
    """测试是否为压力传感器"""
    try:
        import minimalmodbus
    except ImportError:
        print(f"✗ {port} 无法测试压力传感器: 缺少 minimalmodbus 库")
        return False

    instrument = None
    try:
        instrument = minimalmodbus.Instrument(port, slaveaddress=1)
        instrument.serial.baudrate = PRESSURE_BAUDRATE
        instrument.serial.bytesize = 8
        instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
        instrument.serial.stopbits = 2
        instrument.serial.timeout = PRESSURE_TIMEOUT
        instrument.mode = minimalmodbus.MODE_RTU
        instrument.close_port_after_each_call = True
        
        # 清空输入缓冲区，避免读到继电器检测时留下的残留数据
        instrument.serial.reset_input_buffer()
        
        # 尝试读取寄存器
        raw = instrument.read_register(0x0001, number_of_decimals=0, signed=True)
        print(f"✓ {port} 检测为压力传感器 (读取值: {raw})")
        return True
    except Exception as e:
        print(f"✗ {port} 不是压力传感器: {e}")
        return False
    finally:
        # 确保串口被关闭，避免句柄泄漏
        try:
            if instrument is not None:
                instrument.serial.close()
        except Exception:
            pass


def get_usb_port(port):
    """获取串口对应的 USB 物理端口路径（如 1-4.2.1）

    通过 sysfs 找到最深的 USB 设备端口，支持多级集线器。
    避免用 grep 抓取导致多个设备拿到同一个父级集线器端口。

    注意：相比旧实现，这里不再提供「按 ID_SERIAL_SHORT 序列号绑定」的回退。
    标准 USB 拓扑下 sysfs 总存在 N-M 形式的端口段，对现有 ttyUSB 设备无影响；
    若未来接入 PCIe 串口卡、特殊总线适配器等拿不到端口段的设备，需在此补回退逻辑。
    """
    try:
        sysfs = os.path.realpath(f"/sys/class/tty/{os.path.basename(port)}")
        parts = sysfs.split('/')
        # 从里向外找最深的 USB 设备端口（形如 1-4.2.1，不含带冒号的接口段）
        for part in reversed(parts):
            if re.match(r'^[0-9]+-[0-9]+(\.[0-9]+)*$', part):
                return part
    except Exception as e:
        print(f"错误: 获取 USB 端口失败: {e}")
    return None


def configure_udev_rule(port, device_name):
    """配置udev规则"""
    print(f"正在为 {port} 创建固定映射到 /dev/{device_name}...")
    
    # 检查目标端口是否存在
    if not os.path.exists(port):
        print(f"错误: {port} 不存在")
        return False
    
    # 获取设备的物理 USB 端口位置（KERNELS）
    kernels = get_usb_port(port)
    if not kernels:
        print("错误: 无法获取有效的设备标识信息")
        return False
    
    print(f"锁定物理 USB 端口: {kernels}")
    print("注意: 设备必须插在同一个 USB 接口上才能保持映射")
    rule = f'KERNEL=="ttyUSB*", KERNELS=="{kernels}", MODE:="0666", SYMLINK+="{device_name}"'
    
    rule_file = "/etc/udev/rules.d/99-kuavo-relay.rules"
    print("----------------------------------------")
    print(f"规则文件: {rule_file}")
    print(f"规则内容: {rule}")
    
    # 检查规则文件是否存在，如果不存在则创建
    if not os.path.exists(rule_file):
        print(f"创建新的规则文件: {rule_file}")
        try:
            with open(rule_file, 'w') as f:
                pass
        except Exception as e:
            print(f"错误: 创建规则文件失败: {e}")
            return False
    
    # 检查是否已存在相同的规则
    try:
        with open(rule_file, 'r') as f:
            content = f.read()
        
        if f'SYMLINK+="{device_name}"' in content:
            print(f"警告: 规则文件中已存在符号链接 {device_name} 的规则")
            print("替换现有规则...")
            # 删除旧的规则行
            lines = content.split('\n')
            new_lines = [line for line in lines if f'SYMLINK+="{device_name}"' not in line]
            content = '\n'.join(new_lines)
            with open(rule_file, 'w') as f:
                f.write(content)
            print("已删除旧的规则")
    except Exception as e:
        print(f"错误: 读取规则文件失败: {e}")
        return False
    
    # 追加新规则到文件末尾
    try:
        with open(rule_file, 'a') as f:
            f.write(rule + '\n')
        print(f"已添加规则到 {rule_file}")
    except Exception as e:
        print(f"错误: 写入规则文件失败: {e}")
        return False
    
    # 显示当前规则文件的所有内容
    print("----------------------------------------")
    print("当前规则文件内容:")
    try:
        with open(rule_file, 'r') as f:
            print(f.read())
    except Exception as e:
        print(f"错误: 读取规则文件失败: {e}")
    print("----------------------------------------")
    
    # 重载 udev 规则
    print("正在重载 udev 规则...")
    try:
        subprocess.run("udevadm control --reload-rules", shell=True, check=True)
        subprocess.run("udevadm trigger", shell=True, check=True)
        # 等待一下让规则生效
        time.sleep(1)
    except Exception as e:
        print(f"错误: 重载 udev 规则失败: {e}")
        return False
    
    # 验证规则是否生效
    print("----------------------------------------")
    symlink_path = f"/dev/{device_name}"
    if os.path.islink(symlink_path):
        try:
            target = os.path.realpath(symlink_path)
            print(f"成功！已生成固定端口: {symlink_path}")
            print(f"  {symlink_path} -> {target}")
            if target == port:
                print(f"验证通过: 链接正确指向 {port}")
            else:
                print(f"注意: 链接指向 {target} (期望 {port})")
                print("如果设备刚重连，请重新插拔设备或重启系统")
            print("")
            print(f"现在可以使用 {symlink_path} 访问设备")
            return True
        except Exception as e:
            print(f"错误: 验证链接失败: {e}")
            return False
    else:
        print(f"警告: 未能立即生成 {symlink_path}")
        print("请尝试重新插拔设备或重启系统以使规则生效")
        return False


def main():
    # 写 udev 规则和运行 udevadm 都需要 root 权限
    if os.geteuid() != 0:
        print("错误: 需要 root 权限来写入 /etc/udev/rules.d 并运行 udevadm")
        print("请使用: sudo python3 auto_detect_and_configure.py")
        sys.exit(1)
    
    print("开始自动检测设备并配置udev规则...")
    print("=" * 60)
    
    # 列出所有可用串口
    ports = list_serial_ports()
    if not ports:
        print("错误: 未找到可用的串口设备")
        sys.exit(1)
    
    print(f"找到 {len(ports)} 个可用串口:")
    for port in ports:
        print(f"  - {port}")
    print("=" * 60)
    
    # 检测每个串口
    relay_port = None
    pressure_port = None
    
    for port in ports:
        print(f"\n测试 {port}:")
        
        # 先测试继电器
        if test_relay(port):
            relay_port = port
        # 再测试压力传感器
        elif test_pressure_sensor(port):
            pressure_port = port
    
    print("\n" + "=" * 60)
    print("检测结果:")
    print(f"继电器端口: {relay_port if relay_port else '未找到'}")
    print(f"压力传感器端口: {pressure_port if pressure_port else '未找到'}")
    print("=" * 60)
    
    # 两个设备都未检测到时，明确报错并以非零码退出，避免误导为成功
    if not relay_port and not pressure_port:
        print("\n" + "=" * 60)
        print("错误: 未检测到任何继电器或压力传感器设备")
        print("请检查设备连接、串口驱动和供电")
        print("=" * 60)
        sys.exit(1)

    # 配置udev规则
    if relay_port:
        configure_udev_rule(relay_port, 'kuavo_relay')
    
    if pressure_port:
        configure_udev_rule(pressure_port, 'kuavo_pressure')
    
    print("\n" + "=" * 60)
    print("配置完成！")
    print("现在可以使用以下设备名访问:")
    print(f"  - 继电器: /dev/kuavo_relay {'✅ 已找到: ' + relay_port if relay_port else '❌ 未找到'}")
    print(f"  - 压力传感器: /dev/kuavo_pressure {'✅ 已找到: ' + pressure_port if pressure_port else '❌ 未找到'}")


if __name__ == "__main__":
    main()