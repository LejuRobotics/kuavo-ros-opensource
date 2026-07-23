import serial
import time
import subprocess
import os

# 继电器指令定义
RELAY_FULL_ON = bytes.fromhex('FE0F0000000201FFD1D3')
RELAY_FULL_OFF = bytes.fromhex('FE0F0000000201009193')
RELAY_RESPONSE = bytes.fromhex('FE0F00000002C005')

# 压力传感器配置
PRESSURE_BAUDRATE = 19200
PRESSURE_TIMEOUT = 0.5


def list_serial_ports():
    """列出所有可用的串口"""
    ports = []
    # Linux系统
    for i in range(10):
        port = f"/dev/ttyUSB{i}"
        if os.path.exists(port):
            ports.append(port)
    return ports


def test_relay(port):
    """测试是否为继电器"""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=9600,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=1,
            timeout=1
        )
        
        # 发送全关指令（避免继电器吸合）
        ser.write(RELAY_FULL_OFF)
        time.sleep(0.1)
        
        # 读取响应
        response = ser.read(8)  # 期望8字节响应
        ser.close()
        
        # 检查响应是否匹配
        if response == RELAY_RESPONSE:
            print(f"✓ {port} 检测为继电器")
            return True
        else:
            print(f"✗ {port} 不是继电器 (响应: {response.hex() if response else '无'})")
            return False
    except Exception as e:
        print(f"✗ {port} 测试继电器失败: {e}")
        return False


def test_pressure_sensor(port):
    """测试是否为压力传感器"""
    try:
        import minimalmodbus
        instrument = minimalmodbus.Instrument(port, slaveaddress=1)
        instrument.serial.baudrate = PRESSURE_BAUDRATE
        instrument.serial.bytesize = 8
        instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
        instrument.serial.stopbits = 2
        instrument.serial.timeout = PRESSURE_TIMEOUT
        instrument.mode = minimalmodbus.MODE_RTU
        instrument.close_port_after_each_call = True
        
        # 尝试读取寄存器
        raw = instrument.read_register(0x0001, number_of_decimals=0, signed=True)
        print(f"✓ {port} 检测为压力传感器 (读取值: {raw})")
        return True
    except ImportError:
        print(f"✗ {port} 无法测试压力传感器: 缺少 minimalmodbus 库")
        return False
    except Exception as e:
        print(f"✗ {port} 不是压力传感器: {e}")
        return False


def configure_udev_rule(port, device_name):
    """配置udev规则"""
    print(f"正在为 {port} 创建固定映射到 /dev/{device_name}...")
    
    # 检查目标端口是否存在
    if not os.path.exists(port):
        print(f"错误: {port} 不存在")
        return False
    
    # 获取设备的物理 USB 端口位置（KERNELS）
    try:
        # 查找带小数点的物理端口路径
        cmd = f"udevadm info --attribute-walk --name={port} | grep -m 1 'KERNELS==\"[0-9]*-[0-9]*\\.[0-9]*\"' | cut -d'\"' -f2"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        kernels = result.stdout.strip()
        
        # 如果没找到带小数点的，尝试找短格式
        if not kernels:
            cmd = f"udevadm info --attribute-walk --name={port} | grep -m 1 'KERNELS==\"[0-9]*-[0-9]*\"' | cut -d'\"' -f2"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            kernels = result.stdout.strip()
        
        if not kernels:
            # 尝试使用 ID_SERIAL_SHORT
            cmd = f"udevadm info --query=property --name={port} | grep 'ID_SERIAL_SHORT=' | cut -d= -f2"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            serial_short = result.stdout.strip()
            
            if serial_short and ':' not in serial_short:
                print(f"使用序列号: {serial_short}")
                rule = f'KERNEL=="ttyUSB*", ENV{{ID_SERIAL_SHORT}}=="{serial_short}", MODE:="0666", SYMLINK+="{device_name}"'
            else:
                print("错误: 无法获取有效的设备标识信息")
                return False
        else:
            print(f"锁定物理 USB 端口: {kernels}")
            print("注意: 设备必须插在同一个 USB 接口上才能保持映射")
            rule = f'KERNEL=="ttyUSB*", KERNELS=="{kernels}", MODE:="0666", SYMLINK+="{device_name}"'
    except Exception as e:
        print(f"错误: 获取设备信息失败: {e}")
        return False
    
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
    print("开始自动检测设备并配置udev规则...")
    print("=" * 60)
    
    # 列出所有可用串口
    ports = list_serial_ports()
    if not ports:
        print("错误: 未找到可用的串口设备")
        return
    
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
    
    # 配置udev规则
    if relay_port:
        configure_udev_rule(relay_port, 'kuavo_relay')
    
    if pressure_port:
        configure_udev_rule(pressure_port, 'kuavo_pressure')
    
    print("\n" + "=" * 60)
    print("配置完成！")
    print("现在可以使用以下设备名访问:")
    print("  - 继电器: /dev/kuavo_relay")
    print("  - 压力传感器: /dev/kuavo_pressure")


if __name__ == "__main__":
    main()