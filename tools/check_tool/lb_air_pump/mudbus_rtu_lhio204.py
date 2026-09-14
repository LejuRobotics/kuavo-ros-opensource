import argparse
import serial
import time

# -------------------------- 配置区 --------------------------
# 串口参数：9600 8N1（无校验、8数据位、1停止位）
SERIAL_BAUDRATE = 9600
SERIAL_BYTESIZE = serial.EIGHTBITS
SERIAL_PARITY = serial.PARITY_NONE
SERIAL_STOPBITS = serial.STOPBITS_ONE
SERIAL_TIMEOUT = 0.5  # 超时时间，单位秒

SLAVE_ADDRESS = 0xFE
WRITE_MULTIPLE_COILS = 0x0F
COIL_START_ADDRESS = 0x0000
COIL_QUANTITY = 0x0002
MODBUS_RESPONSE_LENGTH = 8


def _calc_modbus_crc(data):
    """计算 Modbus RTU CRC16，返回低字节在前的两个 CRC 字节。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_relay_command(channel=None):
    """构造继电器控制指令。

    channel=1: 只打开第1路
    channel=2: 只打开第2路
    channel=None: 两路都关闭
    """
    relay_bits = {
        None: 0x00,
        1: 0x01,
        2: 0x02,
    }
    if channel not in relay_bits:
        raise ValueError("channel 只能是 1、2 或 None")

    payload = bytes([
        SLAVE_ADDRESS,
        WRITE_MULTIPLE_COILS,
        (COIL_START_ADDRESS >> 8) & 0xFF,
        COIL_START_ADDRESS & 0xFF,
        (COIL_QUANTITY >> 8) & 0xFF,
        COIL_QUANTITY & 0xFF,
        0x01,
        relay_bits[channel],
    ])
    return payload + _calc_modbus_crc(payload)


# -------------------------- 核心控制类 --------------------------
class LHIO204RelayController:
    def __init__(self, port=None):
        """初始化控制器，自动识别串口"""
        self.port = '/dev/kuavo_relay'
        self.ser = None
        self._connect()

    def _connect(self):
        """建立串口连接"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=SERIAL_BAUDRATE,
                bytesize=SERIAL_BYTESIZE,
                parity=SERIAL_PARITY,
                stopbits=SERIAL_STOPBITS,
                timeout=SERIAL_TIMEOUT
            )
            print(f"✅ 串口连接成功：{self.port}")
        except Exception as e:
            print(f"❌ 串口连接失败：{str(e)}")
            raise

    def _send_command(self, command, action_text):
        """发送继电器指令并读取响应。"""
        if not self.ser or not self.ser.is_open:
            print("❌ 串口未连接，无法发送指令")
            return False
        try:
            self.ser.write(command)
            response = self.ser.read(MODBUS_RESPONSE_LENGTH)
            if not response:
                print(f"❌ {action_text}失败：无响应")
                return False
            print(f"✅ {action_text}成功，响应：{response.hex().upper()}")
            return True
        except Exception as e:
            print(f"❌ {action_text}失败：{str(e)}")
            return False

    def select_channel(self, channel):
        """选择接通哪一路继电器，只允许第1路或第2路接通。"""
        command = build_relay_command(channel)
        return self._send_command(command, f"切换到第{channel}路")

    def relay_all_off(self):
        """继电器1、2路同时释放（全关）"""
        command = build_relay_command(None)
        return self._send_command(command, "发送全关指令")

    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 串口已关闭")


# -------------------------- 测试主程序 --------------------------
def parse_args():
    parser = argparse.ArgumentParser(description="LHIO204 两路继电器控制")
    parser.add_argument(
        "--channel",
        type=int,
        choices=[1, 2],
        default=1,
        help="选择接通的继电器通道，只能是 1 或 2，默认 1",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    # 初始化控制器（使用默认串口 /dev/kuavo_relay）
    controller = LHIO204RelayController()
    try:
        print(f"\n=== 执行：接通第{args.channel}路继电器 ===")
        if controller.select_channel(args.channel):
            print("\n按 Ctrl+C 退出，并执行安全全关")
            while True:
                time.sleep(1)  # 保持程序运行，维持当前继电器状态
    except KeyboardInterrupt:
        print("\n\n⚠️  程序被手动终止，执行安全全关")
    finally:
        controller.relay_all_off()
        controller.close()
# python3 /home/lab/kuavo-ros-control/tools/check_tool/mudbus_test/mudbus_rtu_lhio204.py --channel 2
