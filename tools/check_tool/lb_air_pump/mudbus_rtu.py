import serial
import time
import glob
import os

# -------------------------- 配置区 --------------------------
# 串口参数：9600 8N1（无校验、8数据位、1停止位）
SERIAL_BAUDRATE = 9600
SERIAL_BYTESIZE = serial.EIGHTBITS
SERIAL_PARITY = serial.PARITY_NONE
SERIAL_STOPBITS = serial.STOPBITS_ONE
SERIAL_TIMEOUT = 0.5  # 超时时间，单位秒

# 继电器控制指令（直接复制上位机真实指令，无需计算CRC）
CMD_RELAY_ALL_ON = bytes([0xFE, 0x0F, 0x00, 0x00, 0x00, 0x02, 0x01, 0x03, 0xD1, 0x92])  # 全开
CMD_RELAY_ALL_OFF = bytes([0xFE, 0x0F, 0x00, 0x00, 0x00, 0x02, 0x01, 0x00, 0x91, 0x93]) # 全关

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

    def relay_all_on(self):
        """继电器1、2路同时吸合（全开）"""
        if not self.ser or not self.ser.is_open:
            print("❌ 串口未连接，无法发送指令")
            return False
        try:
            self.ser.write(CMD_RELAY_ALL_ON)
            # 读取模块返回的响应（上位机RX数据）
            response = self.ser.read(9)  # 响应长度固定9字节
            if not response:
                print("❌ 发送全开指令失败：无响应")
                return False
            print(f"✅ 发送全开指令成功，响应：{response.hex().upper()}")
            return True
        except Exception as e:
            print(f"❌ 发送全开指令失败：{str(e)}")
            return False

    def relay_all_off(self):
        """继电器1、2路同时释放（全关）"""
        if not self.ser or not self.ser.is_open:
            print("❌ 串口未连接，无法发送指令")
            return False
        try:
            self.ser.write(CMD_RELAY_ALL_OFF)
            # 读取模块返回的响应（上位机RX数据）
            response = self.ser.read(9)  # 响应长度固定9字节
            if not response:
                print("❌ 发送全关指令失败：无响应")
                return False
            print(f"✅ 发送全关指令成功，响应：{response.hex().upper()}")
            return True
        except Exception as e:
            print(f"❌ 发送全关指令失败：{str(e)}")
            return False

    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 串口已关闭")

# -------------------------- 测试主程序 --------------------------
if __name__ == "__main__":
    # 初始化控制器（使用默认串口 /dev/kuavo_relay）
    controller = LHIO204RelayController()
    
    try:
        # 启动时执行全开
        print("\n=== 执行：继电器全开 ===")
        controller.relay_all_on()
        print("\n✅ 继电器已保持全开状态")
        print("⚠️  按 Ctrl+C 退出程序并全关继电器")
        
        # 进入无限循环，保持程序运行
        while True:
            time.sleep(1)  # 避免CPU占用过高
            
    except KeyboardInterrupt:
        print("\n\n⚠️  程序被手动终止，执行安全全关")
        controller.relay_all_off()
        controller.close()