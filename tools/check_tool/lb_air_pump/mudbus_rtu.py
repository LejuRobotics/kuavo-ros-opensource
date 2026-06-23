import serial
import time
import glob
import os
import argparse

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
CMD_RELAY_CH2_ON = bytes([0xFE, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xC9, 0xF5])  # 第二路继电器闭合

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

    def relay_ch2_on(self):
        """第二路继电器闭合"""
        if not self.ser or not self.ser.is_open:
            print("❌ 串口未连接，无法发送指令")
            return False
        try:
            self.ser.write(CMD_RELAY_CH2_ON)
            # 读取模块返回的响应（上位机RX数据）
            response = self.ser.read(8)  # 响应长度固定8字节
            if not response:
                print("❌ 发送第二路闭合指令失败：无响应")
                return False
            print(f"✅ 发送第二路闭合指令成功，响应：{response.hex().upper()}")
            return True
        except Exception as e:
            print(f"❌ 发送第二路闭合指令失败：{str(e)}")
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
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='继电器控制程序')
    parser.add_argument('-m', '--mode', type=str, choices=['inhale', 'blow'], 
                        help='操作模式: inhale=左右一起正吸气, blow=左右一起反喷气')
    args = parser.parse_args()
    
    # 初始化控制器（使用默认串口 /dev/kuavo_relay）
    controller = LHIO204RelayController()
    
    try:
        # 如果有命令行参数，直接执行对应模式
        if args.mode:
            choice = args.mode
        else:
            # 显示菜单选项
            print("\n" + "="*50)
            print("继电器控制程序")
            print("="*50)
            print("1. 左右一起正吸气（仅第2路闭合）")
            print("2. 左右一起反喷气（1、2路同时吸合）")
            print("0. 退出程序")
            print("="*50)
            
            menu_choice = input("请选择操作模式 [1/2/0]: ").strip()
            # 将菜单选择转换为模式
            if menu_choice == "1":
                choice = "inhale"
            elif menu_choice == "2":
                choice = "blow"
            elif menu_choice == "0":
                choice = "exit"
            else:
                choice = "invalid"
        
        if choice == "inhale":
            # 执行第二路闭合（正吸气）
            print("\n=== 执行：左右一起正吸气 ===")
            controller.relay_ch2_on()
            print("\n✅ 左右一起正吸气已启动")
            print("⚠️  按 Ctrl+C 退出程序并全关继电器")
            
            # 进入无限循环，保持程序运行
            while True:
                time.sleep(1)  # 避免CPU占用过高
                
        elif choice == "blow":
            # 执行全开（反喷气）
            print("\n=== 执行：左右一起反喷气 ===")
            controller.relay_all_on()
            print("\n✅ 左右一起反喷气已启动")
            print("⚠️  按 Ctrl+C 退出程序并全关继电器")
            
            # 进入无限循环，保持程序运行
            while True:
                time.sleep(1)  # 避免CPU占用过高
                
        elif choice == "exit":
            print("\n 程序已退出")
            controller.close()
            
        else:
            print("\n❌ 无效选择，程序退出")
            controller.close()
            
    except KeyboardInterrupt:
        print("\n\n⚠️  程序被手动终止，执行安全全关")
        controller.relay_all_off()
        controller.close()