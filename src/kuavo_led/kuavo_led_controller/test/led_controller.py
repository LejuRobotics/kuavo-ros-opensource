try:
    import serial
except ImportError:
    import subprocess
    import sys
    print("正在安装pyserial库...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
    import serial
import time
import sys

class LEDController:
    def __init__(self, port='/dev/ttyLED0', baudrate=115200):
        """
        初始化LED控制器
        :param port: 串口设备路径
        :param baudrate: 波特率
        """
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"成功连接到设备: {port}")
        except serial.SerialException as e:
            print(f"无法连接到设备: {e}")
            sys.exit(1)

    def calculate_checksum(self, data):
        """
        计算校验和
        :param data: 数据列表
        :return: 校验和
        """
        return (~sum(data)) & 0xFF

    def set_led_mode(self, mode, colors):
        """
        设置LED灯的模式和颜色
        :param mode: 模式 (0:常亮, 1:呼吸, 2:快闪, 3:律动)
        :param colors: 颜色列表，每个颜色为(R,G,B)元组
        """
        # 构建数据包
        packet = [0xFF, 0xFF, 0x00, 0x22, 0x02, 0x02, mode]
        
        # 添加颜色数据
        for r, g, b in colors:
            packet.extend([r, g, b])
        
        # 计算校验和
        checksum = self.calculate_checksum(packet[2:])
        packet.append(checksum)
        
        # 发送数据
        self.ser.write(bytes(packet))
        print(f"发送数据: {[hex(x) for x in packet]}")

    def close(self):
        """关闭串口连接"""
        self.ser.close()

    def deinit(self):
        self.set_led_mode(0x00, [(0, 0, 0)] * 10)
        self.close()