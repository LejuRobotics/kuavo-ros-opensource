import platform
import os
import minimalmodbus
import time
import logging

# 降低日志噪声
logging.disable(logging.CRITICAL)

def get_pressure_sensor_port(sensor_id=1):
    """
    返回压力传感器端口
    
    参数:
        sensor_id: 传感器编号，1 或 2
            - sensor_id=1: 左边气压表（站号1）
            - sensor_id=2: 右边负压表（站号2）
    """
    # 两个表都使用同一个485设备
    return '/dev/kuavo_pressure'

def read_pressure_kpa(sensor_id=1):
    """
    读取压力传感器 0x0001 寄存器，返回浮点数 kPa
    
    参数:
        sensor_id: 传感器编号，1 或 2
            - sensor_id=1: 左边气压表（站号1）
            - sensor_id=2: 右边负压表（站号2）
    
    返回:
        压力值 (kPa)，如果读取失败返回 None
    """
    # 初始化仪表
    pressure_port = get_pressure_sensor_port(sensor_id)
    # 根据传感器编号设置站号
    slave_address = 1 if sensor_id == 1 else 2
    
    try:
        instrument = minimalmodbus.Instrument(pressure_port, slaveaddress=slave_address)
        # 通讯参数：19200bps、8位、2停止位、无校验
        instrument.serial.baudrate = 19200
        instrument.serial.bytesize = 8
        instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
        instrument.serial.stopbits = 2
        instrument.serial.timeout = 0.5  # 500 ms
        instrument.mode = minimalmodbus.MODE_RTU

        # 关闭 debug 输出
        instrument.close_port_after_each_call = True
        
        # 读取 0x0001 寄存器（气压值）
        raw = instrument.read_register(0x0001, number_of_decimals=0, signed=True)
        # 根据型号转换为实际压力单位（示例：SP15P：0001H 值 - 101~1000 对应 - 0.101~1.000MPa）
        # 这里假设 1 单位 = 0.1 kPa
        kpa = raw * 0.1
        return kpa
    except minimalmodbus.NoResponseError as e:
        print(f"压力传感器 {sensor_id}（站号 {slave_address}）无响应，请检查接线/供电/波特率/地址: {e}")
        return None
    except Exception as e:
        print(f"读取压力传感器 {sensor_id}（站号 {slave_address}）时发生错误: {e}")
        return None

def main():
    """主函数"""
    print("SP系列MODBUS通讯气压表读取程序")
    print("左边气压表：站号1")
    print("右边负压表：站号2")
    print("使用设备：/dev/kuavo_pressure")
    print("通讯参数：19200bps、8位、2停止位、无校验")
    print("-" * 60)
    
    print("开始实时读取压力值（按 Ctrl+C 停止）...")
    print("-" * 60)
    
    try:
        while True:
            # 读取两个压力传感器的值
            val1 = read_pressure_kpa(sensor_id=1)  # 左边气压表（站号1）
            val2 = read_pressure_kpa(sensor_id=2)  # 右边负压表（站号2）
            
            # 显示读取结果
            if val1 is not None and val2 is not None:
                print(f"左边气压（站号1）：{val1:.2f} kPa | 右边负压（站号2）：{val2:.2f} kPa")
            elif val1 is not None:
                print(f"左边气压（站号1）：{val1:.2f} kPa | 右边负压（站号2）：读取失败")
            elif val2 is not None:
                print(f"左边气压（站号1）：读取失败 | 右边负压（站号2）：{val2:.2f} kPa")
            else:
                print("两个压力传感器均读取失败")
            
            # 每秒读取一次
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n停止读取压力值")

if __name__ == "__main__":
    main()