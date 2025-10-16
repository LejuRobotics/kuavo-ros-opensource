import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os
import re

group_type = None

# 定义电机组合（按身体部位分组）
kuavo_motor_groups = [
    # 上半身电机组合
    {12, 19}, {13, 20}, {14, 21}, {15, 22},
    {16, 23}, {17, 24}, {18, 25},
    # 下半身电机组合
    {0, 6}, {1, 7}, {2, 8}, {3, 9}, {4, 10}, {5, 11}
]
roban2_motor_groups = [
    # 上半身电机组合
    {13, 17}, {14, 18}, {15, 19}, {16, 20},
    # 下半身电机组合
    {0, 0},{1, 7}, {2, 8}, {3, 9}, {4, 10}, {5, 11}, {6, 12}
]

motor_groups = []

def load_data(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"文件不存在: {file_path}")
    data = np.loadtxt(file_path)
    valid_mask = data[:, 0] != 0
    timestamps = data[valid_mask, 0]
    values = data[valid_mask, 1]
    if len(timestamps) == 0:
        raise ValueError("文件中无有效数据（时间戳全为0）")
    return timestamps, values

def analyze_motor_data(input_files, response_files):
    global motor_groups

    """分析可用的电机数据，返回有完整数据的电机组合"""
    # 获取所有有数据的电机ID
    available_motors = set()
    for (idx, motor_id) in input_files.keys():
        if (idx, motor_id) in response_files:
            available_motors.add(motor_id)
    
    # 检查每个组合的可用性
    available_groups = []
    missing_groups = []
    
    for group in motor_groups:
        if group.issubset(available_motors):
            available_groups.append(group)
        else:
            missing_motors = group - available_motors
            missing_groups.append((group, missing_motors))
    
    return available_groups, missing_groups, available_motors

def print_data_status(available_groups, missing_groups, available_motors):
    """打印数据状态信息"""
    print("\n" + "="*60)
    print("📊 数据状态分析")
    print("="*60)
    
    # 统计信息
    if group_type == "2":
        total_motors = 21  # roban2有21个待测电机
    else: 
        total_motors = 26  # kuavo有26个待测电机
    missing_motors = total_motors - len(available_motors)
    
    print(f"✅ 可用电机数量: {len(available_motors)}/{total_motors}")
    print(f"❌ 缺失电机数量: {missing_motors}/{total_motors}")
    
    if available_motors:
        print(f"✅ 可用电机ID: {sorted(available_motors)}")
    
    # 显示可用的电机组合
    if available_groups:
        print(f"\n✅ 可绘制的电机组合数量: {len(available_groups)}")
        for i, group in enumerate(available_groups, 1):
            print(f"  组合 {i}: 电机 {sorted(group)}")
    
    # 显示缺失的电机组合
    if missing_groups:
        print(f"\n❌ 缺失数据的电机组合数量: {len(missing_groups)}")
        for group, missing_motors in missing_groups:
            print(f"  组合 {sorted(group)}: 缺失电机 {sorted(missing_motors)}")
    
    # 身体部位分析
    upper_body_motors = set(range(14, 21))  # 12-25
    lower_body_motors = set(range(13))  # 0-11
    
    available_upper = upper_body_motors.intersection(available_motors)
    available_lower = lower_body_motors.intersection(available_motors)
    
    print(f"\n🏃 身体部位分析:")
    print(f"  上半身电机: {len(available_upper)}/{len(upper_body_motors)} 可用")
    print(f"  下半身电机: {len(available_lower)}/{len(lower_body_motors)} 可用")
    
    if not available_upper:
        print("  ⚠️  警告: 没有上半身电机数据")
    if not available_lower:
        print("  ⚠️  警告: 没有下半身电机数据")
    
    print("="*60)

def main():
    global group_type
    global motor_groups
    global roban2_motor_groups
    global kuavo_motor_groups

    # 选择电机组类型
    print("请选择要绘制的电机组类型：")
    print("1. KUAVO")
    print("2. ROBAN2")
    group_type = input("输入 1 或 2 并回车: ").strip()
    if group_type == "2":
        motor_groups = roban2_motor_groups
        print("已选择 ROBAN2 电机组")
    else:
        motor_groups = kuavo_motor_groups
        print("已选择 KUAVO 电机组")

    # 获取当前脚本所在目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 尝试多个可能的数据目录路径
    possible_data_dirs = [
        './file',  # 相对路径（当前工作目录）
        os.path.join(current_dir, 'file'),  # 脚本所在目录下的file
        os.path.join(current_dir, '..', 'motorTest', 'file'),  # 上级目录的motorTest/file
        os.path.join(current_dir, '..', '..', 'motorTest', 'file'),  # 上上级目录的motorTest/file
        # 添加更多可能的路径
        os.path.join(current_dir, '..', '..', '..', 'tools', 'check_tool', 'selfCheckScripts', 'motorTest', 'file'),
        os.path.join(current_dir, '..', '..', '..', '..', 'tools', 'check_tool', 'selfCheckScripts', 'motorTest', 'file'),
        # 添加从项目根目录开始的绝对路径查找
        os.path.join(os.getcwd(), 'tools', 'check_tool', 'selfCheckScripts', 'motorTest', 'file'),
        os.path.join(os.getcwd(), 'file'),  # 当前工作目录下的file
    ]
    
    data_dir = None
    for dir_path in possible_data_dirs:
        if os.path.exists(dir_path):
            data_dir = dir_path
            print(f"找到数据目录: {data_dir}")
            break
    
    if data_dir is None:
        print("❌ 未找到数据目录！")
        print("请确保已经运行了电机跟随性测试，数据文件应该保存在以下路径之一：")
        for i, dir_path in enumerate(possible_data_dirs, 1):
            print(f"  {i}. {dir_path}")
        print("\n💡 建议：")
        print("1. 先运行电机跟随性测试（通过oneKey自检选择选项4）")
        print("2. 等待测试完成并生成数据文件")
        print("3. 然后再运行绘图脚本")
        print("\n🔍 调试信息：")
        print(f"当前脚本路径: {current_dir}")
        print(f"当前工作目录: {os.getcwd()}")
        return
    
    # 检查数据目录中是否有文件
    if not os.listdir(data_dir):
        print(f"警告: 数据目录 {data_dir} 为空，请确保motorFollowTest已经运行并生成了数据文件")
        print("请先运行电机跟随性测试，然后再运行绘图脚本")
        return
    
    input_pattern = re.compile(r'inputData_(\d+)_(\d+)\.txt')
    response_pattern = re.compile(r'responseData_(\d+)_(\d+)\.txt')
    
    input_files = {}
    response_files = {}
    
    for file in os.listdir(data_dir):
        file_path = os.path.join(data_dir, file)
        input_match = input_pattern.search(file)
        if input_match:
            index = int(input_match.group(1))
            motor_id = int(input_match.group(2))
            input_files[(index, motor_id)] = file_path
        
        response_match = response_pattern.search(file)
        if response_match:
            index = int(response_match.group(1))
            motor_id = int(response_match.group(2))
            response_files[(index, motor_id)] = file_path
    
    if not input_files and not response_files:
        print(f"在目录 {data_dir} 中未找到任何数据文件")
        print("请确保motorFollowTest已经运行并生成了inputData_*.txt和responseData_*.txt文件")
        return
    
    # 分析可用的电机数据
    available_groups, missing_groups, available_motors = analyze_motor_data(input_files, response_files)
    
    # 打印数据状态
    print_data_status(available_groups, missing_groups, available_motors)
    
    # 如果没有可用的电机组合，退出
    if not available_groups:
        print("\n❌ 没有找到任何完整的电机组合数据，无法生成图片")
        print("请确保至少有一对电机（输入和响应数据）可用")
        return
    
    print(f"\n🎨 开始绘制 {len(available_groups)} 个电机组合的图片...")
    
    # 按组合绘制图片
    for group_idx, group in enumerate(available_groups):
        # 创建一个大图，包含组合中的所有电机图
        fig, axes = plt.subplots(len(group), 1, figsize=(12, 6 * len(group)))
        if len(group) == 1:
            axes = [axes]  # 确保axes始终是列表
        
        # 记录该组中成功绘制的电机数量
        plotted_count = 0
        
        for motor_id in sorted(group):
            # 查找该电机的所有索引
            motor_indices = set()
            for (idx, mid) in input_files.keys():
                if mid == motor_id:
                    motor_indices.add(idx)
            for (idx, mid) in response_files.keys():
                if mid == motor_id:
                    motor_indices.add(idx)
            
            if not motor_indices:
                print(f"警告: Motor {motor_id} 无数据，跳过")
                continue
            
            # 选择第一个找到的索引
            index = min(motor_indices)
            key = (index, motor_id)
            
            input_file = input_files.get(key)
            response_file = response_files.get(key)
            
            if not input_file or not response_file:
                print(f"警告: Motor {motor_id} 文件不完整，跳过")
                continue
            
            try:
                input_ts, input_val = load_data(input_file)
                resp_ts, resp_val = load_data(response_file)
                
                # 在大图中对应位置绘制子图
                ax = axes[plotted_count]
                ax.plot(input_ts, input_val, 'b-', label='Input')
                ax.plot(resp_ts, resp_val, 'r--', label='Response')
                ax.set_title(f'Motor {motor_id}')
                ax.set_xlabel('Time (ms)' if input_ts.dtype == np.int64 else 'Time (s)')
                ax.set_ylabel('Signal Value')
                ax.grid(True)
                ax.legend()
                
                print(f"成功绘制: 第 {index} 组 Motor {motor_id}，数据点数量: {len(input_ts)}")
                plotted_count += 1
                
            except Exception as e:
                print(f"处理 Motor {motor_id} 时出错: {e}")
        
        # 如果该组有成功绘制的电机，保存大图
        if plotted_count > 0:
            # 调整子图布局
            plt.tight_layout()
            
            # 在脚本所在目录下创建grouped_images文件夹
            output_dir = os.path.join(current_dir, 'grouped_images')
            os.makedirs(output_dir, exist_ok=True)
            
            # 保存图片到脚本所在目录
            output_path = os.path.join(output_dir, f'group_{group_idx + 1}.png')
            plt.savefig(output_path)
            plt.close()
            print(f"✅ 已保存第 {group_idx + 1} 组图片到: {output_path}，包含 {plotted_count} 个电机")
    
    print(f"\n🎉 绘图完成！共生成 {len(available_groups)} 张图片")
    print(f"📁 图片保存在: {os.path.join(current_dir, 'grouped_images')}")

if __name__ == "__main__":
    main()
