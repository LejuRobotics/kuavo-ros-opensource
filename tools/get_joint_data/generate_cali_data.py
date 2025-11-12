import os
import sys
import yaml
import json
import xml.etree.ElementTree as ET
from math import pi

def parse_urdf_limits(file_path, joint_names):
    """从URDF文件解析给定关节的限位"""
    joint_limits = {}
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        # print(f"Successfully parsed URDF file: {file_path}")
    except FileNotFoundError:
        print(f"Error: URDF file not found at {file_path}")
        sys.exit(1)
    except Exception as e:
        print(f"Error parsing URDF file: {e}")
        sys.exit(1)

    found_joints = 0
    for joint in root.findall('joint'):
        name = joint.get('name')
        if name in joint_names:
            limit_element = joint.find('limit')
            if limit_element is not None:
                lower = float(limit_element.get('lower', 0))
                upper = float(limit_element.get('upper', 0))
                joint_limits[name] = {'lower': lower, 'upper': upper}
                found_joints += 1
                # print(f"Found joint {name}: lower={lower}, upper={upper}")
            else:
                print(f"Warning: Joint {name} has no limit element")
    
    # print(f"Total joints found with limits: {found_joints}")
    return joint_limits

def parse_yaml_offsets(file_path):
    """从YAML文件解析手臂和颈部零点数据"""
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        
        arms_data = data.get('arms_zero_position', [])
        # print(f"Parsed arms_zero.yaml data: {arms_data}")
        # print(f"Number of values in arms_zero.yaml: {len(arms_data)}")
        
        return arms_data
    except Exception as e:
        print(f"Error parsing arms_zero.yaml: {e}")
        return []

def parse_csv_offsets(file_path):
    """从CSV文件解析腿部零点数据（角度转弧度）"""
    offsets = []
    try:
        with open(file_path, 'r') as f:
            for line in f:
                try:
                    # 将角度转换为弧度
                    angle_degrees = float(line.strip())
                    angle_radians = angle_degrees * pi / 180.0
                    offsets.append(angle_radians)
                except ValueError:
                    continue
        # 只返回前14个数据
        return offsets[:14]
    except FileNotFoundError:
        print(f"Error: CSV file not found at {file_path}")
        sys.exit(1)
    except Exception as e:
        print(f"Error parsing CSV file: {e}")
        sys.exit(1)

def parse_negative_motors(config_path):
    """从config.yaml解析反转电机信息"""
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        negative_addresses_raw = config.get('negtive_address', [])
        # print(f"Found negative addresses raw: {negative_addresses_raw}")
        
        # 处理嵌套列表格式，提取实际的十六进制值
        if isinstance(negative_addresses_raw, list) and len(negative_addresses_raw) > 0:
            if isinstance(negative_addresses_raw[0], list):
                # 如果是嵌套列表，取第一个元素
                negative_addresses = negative_addresses_raw[0]
            else:
                negative_addresses = negative_addresses_raw
        else:
            negative_addresses = []
        
        # print(f"Processed negative addresses: {negative_addresses}")
        
        # 创建反转电机映射
        # zarm_l2_joint 到 zarm_l7_joint 对应 0x01 到 0x06
        # zarm_r2_joint 到 zarm_r7_joint 对应 0x07 到 0x0C
        # zhead_1_joint 对应 0x0D
        # zhead_2_joint 对应 0x0E
        negative_motors = {}
        
        # 左臂电机ID映射 (zarm_l2_joint 到 zarm_l7_joint)
        left_arm_ids = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]  # 对应zarm_l2到zarm_l7
        # 右臂电机ID映射 (zarm_r2_joint 到 zarm_r7_joint)  
        right_arm_ids = [0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C]  # 对应zarm_r2到zarm_r7
        # 头部电机ID映射
        head_ids = [0x0D, 0x0E]  # 对应zhead_1和zhead_2
        
        # 检查左臂电机 (从zarm_l2开始)
        for i, motor_id in enumerate(left_arm_ids):
            joint_name = f"zarm_l{i+2}_joint"  # 从l2开始
            if motor_id in negative_addresses:
                negative_motors[joint_name] = 1
                # print(f"Left arm motor {joint_name} (ID: 0x{motor_id:02X}) is negative")
            else:
                negative_motors[joint_name] = 0
                # print(f"Left arm motor {joint_name} (ID: 0x{motor_id:02X}) is positive")
        
        # 检查右臂电机 (从zarm_r2开始)
        for i, motor_id in enumerate(right_arm_ids):
            joint_name = f"zarm_r{i+2}_joint"  # 从r2开始
            if motor_id in negative_addresses:
                negative_motors[joint_name] = 1
                # print(f"Right arm motor {joint_name} (ID: 0x{motor_id:02X}) is negative")
            else:
                negative_motors[joint_name] = 0
                # print(f"Right arm motor {joint_name} (ID: 0x{motor_id:02X}) is positive")
        
        # 检查头部电机
        for i, motor_id in enumerate(head_ids):
            joint_name = f"zhead_{i+1}_joint"
            if motor_id in negative_addresses:
                negative_motors[joint_name] = 1
                # print(f"Head motor {joint_name} (ID: 0x{motor_id:02X}) is negative")
            else:
                negative_motors[joint_name] = 0
                # print(f"Head motor {joint_name} (ID: 0x{motor_id:02X}) is positive")
        
        return negative_motors
        
    except Exception as e:
        print(f"Error parsing config.yaml: {e}")
        return {}

def generate_calibration_json():
    """生成并保存标定信息JSON文件"""

    print("因为每个机器的零点略有差异，需要使用该脚本在每个机器上读取零点文件后生成 json 文件")
    print("特别注意：如果机器人零点文件有改动，需要重新生成 json 文件")
    print("使用前需要确认仓库路径，修改 urdf_path 变量")

    robot_version = os.environ.get('ROBOT_VERSION', '49')
    print(f"Robot version detected: {robot_version}")
    
    # 展开用户主目录路径
    urdf_path = os.path.expanduser(f'~/kuavo-ros-opensource/src/kuavo_assets/models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf')
    arms_zero_yaml_path = os.path.expanduser('~/.config/lejuconfig/arms_zero.yaml')
    offset_csv_path = os.path.expanduser('~/.config/lejuconfig/offset.csv')
    config_path = os.path.expanduser('~/.config/lejuconfig/config.yaml')
    output_json_path = os.path.expanduser('~/robot_calibration.json')

    # 定义所有需要处理的关节名称，并严格按照新给出的顺序
    joint_names_ordered = [
        "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
        "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint", "leg_r6_joint",
        "zarm_l1_joint",
        "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint", "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",
        "zarm_r1_joint",
        "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint", "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint",
        "zhead_1_joint", "zhead_2_joint"
    ]
    
    # 获取零偏数据
    leg_offsets = parse_csv_offsets(offset_csv_path)  # 14个数据：1-6左腿，7-12右腿，13左肩，14右肩
    arms_and_head_offsets = parse_yaml_offsets(arms_zero_yaml_path)  # 14个数据：1-6左臂，7-12右臂，13-14颈部

    # 按照joint_names_ordered的顺序正确映射数据
    all_offsets_mapped = []
    
    # 腿部数据 (0-11)
    all_offsets_mapped.extend(leg_offsets[0:12])  # leg_l1到leg_l6, leg_r1到leg_r6
    
    # 左肩部电机 (12)
    all_offsets_mapped.append(leg_offsets[12])  # zarm_l1_joint
    
    # 左臂数据 (13-18)
    all_offsets_mapped.extend(arms_and_head_offsets[0:6])  # zarm_l2到zarm_l7
    
    # 右肩部电机 (19)
    all_offsets_mapped.append(leg_offsets[13])  # zarm_r1_joint
    
    # 右臂数据 (20-25)
    all_offsets_mapped.extend(arms_and_head_offsets[6:12])  # zarm_r2到zarm_r7
    
    # 头部数据 (26-27)
    all_offsets_mapped.extend(arms_and_head_offsets[12:14])  # zhead_1, zhead_2

    # print(f"Mapped offsets: {all_offsets_mapped}")
    # print(f"Total mapped offsets: {len(all_offsets_mapped)}")

    # 从URDF文件中获取限位数据
    urdf_limits = parse_urdf_limits(urdf_path, joint_names_ordered)
    
    # 获取反转电机信息
    negative_motors = parse_negative_motors(config_path)

    calibration_data = {}
    for i, joint_name in enumerate(joint_names_ordered):
        homing_offset = all_offsets_mapped[i] if i < len(all_offsets_mapped) else 0.0
        limits = urdf_limits.get(joint_name, {'lower': 0, 'upper': 0})
        
        # 确定drive_mode：如果是反转电机则为1，否则为0
        drive_mode = negative_motors.get(joint_name, 0)
        
        calibration_data[joint_name] = {
            "id": i,  # 从0开始
            "drive_mode": drive_mode,
            "homing_offset": homing_offset,
            "range_min": limits['lower'],
            "range_max": limits['upper']
        }

    # 写入JSON文件
    with open(output_json_path, 'w') as f:
        json.dump(calibration_data, f, indent=2)

    print(f"Successfully generated calibration JSON file at {output_json_path}")


if __name__ == '__main__':
    generate_calibration_json()