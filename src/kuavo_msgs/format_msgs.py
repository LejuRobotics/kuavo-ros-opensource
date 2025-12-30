#!/usr/bin/env python3
import os
import re

def camel_to_snake(name):
    """将 camelCase 或 PascalCase 转换为 snake_case，并确保首字母小写"""
    # 先处理第一个单词
    first_word = name[0].lower() + name[1:]
    # 然后处理剩余的驼峰
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', first_word)
    s2 = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1)
    return s2.lower()

def capitalize_type(type_name):
    """确保自定义消息类型首字母大写，但保持基本类型的原始大小写"""
    # 基本类型保持原始大小写
    basic_types = {
        'bool', 'byte', 'char',
        'float32', 'float64',
        'int8', 'uint8',
        'int16', 'uint16',
        'int32', 'uint32',
        'int64', 'uint64',
        'string', 'wstring'
    }
    
    # 处理数组标记
    array_suffix = ''
    # 匹配 [] 或 [数字]
    array_match = re.search(r'(\[\d*\])', type_name)
    if array_match:
        array_suffix = array_match.group(1)
        type_name = type_name.replace(array_suffix, '')
    
    # 如果是基本类型，统一转换为小写
    if type_name.lower() in {t.lower() for t in basic_types}:
        return type_name.lower() + array_suffix
    
    # 处理可能包含命名空间的类型
    parts = type_name.split('/')
    if len(parts) > 1:
        # 保持命名空间小写，类型名首字母大写
        return '/'.join(parts[:-1] + [parts[-1][0].upper() + parts[-1][1:]]) + array_suffix
    
    # 自定义类型首字母大写
    return type_name[0].upper() + type_name[1:] + array_suffix

def convert_fields_to_snake_case(file_path):
    """转换文件中的字段名为 snake_case 并确保类型名称正确"""
    with open(file_path, 'r') as f:
        content = f.read()
    
    modified = False
    new_lines = []
    
    for line in content.splitlines():
        # 跳过注释和空行
        if not line.strip() or line.strip().startswith('#'):
            new_lines.append(line)
            continue
        
        # 修正 Header 和 Time 类型
        if 'kuavo_msgs/Header' in line:
            line = line.replace('kuavo_msgs/Header', 'std_msgs/Header')
            modified = True
        elif 'kuavo_msgs/Time' in line:
            line = line.replace('kuavo_msgs/Time', 'builtin_interfaces/Time')
            modified = True
        elif 'Header' in line and not 'std_msgs/Header' in line:
            line = line.replace('Header', 'std_msgs/Header')
            modified = True
        elif 'Time' in line and not 'builtin_interfaces/Time' in line and not line.strip().startswith('#'):
            line = line.replace('Time', 'builtin_interfaces/Time')
            modified = True
        elif re.search(r'\btime\b', line, re.IGNORECASE) and not 'builtin_interfaces/Time' in line and not line.strip().startswith('#'):
            line = re.sub(r'\btime\b', 'builtin_interfaces/Time', line, flags=re.IGNORECASE)
            modified = True
        
        # 处理字段定义
        # 使用正则表达式匹配，以处理多个空格的情况
        match = re.match(r'^(\s*)([a-zA-Z0-9_/]+(?:\[\d*\])?)\s+([a-zA-Z][a-zA-Z0-9_]*)(.*?)$', line.strip())
        if match:
            indent = line[:len(line) - len(line.lstrip())]
            type_part = match.group(2)
            field_name = match.group(3)
            rest = match.group(4)
            
            # 检查是否是常量定义
            if '=' in line:
                if not field_name.isupper():  # 如果常量名不是全大写
                    new_const = field_name.upper()
                    line = f"{indent}{type_part} {new_const}{rest}"
                    modified = True
            else:
                # 处理类型名（包括数组）
                new_type = capitalize_type(type_part)
                # 处理字段名
                new_field = camel_to_snake(field_name)
                
                if new_type != type_part or new_field != field_name:
                    modified = True
                    line = f"{indent}{new_type} {new_field}{rest}"
        
        new_lines.append(line)
    
    if modified:
        with open(file_path, 'w') as f:
            f.write('\n'.join(new_lines) + '\n')
        print(f"Modified fields in: {file_path}")

def capitalize_first_letter(directory):
    """处理目录中的所有消息文件"""
    if not os.path.exists(directory):
        print(f"Directory {directory} does not exist")
        return
    
    for filename in os.listdir(directory):
        if not (filename.endswith('.msg') or filename.endswith('.srv')):
            continue
            
        # 只将第一个字母大写
        new_filename = filename[0].upper() + filename[1:]
        
        file_path = os.path.join(directory, filename)
        new_path = os.path.join(directory, new_filename)
        
        # 首先转换文件中的字段名和类型名
        convert_fields_to_snake_case(file_path)
        
        # 如果文件名需要改变，则重命名
        if filename != new_filename:
            try:
                os.rename(file_path, new_path)
                print(f"Renamed: {filename} -> {new_filename}")
            except Exception as e:
                print(f"Error renaming {filename}: {str(e)}")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    
    msg_dir = os.path.join(script_dir, 'msg')
    print("\nProcessing msg directory...")
    capitalize_first_letter(msg_dir)
    
    srv_dir = os.path.join(script_dir, 'srv')
    print("\nProcessing srv directory...")
    capitalize_first_letter(srv_dir)

if __name__ == "__main__":
    main()