#!/usr/bin/env python3
import os
import sys
import yaml
import argparse
from typing import Dict, List, Tuple, Optional
import re

def camel_to_snake(name: str) -> str:
    """Convert camelCase or PascalCase to snake_case"""
    first_word = name[0].lower() + name[1:]
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', first_word)
    s2 = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1)
    return s2.lower()

def capitalize_type(type_name: str) -> str:
    """Ensure custom message types start with capital letter"""
    basic_types = {
        'bool', 'byte', 'char',
        'float32', 'float64',
        'int8', 'uint8',
        'int16', 'uint16',
        'int32', 'uint32',
        'int64', 'uint64',
        'string', 'wstring'
    }
    
    array_suffix = ''
    array_match = re.search(r'(\[\d*\])', type_name)
    if array_match:
        array_suffix = array_match.group(1)
        type_name = type_name.replace(array_suffix, '')
    
    if type_name.lower() in {t.lower() for t in basic_types}:
        return type_name.lower() + array_suffix
    
    parts = type_name.split('/')
    if len(parts) > 1:
        return '/'.join(parts[:-1] + [parts[-1][0].upper() + parts[-1][1:]]) + array_suffix
    
    return type_name[0].upper() + type_name[1:] + array_suffix

def parse_message_file(file_path: str) -> List[Tuple[str, str]]:
    """Parse a message file and return list of (type, field_name) tuples"""
    fields = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            # Match field definition
            match = re.match(r'^([a-zA-Z0-9_/]+(?:\[\d*\])?)\s+([a-zA-Z][a-zA-Z0-9_]*)(.*?)$', line)
            if match:
                type_part = match.group(1)
                field_name = match.group(2)
                
                # Skip constant definitions
                if '=' in line:
                    continue
                
                # Process type and field name
                new_type = capitalize_type(type_part)
                fields.append((new_type, field_name))  # Keep original field name
    
    return fields

def compare_messages(ros1_fields: List[Tuple[str, str]], 
                    ros2_fields: List[Tuple[str, str]]) -> Optional[Dict[str, str]]:
    """Compare message fields and return field mapping if different"""
    if len(ros1_fields) != len(ros2_fields):
        return {ros1_field: ros2_field for (_, ros1_field), (_, ros2_field) in zip(ros1_fields, ros2_fields)}
    
    field_mapping = {}
    for (ros1_type, ros1_field), (ros2_type, ros2_field) in zip(ros1_fields, ros2_fields):
        # 检查字段名是否符合 snake_case 规则
        ros1_snake = camel_to_snake(ros1_field)
        ros2_snake = camel_to_snake(ros2_field)
        
        # 如果原始字段名不符合 snake_case 规则，添加到映射中
        if ros1_field != ros1_snake or ros2_field != ros2_snake:
            field_mapping[ros1_field] = ros2_field
    
    return field_mapping if field_mapping else None

def find_message_files(package_path: str) -> List[str]:
    """Find all .msg files in the package"""
    msg_files = []
    msg_dir = os.path.join(package_path, 'msg')
    if os.path.exists(msg_dir):
        for file in os.listdir(msg_dir):
            if file.endswith('.msg'):
                msg_files.append(os.path.join(msg_dir, file))
    return msg_files

def get_message_name(file_path: str) -> str:
    """Get message name without extension and normalized for comparison"""
    name = os.path.basename(file_path)[:-4]
    # Normalize by removing underscores and converting to lowercase
    # This handles both SetLEDMode_free and SetLEDModeFree as the same entity
    return name.lower().replace('_', '')

def underscore_to_camel_case(name: str) -> str:
    """Convert underscore names to PascalCase - matches format_msgs.py logic"""
    if '_' not in name:
        return name[0].upper() + name[1:] if name else name
    
    parts = name.split('_')
    return ''.join(word[0].upper() + word[1:] if word else '' for word in parts)

def parse_service_file(file_path: str) -> Tuple[List[Tuple[str, str]], List[Tuple[str, str]]]:
    """Parse a service file and return tuple of (request_fields, response_fields)"""
    request_fields = []
    response_fields = []
    current_section = request_fields
    
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            # Check for section separator
            if line == '---':
                current_section = response_fields
                continue
            
            # Match field definition
            match = re.match(r'^([a-zA-Z0-9_/]+(?:\[\d*\])?)\s+([a-zA-Z][a-zA-Z0-9_]*)(.*?)$', line)
            if match:
                type_part = match.group(1)
                field_name = match.group(2)
                
                # Skip constant definitions
                if '=' in line:
                    continue
                
                # Process type and field name
                new_type = capitalize_type(type_part)
                current_section.append((new_type, field_name))
    
    return request_fields, response_fields

def compare_service_fields(ros1_fields: List[Tuple[str, str]], 
                         ros2_fields: List[Tuple[str, str]]) -> Optional[Dict[str, str]]:
    """Compare service fields and return field mapping if different"""
    if len(ros1_fields) != len(ros2_fields):
        return {ros1_field: ros2_field for (_, ros1_field), (_, ros2_field) in zip(ros1_fields, ros2_fields)}
    
    field_mapping = {}
    for (ros1_type, ros1_field), (ros2_type, ros2_field) in zip(ros1_fields, ros2_fields):
        # 检查字段名是否符合 snake_case 规则
        ros1_snake = camel_to_snake(ros1_field)
        ros2_snake = camel_to_snake(ros2_field)
        
        # 如果原始字段名不符合 snake_case 规则，添加到映射中
        if ros1_field != ros1_snake or ros2_field != ros2_snake:
            field_mapping[ros1_field] = ros2_field
    
    return field_mapping if field_mapping else None

def find_service_files(package_path: str) -> List[str]:
    """Find all .srv files in the package"""
    srv_files = []
    srv_dir = os.path.join(package_path, 'srv')
    if os.path.exists(srv_dir):
        for file in os.listdir(srv_dir):
            if file.endswith('.srv'):
                srv_files.append(os.path.join(srv_dir, file))
    return srv_files

def generate_mapping_rules(ros1_path: str, ros2_path: str) -> List[Dict]:
    """Generate mapping rules between ROS1 and ROS2 messages and services"""
    mapping_rules = []
    
    # Get message and service files from both packages
    ros1_msgs = find_message_files(ros1_path)
    ros2_msgs = find_message_files(ros2_path)
    ros1_srvs = find_service_files(ros1_path)
    ros2_srvs = find_service_files(ros2_path)
    
    # Create lookup for ROS2 messages and services (case-insensitive)
    ros2_msg_map = {get_message_name(msg): msg for msg in ros2_msgs}
    ros2_srv_map = {get_message_name(srv): srv for srv in ros2_srvs}
    
    # Process all ROS1 messages
    for ros1_msg in ros1_msgs:
        ros1_msg_name = os.path.basename(ros1_msg)[:-4]  # Remove .msg extension
        ros1_pkg_name = os.path.basename(os.path.dirname(os.path.dirname(ros1_msg)))
        
        # Find corresponding ROS2 message (handle underscore to PascalCase conversion)
        ros1_msg_key = get_message_name(ros1_msg)
        ros2_msg = ros2_msg_map.get(ros1_msg_key)
        
        # If not found and ros1 has underscores, try to find the PascalCase version
        if not ros2_msg and '_' in ros1_msg_name:
            expected_ros2_name = underscore_to_camel_case(ros1_msg_name)
            expected_ros2_key = get_message_name(expected_ros2_name + '.msg')
            ros2_msg = ros2_msg_map.get(expected_ros2_key)
        
        if ros2_msg:
            ros2_pkg_name = os.path.basename(os.path.dirname(os.path.dirname(ros2_msg)))
            ros2_msg_name = os.path.basename(ros2_msg)[:-4]  # Remove .msg extension
            
            # Parse message contents
            ros1_fields = parse_message_file(ros1_msg)
            ros2_fields = parse_message_file(ros2_msg)
            
            # Compare messages
            field_mapping = compare_messages(ros1_fields, ros2_fields)
            
            # Create mapping rule
            rule = {
                'ros1_package_name': ros1_pkg_name,
                'ros1_message_name': ros1_msg_name,
                'ros2_package_name': ros2_pkg_name,
                'ros2_message_name': ros2_msg_name
            }
            
            if field_mapping:
                rule['fields_1_to_2'] = field_mapping
            
            mapping_rules.append(rule)
        else:
            # If no corresponding ROS2 message found, still create a rule
            rule = {
                'ros1_package_name': ros1_pkg_name,
                'ros1_message_name': ros1_msg_name,
                'ros2_package_name': ros1_pkg_name,  # Use same package name
                'ros2_message_name': ros1_msg_name   # Use same message name
            }
            mapping_rules.append(rule)
    
    # Process ROS2 messages that don't have corresponding ROS1 messages
    ros1_msg_keys = {get_message_name(msg) for msg in ros1_msgs}
    for ros2_msg in ros2_msgs:
        ros2_msg_key = get_message_name(ros2_msg)
        if ros2_msg_key not in ros1_msg_keys:
            ros2_msg_name = os.path.basename(ros2_msg)[:-4]  # Remove .msg extension
            ros2_pkg_name = os.path.basename(os.path.dirname(os.path.dirname(ros2_msg)))
            
            rule = {
                'ros1_package_name': ros2_pkg_name,  # Use same package name
                'ros1_message_name': ros2_msg_name,  # Use same message name
                'ros2_package_name': ros2_pkg_name,
                'ros2_message_name': ros2_msg_name
            }
            mapping_rules.append(rule)
    
    # Process all ROS1 services
    for ros1_srv in ros1_srvs:
        ros1_srv_name = os.path.basename(ros1_srv)[:-4]  # Remove .srv extension
        ros1_pkg_name = os.path.basename(os.path.dirname(os.path.dirname(ros1_srv)))
        
        # Find corresponding ROS2 service (handle underscore to PascalCase conversion)
        ros1_srv_key = get_message_name(ros1_srv)
        ros2_srv = ros2_srv_map.get(ros1_srv_key)
        
        # If not found and ros1 has underscores, try to find the PascalCase version
        if not ros2_srv and '_' in ros1_srv_name:
            expected_ros2_name = underscore_to_camel_case(ros1_srv_name)
            expected_ros2_key = get_message_name(expected_ros2_name + '.srv')
            ros2_srv = ros2_srv_map.get(expected_ros2_key)
        
        if ros2_srv:
            ros2_pkg_name = os.path.basename(os.path.dirname(os.path.dirname(ros2_srv)))
            ros2_srv_name = os.path.basename(ros2_srv)[:-4]  # Remove .srv extension
            
            # Parse service contents
            ros1_request_fields, ros1_response_fields = parse_service_file(ros1_srv)
            ros2_request_fields, ros2_response_fields = parse_service_file(ros2_srv)
            
            # Compare service fields
            request_mapping = compare_service_fields(ros1_request_fields, ros2_request_fields)
            response_mapping = compare_service_fields(ros1_response_fields, ros2_response_fields)
            
            # Create mapping rule
            rule = {
                'ros1_package_name': ros1_pkg_name,
                'ros1_service_name': ros1_srv_name,
                'ros2_package_name': ros2_pkg_name,
                'ros2_service_name': ros2_srv_name
            }
            
            if request_mapping:
                rule['request_fields_1_to_2'] = request_mapping
            if response_mapping:
                rule['response_fields_1_to_2'] = response_mapping
            
            mapping_rules.append(rule)
        else:
            # If no corresponding ROS2 service found, still create a rule
            rule = {
                'ros1_package_name': ros1_pkg_name,
                'ros1_service_name': ros1_srv_name,
                'ros2_package_name': ros1_pkg_name,  # Use same package name
                'ros2_service_name': ros1_srv_name   # Use same service name
            }
            mapping_rules.append(rule)
    
    # Process ROS2 services that don't have corresponding ROS1 services
    ros1_srv_keys = {get_message_name(srv) for srv in ros1_srvs}
    for ros2_srv in ros2_srvs:
        ros2_srv_key = get_message_name(ros2_srv)
        if ros2_srv_key not in ros1_srv_keys:
            ros2_srv_name = os.path.basename(ros2_srv)[:-4]  # Remove .srv extension
            ros2_pkg_name = os.path.basename(os.path.dirname(os.path.dirname(ros2_srv)))
            
            rule = {
                'ros1_package_name': ros2_pkg_name,  # Use same package name
                'ros1_service_name': ros2_srv_name,  # Use same service name
                'ros2_package_name': ros2_pkg_name,
                'ros2_service_name': ros2_srv_name
            }
            mapping_rules.append(rule)
    
    # Remove duplicate rules, but keep valid ROS1->ROS2 transformations
    unique_rules = []
    seen_combinations = set()
    
    for rule in mapping_rules:
        # Create a unique key for each rule based on exact names, not normalized
        if 'ros1_message_name' in rule:
            # Message rule
            key = (
                rule['ros1_package_name'], 
                rule['ros1_message_name'],
                rule['ros2_package_name'],
                rule['ros2_message_name'],
                'message'
            )
        else:
            # Service rule
            key = (
                rule['ros1_package_name'],
                rule['ros1_service_name'],
                rule['ros2_package_name'], 
                rule['ros2_service_name'],
                'service'
            )
        
        if key not in seen_combinations:
            seen_combinations.add(key)
            unique_rules.append(rule)
    
    return unique_rules

def main():
    parser = argparse.ArgumentParser(description='Generate mapping rules between ROS1 and ROS2 messages')
    parser.add_argument('--ros1_path', help='Path to ROS1 package')
    parser.add_argument('--ros2_path', help='Path to ROS2 package')
    parser.add_argument('--output', '-o', default='mapping_rules.yaml',
                      help='Output YAML file (default: mapping_rules.yaml)')
    
    args = parser.parse_args()
    
    # Generate mapping rules
    mapping_rules = generate_mapping_rules(args.ros1_path, args.ros2_path)
    
    # Write to YAML file with spacing between rules
    with open(args.output, 'w') as f:
        for rule in mapping_rules:
            yaml.dump([rule], f, default_flow_style=False, sort_keys=False)
            f.write('\n')  # Add extra newline between rules
    
    print(f"Generated mapping rules written to {args.output}")

if __name__ == "__main__":
    main()
