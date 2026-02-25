import json
import serial
import serial.tools.list_ports
import math
import numpy as np

INIT_ARM_POS = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def read_json_file(file_path):
    with open(file_path) as json_file:
        data = json.load(json_file)
    return data


def map_value(value, O_min, O_max, N_min, N_max):
    return (value - O_min) * (N_max - N_min) / (O_max - O_min) + N_min


def find_and_send():
    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        if "CP2102 USB to UART Bridge Controller" in port.description:
            print("设备: {}".format(port.device))
            print("名称: {}".format(port.name))
            print("描述: {}".format(port.description))
            print("物理位置: {}".format(port.location))
            print("制造商: {}".format(port.manufacturer))
            print("串口号: {}".format(port.serial_number))
            return port
    return 0


def open_serial_port(serial_port):
    try:
        ser = serial.Serial(serial_port.device, baudrate=9600, timeout=1)
        print(f"已成功打开串口 {serial_port.device}")
        return ser
    except Exception as e:
        print(f"ERROR: 打开串口 {serial_port.device} 时出错：{e}")
        return None


def send_data_to_port(ser, data):
    try:
        ser.write(data)

    except Exception as e:
        print(f"发送数据到串口时出错：{e}")

def frames_to_custom_action_data_ocs2(file_path: str, start_frame_time: float, current_arm_joint_state: list):
    def filter_data(action_data, start_frame_time, current_arm_joint_state):
        filtered_action_data = {}
        x_shift = start_frame_time - 1
        for key, frames in action_data.items():
            filtered_frames = []
            found_start = False
            skip_next = False
            for i in range(-1, len(frames)):
                frame = frames[i]
                if i == len(frames) - 1:
                    next_frame = frame
                else:
                    next_frame = frames[i+1]
                end_time = next_frame[0][0]

                if not found_start and end_time >= start_frame_time:
                    found_start = True

                    p0 = np.array([0, current_arm_joint_state[key-1]])
                    p3 = np.array([next_frame[0][0] - x_shift, next_frame[0][1]])

                    # Calculate control points for smooth transition
                    curve_length = np.linalg.norm(p3 - p0)
                    p1 = p0 + curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the right
                    p2 = p3 - curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the left

                    # Create new frame
                    frame1 = [
                        p0.tolist(),
                        p0.tolist(),
                        p1.tolist()
                    ]

                    # Modify next_frame's left control point
                    next_frame[1] = p2.tolist()

                    filtered_frames.append(frame1)
                    skip_next = True

                if found_start:
                    if skip_next:
                        skip_next = False
                        continue
                    end_point = [round(frame[0][0] - x_shift, 1), round(frame[0][1], 1)]
                    left_control_point = [round(frame[1][0] - x_shift, 1), round(frame[1][1], 1)]
                    right_control_point = [round(frame[2][0] - x_shift, 1), round(frame[2][1], 1)]
                    filtered_frames.append([end_point, left_control_point, right_control_point])

            filtered_action_data[key] = filtered_frames
        return filtered_action_data

    with open(file_path, "r") as f:
        data = json.load(f)
    frames = data["frames"]
    action_data = {}
    for frame in frames:
        servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
        for index, value in enumerate(servos):
            key = index + 1
            if key not in action_data:
                action_data[key] = []
                if keyframe != 0 and len(action_data[key]) == 0:
                    if key <= len(INIT_ARM_POS):
                        action_data[key].append([
                            [0, math.radians(INIT_ARM_POS[key-1])],
                            [0, math.radians(INIT_ARM_POS[key-1])],
                            [0, math.radians(INIT_ARM_POS[key-1])],
                ])
            if value is not None:
                CP = attribute[str(key)]["CP"]
                left_CP, right_CP = CP
                action_data[key].append([
                    [round(keyframe/100, 1), math.radians(value)],
                    [round((keyframe+left_CP[0])/100, 1), math.radians(value+left_CP[1])],
                    [round((keyframe+right_CP[0])/100, 1), math.radians(value+right_CP[1])],
                ])
    return filter_data(action_data, start_frame_time, current_arm_joint_state)

def get_start_end_frame_time(file_path: str):
    with open(file_path, "r") as f:
        data = json.load(f)
    start_frame_time = data["first"] / 100
    end_frame_time = data["finish"] / 100
    return start_frame_time, end_frame_time