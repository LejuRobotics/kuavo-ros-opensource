#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import argparse
import rosbag
import os
import math
import time
from kuavo_msgs.msg import sensorsData, armTargetPoses
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from sensor_msgs.msg import JointState
from key_listener import KeyListener


arm_joint_names = [
    "zarm_l1_link", "zarm_l2_link", "zarm_l3_link", "zarm_l4_link", "zarm_l5_link", "zarm_l6_link", "zarm_l7_link",
    "zarm_r1_link", "zarm_r2_link", "zarm_r3_link", "zarm_r4_link", "zarm_r5_link", "zarm_r6_link", "zarm_r7_link",
]

latest_joint_state = None
record_mode = False
recording = False
bag_file = None
bag = None
frame_counter = 0
output_file = None
file_descriptor = None  # 文件描述符，用于 os.fsync()

def set_robot_arm_ctl_mode(control_mode)->bool:
    """ 切换手臂规划模式 
    :param control_mode: uint8, # 0: keep pose, 1: auto_swing_arm, 2: external_control 
    :return: bool, 服务调用结果
    """
    try:
        rospy.wait_for_service('/arm_traj_change_mode')
        arm_ctrl_mode_client  = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)
        request = changeArmCtrlModeRequest()
        request.control_mode = control_mode

        response = arm_ctrl_mode_client(request)

        return response.result

    except rospy.ServiceException as e:
        rospy.logerr(f"changeArmCtrlMode Service call failed: {e}")
        return False

def set_kuavo_arm_traj(publisher, joint_state):
    """ 发布手臂控制命令
    :param joint_state
    """
    msg = JointState()  
    msg.header.stamp = rospy.Time.now()
    msg.name = arm_joint_names
    msg.position = [pos * (180 / math.pi) for pos in joint_state.position]
    publisher.publish(msg)

def set_kuavo_arm_target_poses(publisher, times, frame_list):
    arm_target_poses_msg = armTargetPoses()
    arm_target_poses_msg.times = times
    for pos in frame_list:
        degs = [v * (180 / math.pi) for v in pos]
        arm_target_poses_msg.values.extend(degs)

    publisher.publish(arm_target_poses_msg)

def joint_data_callback(sensors_data):
    global latest_joint_state, recording, bag, record_mode
    joint_data = sensors_data.joint_data
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = arm_joint_names

    if hasattr(joint_data, 'joint_q') and hasattr(joint_data, 'joint_v') and hasattr(joint_data, 'joint_vd'):
        joint_state.position = joint_data.joint_q[12:26]
        joint_state.velocity = joint_data.joint_v[12:26]
        joint_state.effort = joint_data.joint_torque[12:26]
    else:
        rospy.logwarn("The received message does not contain the expected attributes.")
        return
    
    # Publish the JointState message to /teach_pendant/record_arm_traj topic
    if record_mode:
        arm_joint_pub.publish(joint_state)

    # Store the latest joint state for potential saving on key press
    latest_joint_state = joint_state

    # If we are recording, also write the message to the bag file
    if recording and bag is not None:
        bag.write('/teach_pendant/record_arm_traj', joint_state)

def save_joint_state(key):
    global latest_joint_state, frame_counter, output_file, file_descriptor
    if latest_joint_state is not None:
        print("==> Saving current joint state(rad):\n {}".format(latest_joint_state.position))
        
        # Convert radian positions to degrees for saving
        angles_degrees = [math.degrees(pos) for pos in latest_joint_state.position]

        # Write to file with a frame name
        if output_file is not None:
            frame_name = f"frame_{frame_counter + 1}"
            output_file.write(f"{frame_name}: {angles_degrees}\n")
            frame_counter += 1

            # Flush the buffer and sync to disk
            output_file.flush()
            if file_descriptor is not None:
                os.fsync(file_descriptor)

        print(f"Saved as {frame_name}")
    else:
        print("No joint state to save.")    

def play_bag(file_path):
    global bag_file, bag, latest_joint_state
    
    bag_file = file_path
    bag = rosbag.Bag(bag_file, 'r')
    
    kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
    kuavo_arm_target_poses_pub = rospy.Publisher("/kuavo_arm_target_poses", armTargetPoses, queue_size=10)
    time.sleep(1.5)

    try:
        # 切换手臂控制模式
        set_robot_arm_ctl_mode(2)
        print(f"Playing back from {bag_file}...")

        # 获取第一帧的数据
        for topic, msg, t in bag.read_messages(topics=['/teach_pendant/record_arm_traj']):
            frame_0 = msg
            break  # 只需要第一帧

        if frame_0 is None:
            print("No messages found in the specified topic.")
            return

        # 确保 latest_joint_state 不为 None
        if latest_joint_state is None:
            print("Waiting for the first joint state...")
            while latest_joint_state is None and not rospy.is_shutdown():
                rospy.sleep(0.1) 
        print("==> Playing back reset to first frame.\n")        
        set_kuavo_arm_target_poses(kuavo_arm_target_poses_pub, [1.0, 3.0], [latest_joint_state.position, frame_0.position])
        time.sleep(3.5)

        # 开始回放
        print("==> Playing back  Starting...\n")        
        rate = rospy.Rate(500)  # 根据需要调整播放速度
        for topic, msg, t in bag.read_messages(topics=['/teach_pendant/record_arm_traj']):
            print(f"==> Playing back: {t}")
            if rospy.is_shutdown():
                print("ROS node is shutting down. Exiting the loop.")
                break
            set_kuavo_arm_traj(kuavo_arm_traj_pub, msg)
            rate.sleep()
    except KeyboardInterrupt:
        print("Caught keyboard interrupt. Exiting gracefully.")
    finally:
        bag.close()
        print("Playback complete.")


def play_from_file(file_path, play_interval_sec):
    """ 从文本文件中播放关节状态 """
    global latest_joint_state, paused
    
    kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
    kuavo_arm_target_poses_pub = rospy.Publisher("/kuavo_arm_target_poses", armTargetPoses, queue_size=10)
    time.sleep(1.5)

    try:
        # 切换手臂控制模式
        set_robot_arm_ctl_mode(2)
        print(f"Playing back from {file_path}...")

        
        # 读取文本文件
        with open(file_path, 'r') as f:
            lines = f.readlines()

        if not lines:
            print("No frames found in the file.")
            return

        # 解析每一行数据
        frames = []
        for line in lines:
            line = line.strip()
            if line and line.startswith("frame_"):
                frame_data = line.split(":")[1].strip()
                try:
                    angles_degrees = [float(angle) * (math.pi / 180) for angle in frame_data.strip('[]').split(',')]
                    frames.append(angles_degrees)
                except ValueError:
                    rospy.logwarn(f"Failed to parse frame data: {line}")

        if not frames:
            print("No valid frames found in the file.")
            return
        
        # 确保 latest_joint_state 不为 None
        if latest_joint_state is None:
            print("Waiting for the first joint state...")
            while latest_joint_state is None and not rospy.is_shutdown():
                rospy.sleep(0.1) 
        print("==> Playing back reset to first frame.\n")  
        set_kuavo_arm_target_poses(kuavo_arm_target_poses_pub, [1.0, play_interval_sec], [latest_joint_state.position, frames[0]])
        time.sleep(play_interval_sec+0.5)     

        # 开始回放
        print("==> Playing back Starting...\n")        

        set_kuavo_arm_target_poses(kuavo_arm_target_poses_pub, 
                                   [i * play_interval_sec for i in range(len(frames))],
                                     frames)

        time.sleep(play_interval_sec * len(frames))  

    except KeyboardInterrupt:
        print("Caught keyboard interrupt. Exiting gracefully.")
    finally:
        print("Playback complete.")

def listener(record=False, save_bag=None, play=None, save_to_file=None, play_file=None, play_file_interval_sec=None):
    global arm_joint_pub, latest_joint_state, recording, bag_file, bag, output_file, file_descriptor, record_mode

    rospy.init_node('kuavo_teach_pendant_node', anonymous=True)

    # record joint state publisher
    arm_joint_pub = rospy.Publisher('/teach_pendant/record_arm_traj', JointState, queue_size=10)
    rospy.Subscriber("/sensors_data_raw", sensorsData, joint_data_callback)

    if play:
        play_bag(play)
        return

    if play_file:

        if play_file_interval_sec:
            play_interval_sec = max(0.5, play_file_interval_sec)
        else: play_interval_sec = 1

        play_from_file(play_file, play_interval_sec)
        return
    
    if record:
        record_mode = True

        print("\nRecording started.\n")
        if save_bag:
            bag_file = save_bag
            try:
                bag = rosbag.Bag(bag_file, 'w')
                recording = True
            except Exception as e:
                rospy.logerr(f"Failed to open bag file {bag_file} for writing: {e}")
                exit(1)

            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                rate.sleep()

            recording = False
            if bag is not None:
                bag.close()
        
        elif save_to_file:
            try:
                output_file = open(save_to_file, 'a')  # Append mode to keep previous data
                file_descriptor = output_file.fileno()  # Get file descriptor for fsync
            except Exception as e:
                rospy.logerr(f"Failed to open file {save_to_file} for writing: {e}")
                exit(1)

            # Print key help information
            print("Key commands:")
            print("  s - Save the current joint state to file")
            print("  Ctrl+C - Exit the program")

            kl = KeyListener()
            kl.register_callback('s', save_joint_state)
            
            try:
                kl.loop_control()
            except KeyboardInterrupt:
                print("\nExiting program.")
                kl.stop()
            finally:

                if output_file is not None:
                    output_file.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ROS node for recording and playing back arm joint states.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--record', action='store_true', help='Record arm joint states.')
    group.add_argument('--play', type=str, help='Play back a ROS bag file containing arm joint states.')
    group.add_argument('--play_from_file', type=str, help='Play back from a text file containing joint states.')

    parser.add_argument('--play_file_interval_sec', type=float, help='Interval sec while playing file.')

    parser.add_argument('--save_bag', type=str, help='Save recorded data to a ROS bag file (only with --record).')
    parser.add_argument('--save_to_file', type=str, help='Save recorded joint states to a text file (only with --record).')

    args, unknown = parser.parse_known_args()

    if args.play and not os.path.isfile(args.play):
        print(f"Error: The specified bag file {args.play} does not exist.")
        exit(1)
    
    if args.play_from_file and not os.path.isfile(args.play_from_file):
        print(f"Error: The specified text file {args.play_from_file} does not exist.")
        exit(1)

    if args.save_bag and not args.record:
        parser.error("--save_bag can only be used with --record.")

    if args.save_to_file and not args.record:
        parser.error("--save_to_file can only be used with --record.")

    try:
        listener(record=args.record, save_bag=args.save_bag, play=args.play, 
                 save_to_file=args.save_to_file, play_file=args.play_from_file, 
                 play_file_interval_sec=args.play_file_interval_sec)
    except rospy.ROSInterruptException:
        pass
