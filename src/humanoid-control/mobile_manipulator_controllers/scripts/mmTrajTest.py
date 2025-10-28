#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeTorsoCtrlMode, changeTorsoCtrlModeRequest, changeArmCtrlMode, changeArmCtrlModeRequest
import os

def read_csv_file(file_path):
    """
    Reads a CSV file containing trajectory data.
    Each line is expected to have a name, a timestamp, and 14 pose values.
    """
    times = []
    values = []
    with open(file_path, 'r') as f:
        for line in f:
            tokens = line.strip().split()
            if not tokens:
                continue  # Skip empty lines
            
            # Skip the first column (name)
            tokens = tokens[1:]
            if len(tokens) < 15:  # Expect at least 1 timestamp and 14 values
                rospy.logwarn("Row has insufficient data, skipping: %s", line)
                continue
            
            time = float(tokens[0])
            pose_values = [float(x) for x in tokens[1:15]]  # 14 pose values
            times.append(time)
            values.extend(pose_values)  # Flatten pose values into a single list
            
    return times, values

def change_mm_ctrl_mode(control_mode):
    """
    Calls the service to change the mobile manipulator control mode.
    """
    rospy.wait_for_service('/mobile_manipulator_mpc_control')
    try:
        change_mode = rospy.ServiceProxy('/mobile_manipulator_mpc_control', changeTorsoCtrlMode)
        req = changeTorsoCtrlModeRequest()
        req.control_mode = control_mode
        res = change_mode(req)
        if res.result:
            rospy.loginfo("Mobile manipulator control mode changed to %d", control_mode)
        else:
            rospy.logerr("Failed to change mobile manipulator control mode to %d", control_mode)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def change_arm_ctrl_mode(control_mode):
    rospy.wait_for_service('/arm_traj_change_mode')
    try:
        change_mode = rospy.ServiceProxy('/arm_traj_change_mode', changeArmCtrlMode)
        req = changeArmCtrlModeRequest()
        req.control_mode = control_mode
        res = change_mode(req)
        if res.result:
            rospy.loginfo("手臂控制模式已更改为 %d", control_mode)
        else:
            rospy.logerr("无法将手臂控制模式更改为 %d", control_mode)
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)

def srv_change_manipulation_mpc_control_flow(ctrl_flow):
    try:
        service_name = '/enable_mm_wbc_arm_trajectory_control'
        rospy.wait_for_service(service_name, timeout=2.0)
        set_mode_srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)

        req = changeArmCtrlModeRequest()
        req.control_mode = ctrl_flow

    except rospy.ServiceException as e:
        rospy.logerr("Service call to %s failed: %s", service_name, e)
    except rospy.ROSException as e:
        rospy.logerr("Failed to connect to service %s: %s", service_name, e)
    except Exception as e:
        rospy.logerr("Failed to change manipulation mpc control flow: %s", e)

def reset_mm_mpc():
    rospy.wait_for_service('/reset_mm_mpc')
    try:
        reset_mpc = rospy.ServiceProxy('/reset_mm_mpc', changeTorsoCtrlMode)
        req = changeTorsoCtrlModeRequest()
        res = reset_mpc(req)
        if res.result:
            rospy.loginfo("Mobile manipulator MPC reset successfully")
        else:
            rospy.logerr("Failed to reset mobile manipulator MPC")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to %s failed: %s", '/reset_mm_mpc', e)
    except rospy.ROSException as e:
        rospy.logerr("Failed to connect to service %s: %s", '/reset_mm_mpc', e)
    except Exception as e:
        rospy.logerr("Failed to reset mobile manipulator MPC: %s", e)

def main():
    rospy.init_node('mm_traj_publisher')
    pub = rospy.Publisher('/mm/end_effector_trajectory', armTargetPoses, queue_size=10)

    # Path to the CSV file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file_name = '../data/mm_poses_1.csv'
    file_path = os.path.join(script_dir, csv_file_name)

    if not os.path.isfile(file_path):
        rospy.logerr("File not found: %s", file_path)
        return

    times, values = read_csv_file(file_path)

    if not times or not values:
        rospy.logerr("Failed to read data from CSV file, data is empty.")
        return

    # Before publishing, call the service to set the control mode to 3 (BaseArm)
    # This allows the controller to process trajectories for the base and arms.

    for i in range(10):

        change_mm_ctrl_mode(1)
        change_arm_ctrl_mode(2)
        srv_change_manipulation_mpc_control_flow(2)
        msg = armTargetPoses()
        msg.times = times
        msg.values = values
        msg.frame = 2   # 0 keep current frame  1 world frame (based on odom)  2  local frame  3  VRFrame  4  manipulation world frame

        rospy.loginfo("Publishing mobile manipulator target poses to topic '/mm/end_effector_trajectory'")
        
        # Wait for a subscriber to connect
        rate = rospy.Rate(10)  # 10Hz
        while pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("Waiting for a subscriber to connect...")
            rate.sleep()

        # reset_mm_mpc()
        
        if not rospy.is_shutdown():
            pub.publish(msg)
            rospy.loginfo("Message published.")
        rospy.sleep(15)
        change_mm_ctrl_mode(0)
        change_arm_ctrl_mode(1)
        srv_change_manipulation_mpc_control_flow(0)
        rospy.sleep(4)
    
    # After publishing, you might want to switch back to another mode.
    # For example, setting it to 0 (None) will stop the controller from following trajectories.
    # rospy.sleep(max(times) + 1.0) # wait for trajectory to finish
    # change_mm_ctrl_mode(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 