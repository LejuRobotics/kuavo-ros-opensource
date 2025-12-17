import sys
import rospy
from kuavo_msgs.srv import fkSrv
from kuavo_msgs.msg import sensorsData  

joint_angles = None
def cb(msg):
	global joint_angles
	if hasattr(msg, 'joint_data') and hasattr(msg.joint_data, 'joint_q') and len(msg.joint_data.joint_q) >= 26:
		joint_angles = list(msg.joint_data.joint_q[12:26])

def main():
	global joint_angles
	rospy.init_node('example_fk_srv_node')
	fk_srv = rospy.ServiceProxy('/ik/fk_srv', fkSrv)

	use_sensor = False  # True: 订阅话题获取关节角度，False: 手动输入

	if use_sensor:
		sub = rospy.Subscriber('/sensors_data_raw', sensorsData, cb, queue_size=1)
		# 等待关节角度
		while not rospy.is_shutdown() and joint_angles is None:
			rospy.sleep(0.05)
		print(f"订阅到关节角度: {joint_angles}")
	else:
		# 直接在代码里写入关节角度（单位：弧度）
		joint_angles = [
			-0.76, 0.5, -0.1, -0.7, -0.0, -0.02, -0.27,
			-0.76, -0.5, 0.1, -0.7, -0.0, -0.02, -0.27
		]
		print(f"输入关节角度(弧度): {joint_angles}")

	if not joint_angles or len(joint_angles) != 14:
		print(f"关节角度数量错误，需14个，实际输入{len(joint_angles) if joint_angles else 'None'}")
		return

	rospy.wait_for_service('/ik/fk_srv')
	try:
		response = fk_srv(joint_angles)
		left_pose = response.hand_poses.left_pose
		right_pose = response.hand_poses.right_pose
		print("左臂末端:")
		print(f"  位置: {left_pose.pos_xyz}")
		print(f"  姿态(四元数): {left_pose.quat_xyzw}")
		print("右臂末端:")
		print(f"  位置: {right_pose.pos_xyz}")
		print(f"  姿态(四元数): {right_pose.quat_xyzw}")

		yaml_path = 'src/demo/motion_capture_calibration_arm/config/target_pose.yaml'
		with open(yaml_path, 'a') as f:
			f.write('  - \n')
			f.write('    left:\n')
			f.write(f'      pos_xyz: {list(left_pose.pos_xyz)}\n')
			f.write(f'      quat_xyzw: {list(left_pose.quat_xyzw)}\n')
			f.write('    right:\n')
			f.write(f'      pos_xyz: {list(right_pose.pos_xyz)}\n')
			f.write(f'      quat_xyzw: {list(right_pose.quat_xyzw)}\n')
		print(f"已追加到 {yaml_path}，请自行填写 time 字段")
	except Exception as e:
		print(f"FK服务调用失败: {e}")

if __name__ == '__main__':
	main()