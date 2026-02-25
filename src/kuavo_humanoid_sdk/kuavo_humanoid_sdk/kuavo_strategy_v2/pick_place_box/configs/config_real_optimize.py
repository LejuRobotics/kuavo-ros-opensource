import math
from typing import Dict, Tuple


def _build_pick_offsets(
    pre_pos: Tuple[float, float, float],
    pre_euler_deg: Tuple[float, float, float],
    grasp_pos: Tuple[float, float, float],
    grasp_euler_deg: Tuple[float, float, float],
) -> Dict[str, Dict[str, Tuple[float, float, float]]]:
    """统一构造抓取阶段偏移结构。"""
    return {
        'pre_grasp': {
            'pos_offset': pre_pos,
            'euler_deg': pre_euler_deg,
        },
        'grasp': {
            'pos_offset': grasp_pos,
            'euler_deg': grasp_euler_deg,
        },
    }

def _build_post_grasp_offsets(
    withdraw_pos_offset: Tuple[float, float, float],
    withdraw_euler_deg: Tuple[float, float, float],
    observe_pos_abs: Tuple[float, float, float],
    observe_euler_deg: Tuple[float, float, float],
) -> Dict[str, Dict[str, Tuple[float, float, float]]]:
    """统一构造抓取后的撤手与观察位姿结构。"""
    return {
        'withdraw': {
            'pos_offset': withdraw_pos_offset,
            'euler_deg': withdraw_euler_deg,
        },
        'observe': {
            'pos_abs': observe_pos_abs,
            'euler_deg': observe_euler_deg,
        },
    }


def _make_row_nav_positions(ids, x_values, y_default, yaw_default):
    """按行批量生成导航位姿: 统一 y/θ，逐项给 x。"""
    assert len(ids) == len(x_values)
    return {tag_id: (x_values[i], y_default, yaw_default) for i, tag_id in enumerate(ids)}


def _build_pick_nav_positions() -> Dict[int, Tuple[float, float, float]]:
    """构造抓取导航位姿，支持行级默认 + ID 级覆盖，避免重复。"""
    # 各排的 ID 列表
    third_ids = list(range(0, 9))
    fourth_ids = list(range(10, 19))
    second_ids = list(range(20, 29))
    fifth_ids = list(range(30, 39))
    sixth_ids = list(range(40, 49))
    seventh_ids = list(range(50,59))

    # y 模板
    x_third = [-5.895, -5.775, -5.655, -5.535, -5.415, -5.295, -5.175, -5.055, -4.935]
    x_other = [-5.895, -5.775, -5.655, -5.535, -5.415, -5.295, -5.175, -5.055, -4.935]

    yaw = 1.57

    positions = {}
    positions.update(_make_row_nav_positions(second_ids, x_other, 7.37, yaw))
    positions.update(_make_row_nav_positions(third_ids, x_third, 7.35 + 0.03, yaw))
    positions.update(_make_row_nav_positions(fourth_ids, x_other, 7.30, yaw))
    positions.update(_make_row_nav_positions(fifth_ids, x_other, 7.33, yaw))
    positions.update(_make_row_nav_positions(sixth_ids, x_other, 7.31 ,yaw))
    positions.update(_make_row_nav_positions(seventh_ids, x_other, 7.31 - 0.16, yaw))

    # 个别 ID 的 x 覆盖
    y_overrides = {

    }
    for tag_id, y in y_overrides.items():
        pos = positions[tag_id]
        positions[tag_id] = (pos[0], y_overrides, pos[2])

    return positions

class config:
    class common:
        """通用配置"""
        step_back_distance = 0.2

        arm_timeout = 50  # 手臂移动事件的超时时间，单位秒

        arm_control_mode = "fixed_base" # 'manipulation_mpc'  # 手臂控制模式，使用Manipulation MPC控制模式
        arm_pos_threshold = 0.1  # 手臂位置阈值，单位米
        arm_angle_threshold = math.radians(20)  # 手臂角度阈值，单位弧度

        walk_timeout = 50  # 导航事件的超时时间，单位秒
        walk_yaw_threshold = 0.1  # 偏航角允许的误差阈值，单位弧度
        walk_pos_threshold = 0.05  # 位置允许的误差阈值，单位米
        walk_control_mode = "cmd_pos_world"  # 控制模式（如"cmd_vel"速度控制、"cmd_pos_world"世界坐标控制）

        # 循环次数配置
        max_cycles = 1 # 设定循环执行次数，设为0表示无限循环

    class pick:
        """搬框配置"""

        arm_pre_grab_pose = {
            'second_row': {
                'ids': [20,21,22,23,24,25,26,27,28],
                'debug_row_name': '第2排',
                'pre_target_poses':[-1.864925884570995, 0.07171760169612298, -0.23956699093767694, -1.1448082175063021-1 , -0.8232264009885075, -1.2058446279139248, 0.5024020311236667,
                                0.4138207404885018, -0.15983968848691735, 0.18387089892057792, -0.8167376213993754, -0.3093762427452447, -0.05721935293794519, 0.2864843218217582]
            },
            'third_row': {
                'ids': [0,1,2,3,4,5,6,7,8],
                'debug_row_name': '第3排',
                'pre_target_poses':[0.05, -0.10948231523235437+0.25, -0.3532446187553241, -1.7101551237921435 - 1, -1.2359804166095902, -0.8991385506669644, 0.5264348757480797,
                                 0.5562323778035756, -0.19875007172679646, 0.17586112967766002, -1.0509647642419573, -0.25368174627202994, -0.058369492521587955, 0.19493789749930354]
            },
            'fourth_row': {
                'ids': [10,11,12,13,14,15,16,17,18],
                'debug_row_name': '第4排',
                'pre_target_poses':[-0.5529355567248357+0.8, 0.0221262170782787, -0.3238730914104787, -2.108416615846741-0.1, -1.264970898259671-0.1, -0.9246949554027776 + 0.5, 0.4917224646445439,
                                 0.34917966466875316, -0.13275350305790296, 0.17548065176552988, -0.6374432374623392, -0.25406001412390417, -0.032806133098018816, 0.25291731925469113]
            },
            'fifth_row': {
                'ids': [30,31,32,33,34,35,36,37,38],
                'debug_row_name': '第5排',
                'pre_target_poses':[0, 0.10223454694164179, -0.3864361066765584, -1.9714657352099847, -1.2699331368659355, -0.6485125212790057, 0.47608475415239304,
                                0.4251242660132635, -0.16518011357117365, 0.18921250807366002, -0.7991898124181177, -0.23346091129042876, -0.017546943649274373, 0.1235976606162396]
            },
            'sixth_row': {
                'ids': [40,41,42,43,44,45,46,47,48],
                'debug_row_name': '第6排',
                'pre_target_poses':[0.2, 0.2, 0, -0.9792472165750149, -1.2119498484466946, -0.0949838005376901, 0.12,
                                0.49272451974661646, -0.19073830559927674, 0.2017998925078215, -0.9086734624982771, -0.2166766667081763, -0.019456379369027266, 0.13274701606817893]
            },
        }

        # 格式: (x, y, theta)，用于SLAM导航的目标位置
        pick_nav_positions = _build_pick_nav_positions()


        # 行配置：将重复的参数集中管理
        row_configs = {
            'second_row': {
                'ids': [20,21,22,23,24,25,26,27,28],
                'debug_row_name': '第2排',
                'tag_offset': 20,
                'squat': None,
                'stand': None,
                'motion_delay': 0.0,
                'stand_delay': 0.0,
                'head_pitch_deg': -10.0,
            },
            'fourth_row': {
                'ids': [10,11,12,13,14,15,16,17,18],
                'debug_row_name': '第4排',
                'tag_offset': 10,
                'squat': { 'height': -0.08, 'pitch': 0.26 },   # 弯腰角度15°
                'stand': { 'height': 0.08, 'pitch': -0.26 },  # 起立角度15°
                'motion_delay': 1,
                'stand_delay': 0.5,
                'head_pitch_deg': 0,
            },
            'fifth_row': {
                'ids': [30,31,32,33,34,35,36,37,38],
                'debug_row_name': '第5排',
                'tag_offset': 30,
                'squat': { 'height': -0.20, 'pitch': 0.15 },
                'stand': { 'height': 0.20,  'pitch': -0.15 },
                'motion_delay': 1.5,
                'stand_delay': 0.8,
                'head_pitch_deg': -15.0,
            },
            'sixth_row': {
                'ids': [40,41,42,43,44,45,46,47,48],
                'debug_row_name': '第6排',
                'tag_offset': 40,
                'squat': { 'height': -0.30, 'pitch': 0.15 },
                'stand': { 'height': 0.30,  'pitch': -0.15 },
                'motion_delay': 1.3,
                'stand_delay': 1.0,
                'head_pitch_deg': -25,
            },
        }

        # 兼容旧字段名称（尽量不破坏外部引用）
        second_row = row_configs['second_row']['ids']
        fourth_row = row_configs['fourth_row']['ids']
        fifth_row = row_configs['fifth_row']['ids']
        sixth_row = row_configs['sixth_row']['ids']

        # 抓取偏移配置（相对于检测到的 tag 位姿）
        # 结构说明：
        # pick_pose_offsets[mode][stage] -> { 'pos_offset': (dx, dy, dz), 'euler_deg': (roll, pitch, yaw) }
        # - mode: 'fourth_row' 弯腰抓取，'third_row' 直腰抓取
        # - stage: 'pre_grasp' 预抓取点，'grasp' 抓取点

        pick_pose_offsets = {
            'second_row': _build_pick_offsets(
                pre_pos=(-0.15, -0.03, 0.3-0.01),
                pre_euler_deg=(100, 0.0, -90),
                grasp_pos=(0.156, 0.03+0.07, 0.25),  #抓取点位
                grasp_euler_deg=(90, 0.0, -90),
            ),
            'third_row': _build_pick_offsets(
                pre_pos=(-0.02-0.03, -0.03, -0.05 + 0.11),
                pre_euler_deg=(90, 0.0, -90),
                grasp_pos=(0.11-0.03, -0.07, -0.09+0.02),
                grasp_euler_deg=(90, 0.0, -90),
            ),
            'fourth_row': _build_pick_offsets(
                pre_pos=(-0.2, -0.03, -0.28),
                pre_euler_deg=(90, 0.0, -90),
                grasp_pos=(0.16, -0.03, -0.2),
                grasp_euler_deg=(90, 0.0, -90),
            ),
            'fifth_row': _build_pick_offsets(
                pre_pos=(-0.05, 0.14, -0.48),
                pre_euler_deg=(85, 0.0, -90),
                grasp_pos=(0.23,  0.17, -0.61),
                grasp_euler_deg=(85, 0.0, -90),
            ),
            'sixth_row': _build_pick_offsets(
                pre_pos=(-0.15, -0.05, -0.5),
                pre_euler_deg=(80, 0.0, -90),
                grasp_pos=(0.15, -0.08, -0.9),
                grasp_euler_deg=(80, 0.0, -90),
            ),
        }

        # 抓取后的撤手与观察位姿
        # 说明：
        # - withdraw 使用相对 tag 的偏移（pos_offset）
        # - observe 使用绝对坐标（pos_abs），如果需要也可以改为相对偏移
        post_grasp_offsets = {
            'second_row': _build_post_grasp_offsets(
                withdraw_pos_offset=(-0.45, 0.0, 0.17),
                withdraw_euler_deg=(130, 0.0, -90),
                observe_pos_abs=(0.4, 0.12, 0.4),
                observe_euler_deg=(90, -60.0, -160),
            ),
            'third_row': _build_post_grasp_offsets(
                withdraw_pos_offset=(-0.4, -0.02, 0.08),
                withdraw_euler_deg=(90, 0.0, -90),
                observe_pos_abs=(0.41, 0.12, 0.4),
                observe_euler_deg=(90, -60.0, -160),
            ),
            'fourth_row': _build_post_grasp_offsets(
                withdraw_pos_offset=(-0.4, -0.03, -0.04),
                withdraw_euler_deg=(90, 0.0, -90),
                observe_pos_abs=(0.41+0.02, 0.12+0.02, 0.4),
                observe_euler_deg=(90, -60.0, -160),
            ),
            'fifth_row': _build_post_grasp_offsets(
                withdraw_pos_offset=(-0.45, -0.0, -0.16),
                withdraw_euler_deg=(90, 0.0, -90),
                observe_pos_abs=(0.35, 0.12, 0.4),
                observe_euler_deg=(90, -60.0, -160),
            ),
            'sixth_row': _build_post_grasp_offsets(
                withdraw_pos_offset=(-0.35, -0.0, -0.35),
                withdraw_euler_deg=(90, 0.0, -90),
                observe_pos_abs=(0.35, 0.12, 0.4),
                observe_euler_deg=(90, -60.0, -160),
            ),
        }



    class place:
        """放框配置"""

        place_pre_positions = {
            0: (-5.367,6.509,-1.57),
        }

        place_nav_positions = {
            0: (-5.367,6.309,-1.57),
        }

        place_squat_height_pick = -0.12
        place_squat_pitch_pick = 0.35   # 弯腰角度20°

        place_stand_pitch_pick = -0.4  # 起立角度20°
        place_stand_height_pick = 0.12
        place_motion_delay = 2
