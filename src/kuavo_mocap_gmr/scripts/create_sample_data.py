#!/usr/bin/env python3
"""
Create sample data file from actual OptiTrack streaming data.
This data can be used to test the simulation node.
"""

import os
import pickle
import numpy as np

# Actual rb_desc from OptiTrack (Y-up coordinate system)
rb_desc = {
    1: {'name': b'Skeleton 002_Hips', 'parent_id': 0, 'offset': [-0.0, 0.9670180082321167, 0.0]},
    2: {'name': b'Skeleton 002_Chest', 'parent_id': 1, 'offset': [-0.0, 0.08136750012636185, 0.0]},
    3: {'name': b'Skeleton 002_Chest2', 'parent_id': 2, 'offset': [-0.0, 0.20814700424671173, 0.0]},
    4: {'name': b'Skeleton 002_Neck', 'parent_id': 3, 'offset': [-0.0, 0.22799299657344818, 0.0]},
    5: {'name': b'Skeleton 002_Head', 'parent_id': 4, 'offset': [-0.0, 0.1538040041923523, 0.01984569989144802]},
    6: {'name': b'Skeleton 002_LeftCollar', 'parent_id': 3, 'offset': [0.039681099355220795, 0.14349399507045746, -0.010327699594199657]},
    7: {'name': b'Skeleton 002_LeftShoulder', 'parent_id': 6, 'offset': [0.17269200086593628, 0.0, 0.0]},
    8: {'name': b'Skeleton 002_LeftElbow', 'parent_id': 7, 'offset': [0.24771900475025177, 0.0, 0.0]},
    9: {'name': b'Skeleton 002_LeftWrist', 'parent_id': 8, 'offset': [0.23940299451351166, 0.0, 0.0]},
    10: {'name': b'Skeleton 002_RightCollar', 'parent_id': 3, 'offset': [-0.039681099355220795, 0.14349399507045746, -0.010327699594199657]},
    11: {'name': b'Skeleton 002_RightShoulder', 'parent_id': 10, 'offset': [-0.17269200086593628, 0.0, 0.0]},
    12: {'name': b'Skeleton 002_RightElbow', 'parent_id': 11, 'offset': [-0.24771900475025177, 0.0, 0.0]},
    13: {'name': b'Skeleton 002_RightWrist', 'parent_id': 12, 'offset': [-0.23940299451351166, 0.0, 0.0]},
    14: {'name': b'Skeleton 002_LeftHip', 'parent_id': 1, 'offset': [0.09922870248556137, 0.0, 0.0]},
    15: {'name': b'Skeleton 002_LeftKnee', 'parent_id': 14, 'offset': [-0.0, -0.44121500849723816, 0.0]},
    16: {'name': b'Skeleton 002_LeftAnkle', 'parent_id': 15, 'offset': [-0.0, -0.44642001390457153, 0.0]},
    17: {'name': b'Skeleton 002_LeftToe', 'parent_id': 16, 'offset': [-0.0, -0.06449870020151138, 0.14884300529956818]},
    18: {'name': b'Skeleton 002_RightHip', 'parent_id': 1, 'offset': [-0.09922870248556137, 0.0, 0.0]},
    19: {'name': b'Skeleton 002_RightKnee', 'parent_id': 18, 'offset': [-0.0, -0.44121500849723816, 0.0]},
    20: {'name': b'Skeleton 002_RightAnkle', 'parent_id': 19, 'offset': [-0.0, -0.44642001390457153, 0.0]},
    21: {'name': b'Skeleton 002_RightToe', 'parent_id': 20, 'offset': [-0.0, -0.06449870020151138, 0.14884300529956818]},
}

# Actual frame data from OptiTrack streaming (already converted to Z-up and standard names)
# Format: {body_name: (position_xyz, quaternion_wxyz)}
frame_data = {
    'Hips': ((-0.002759732073172927, 0.1113460436463356, 0.9506919384002686), np.array([0.99945921, 0.00161138, 0.00100068, -0.0328293])),
    'Spine': ((-0.0025957131292670965, 0.11106149107217789, 1.037268877029419), np.array([9.99718666e-01, 2.73998687e-03, 5.87610994e-04, -2.35540755e-02])),
    'Spine1': ((-0.002372581046074629, 0.10988672822713852, 1.2506225109100342), np.array([0.99989003, 0.00382231, -0.0012271, -0.0142822])),
    'Neck': ((-0.002967696636915207, 0.10812009125947952, 1.4828006029129028), np.array([0.98826844, -0.03384563, 0.0040212, -0.14887507])),
    'Head': ((-0.005954049527645111, 0.09957992285490036, 1.6417815685272217), np.array([0.95718133, -0.07084692, 0.00919029, -0.28053567])),
    'LeftShoulder': ((0.03721929341554642, 0.11798439174890518, 1.3942842483520508), np.array([0.9859432, 0.12685536, -0.01081129, 0.10819843])),
    'LeftArm': ((0.20582756400108337, 0.15435540676116943, 1.4027063846588135), np.array([0.72625792, 0.31838971, 0.52654988, -0.30646807])),
    'LeftForeArm': ((0.2696511745452881, 0.12714260816574097, 1.1649024486541748), np.array([0.67076421, 0.01459338, 0.68553257, -0.28267914])),
    'LeftHand': ((0.24577677249908447, 0.04114577919244766, 0.9427575469017029), np.array([7.01771557e-01, 2.55047315e-04, 6.54760480e-01, -2.80722886e-01])),
    'RightShoulder': ((-0.04211028665304184, 0.12025181949138641, 1.394098162651062), np.array([0.98475313, 0.104957, -0.03119964, -0.13517331])),
    'RightArm': ((-0.20815527439117432, 0.1673576980829239, 1.3883867263793945), np.array([0.76482046, 0.22395104, -0.55484265, 0.23884168])),
    'RightForeArm': ((-0.2750910520553589, 0.13841748237609863, 1.1516447067260742), np.array([0.73649824, -0.00811137, -0.63951254, 0.22029138])),
    'RightHand': ((-0.29543817043304443, 0.05825027823448181, 0.9269827008247375), np.array([0.71421516, -0.00747435, -0.66272962, 0.2250118])),
    'LeftUpLeg': ((0.0962548777461052, 0.1048346683382988, 0.950482964515686), np.array([0.99809128, -0.05513556, -0.02550886, -0.01110467])),
    'LeftLeg': ((0.11818142235279083, 0.05602429434657097, 0.5125247240066528), np.array([0.99294376, 0.11527719, -0.02702492, -0.00660831])),
    'LeftFoot': ((0.14282025396823883, 0.15806268155574799, 0.07862159609794617), np.array([0.99469465, -0.06285419, -0.06936577, 0.04266561])),
    'LeftToeBase': ((0.16340239346027374, 0.0032543838024139404, 0.03474576771259308), np.array([0.99656886, 0.01478802, -0.06583304, 0.04793721])),
    'RightUpLeg': ((-0.10177434235811234, 0.1178574189543724, 0.9509009122848511), np.array([0.99522239, -0.08796262, 0.02293959, -0.03562177])),
    'RightLeg': ((-0.12468519806861877, 0.04132848232984543, 0.5169780254364014), np.array([0.99234569, 0.11599505, 0.01522691, -0.03953829])),
    'RightFoot': ((-0.13408154249191284, 0.144638329744339, 0.08277806639671326), np.array([0.99706423, -0.01356165, 0.04443965, -0.06086231])),
    'RightToeBase': ((-0.15778905153274536, -0.0044426023960113525, 0.023388266563415527), np.array([0.9969061, -0.02234125, 0.04497388, -0.06046862])),
}

# Convert frame data to the format expected by GMR: {body_name: (pos_array, quat_wxyz_array)}
def convert_frame(frame):
    result = {}
    for name, (pos, quat) in frame.items():
        result[name] = (np.array(pos), np.array(quat))
    return result

# Save data
data = {
    'rb_desc': rb_desc,
    'frames': [convert_frame(frame_data)],  # List of frames for playback
    'fps': 30.0,
    'format': 'optitrack_streaming',
}

# Get package directory (parent of scripts/)
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(script_dir)
output_path = os.path.join(pkg_dir, 'data', 'sample_optitrack.pkl')
os.makedirs(os.path.dirname(output_path), exist_ok=True)

with open(output_path, 'wb') as f:
    pickle.dump(data, f)

print(f"Sample data saved to: {output_path}")
print(f"Contains {len(data['frames'])} frames")
print(f"Bodies: {list(frame_data.keys())}")
