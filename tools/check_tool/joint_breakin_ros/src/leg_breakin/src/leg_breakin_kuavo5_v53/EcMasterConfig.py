import os
import sys

BIT_17 = 1 << 17
BIT_17_8 = BIT_17 * 8
BIT_17_9 = BIT_17 * 9
BIT_17_10 = BIT_17 * 10
BIT_17_18 = BIT_17 * 18
BIT_17_36 = BIT_17 * 36
BIT_17_25 = BIT_17 * 25

# 机器人类型 -> 电机id(int) -> 编码器范围
ENCODER_RANGE_TABLE = {
    13: {
        1: BIT_17_25,
        2: BIT_17_25, 3: BIT_17_36, 4: BIT_17_25, 5: BIT_17_36, 6: BIT_17_25, 7: BIT_17_25,
        8: BIT_17_25, 9: BIT_17_36, 10: BIT_17_25, 11: BIT_17_36, 12: BIT_17_25, 13: BIT_17_25
    },
    14: {
        1: BIT_17_25,
        2: BIT_17_25, 3: BIT_17_36, 4: BIT_17_25, 5: BIT_17_36, 6: BIT_17_25, 7: BIT_17_25,
        8: BIT_17_25, 9: BIT_17_36, 10: BIT_17_25, 11: BIT_17_36, 12: BIT_17_25, 13: BIT_17_25
    },
    42: {
        1: BIT_17_18, 2: BIT_17_10, 3: BIT_17_10, 4: BIT_17_18, 5: BIT_17_36, 6: BIT_17_36,
        7: BIT_17_18,8: BIT_17_10, 9: BIT_17_10, 10: BIT_17_18, 11: BIT_17_36, 12: BIT_17_36,
        13: BIT_17_10, 14: BIT_17_10
    },
    45: {
        1: BIT_17_18, 2: BIT_17_10, 3: BIT_17_10, 4: BIT_17_18, 5: BIT_17_36, 6: BIT_17_36,
        7: BIT_17_18,8: BIT_17_10, 9: BIT_17_10, 10: BIT_17_18, 11: BIT_17_36, 12: BIT_17_36,
        13: BIT_17_10, 14: BIT_17_10
    },
    49: {
        1: BIT_17_18, 2: BIT_17_10, 3: BIT_17_10, 4: BIT_17_18, 5: BIT_17_36, 6: BIT_17_36,
        7: BIT_17_18,8: BIT_17_10, 9: BIT_17_10, 10: BIT_17_18, 11: BIT_17_36, 12: BIT_17_36,
        13: BIT_17_10, 14: BIT_17_10
    },
    51: {
        1: BIT_17_18, 2: BIT_17_25, 3: BIT_17_18, 4: BIT_17_18, 5: BIT_17_18, 6: BIT_17_18,
        7: BIT_17_18, 8: BIT_17_25, 9: BIT_17_18, 10: BIT_17_18, 11: BIT_17_18, 12: BIT_17_18,
        13: BIT_17_25, 14: BIT_17_10, 15: BIT_17_10
    },
    52: {
        1: BIT_17_18, 2: BIT_17_25, 3: BIT_17_18, 4: BIT_17_18, 5: BIT_17_18, 6: BIT_17_18,
        7: BIT_17_18, 8: BIT_17_25, 9: BIT_17_18, 10: BIT_17_18, 11: BIT_17_18, 12: BIT_17_18,
        13: BIT_17_25, 14: BIT_17_10, 15: BIT_17_10
    },
    53: {
        1: BIT_17_18, 2: BIT_17_25, 3: BIT_17_18, 4: BIT_17_18, 5: BIT_17_18, 6: BIT_17_18,
        7: BIT_17_18, 8: BIT_17_25, 9: BIT_17_18, 10: BIT_17_18, 11: BIT_17_18, 12: BIT_17_18,
        13: BIT_17_25
    },
    10045: {
        1: BIT_17_18, 2: BIT_17_10, 3: BIT_17_10, 4: BIT_17_18, 5: BIT_17_36, 6: BIT_17_36,
        7: BIT_17_18,8: BIT_17_10, 9: BIT_17_10, 10: BIT_17_18, 11: BIT_17_36, 12: BIT_17_36,
        13: BIT_17_10, 14: BIT_17_10
    },
    10049: {
        1: BIT_17_18, 2: BIT_17_10, 3: BIT_17_10, 4: BIT_17_18, 5: BIT_17_36, 6: BIT_17_36,
        7: BIT_17_18,8: BIT_17_10, 9: BIT_17_10, 10: BIT_17_18, 11: BIT_17_36, 12: BIT_17_36,
        13: BIT_17_10, 14: BIT_17_10
    }
}

# Joint -> Slave
JOINT2SLAVE_TABLE = {
    13: {
        1: 1,
        2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7,
        8: 13, 9: 12, 10: 11, 11: 10, 12: 9, 13: 8
    },
    14: {
        1: 1,
        2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7,
        8: 13, 9: 12, 10: 11, 11: 10, 12: 9, 13: 8
    },
    42: {
        1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6,
        7: 7, 8: 8, 9: 9, 10: 10, 11: 11, 12: 12,
        13: 13, 14: 14
    },
    45: {
        1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6,
        7: 7, 8: 8, 9: 9, 10: 10, 11: 11, 12: 12,
        13: 13, 14: 14
    },
    49: {
        1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6,
        7: 7, 8: 8, 9: 9, 10: 10, 11: 11, 12: 12,
        13: 13, 14: 14
    },
    51: {
        1: 2, 2: 3, 3: 4, 4: 5, 5: 6, 6: 7,
        7: 8, 8: 9, 9: 10, 10: 11, 11: 12, 12: 13,
        13: 14,
        14: 15, 15: 16
    },
    52: {
        1: 2, 2: 3, 3: 4, 4: 5, 5: 6, 6: 7,
        7: 8, 8: 9, 9: 10, 10: 11, 11: 12, 12: 13,
        13: 14,
        14: 15, 15: 16
    },
    53: {
        1: 2, 2: 3, 3: 4, 4: 5, 5: 6, 6: 7,
        7: 8, 8: 9, 9: 10, 10: 11, 11: 12, 12: 13,
        13: 14,
    },
    10045: {
        1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6,
        7: 7, 8: 8, 9: 9, 10: 10, 11: 11, 12: 12,
        13: 13, 14: 14
    },
    10049: {
        1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6,
        7: 7, 8: 8, 9: 9, 10: 10, 11: 11, 12: 12,
        13: 13, 14: 14
    }
}

class EcMasterConfig:
    def __init__(self):
        self.driver_type = None
        self.slave_num = None
        self.ec_master_type_ini_path = "~/.config/lejuconfig/EcMasterType.ini"
        self.command_args = None
        self.robot_version = None
        self.slave2joint = {}

        self.initialize()

    def get_encoder_range(self, joint_id):
        robot_table = ENCODER_RANGE_TABLE.get(self.robot_version)
        if robot_table is None:
            print(f"\033[33mwarning: 未找到ROBOT_VERSION={self.robot_version}的编码器映射表\033[0m")
            return None
        encoder_range = robot_table.get(joint_id)
        if encoder_range is None:
            print(f"\033[33mwarning: ROBOT_VERSION={self.robot_version} 未配置 joint_id={joint_id} 的编码器范围\033[0m")
        return encoder_range

    def getSlaveIdByJointId(self, joint_id):
        joint2slave_table = JOINT2SLAVE_TABLE.get(self.robot_version)
        if joint2slave_table is None:
            print(f"\033[33mwarning: 未找到ROBOT_VERSION={self.robot_version}的JOINT2SLAVE_TABLE\033[0m")
            return None
        slave_id = joint2slave_table.get(joint_id)
        if slave_id is None:
            print(f"\033[33mwarning: ROBOT_VERSION={self.robot_version} 未配置 joint_id={joint_id} 的从站ID\033[0m")
        return slave_id

    def initialize_slave2joint_mapping(self):
        joint2slave_table = JOINT2SLAVE_TABLE.get(self.robot_version)
        if joint2slave_table is None:
            print(f"\033[33mwarning: 未找到ROBOT_VERSION={self.robot_version}的JOINT2SLAVE_TABLE\033[0m")
            self.slave2joint = {}
            return
        
        self.slave2joint = {}
        for joint_id, slave_id in joint2slave_table.items():
            self.slave2joint[slave_id] = joint_id

    def get_ecmaster_driver_type(self, ini_path):
        file_path = os.path.expanduser(ini_path)
        ec_master_type = None
        try:
            with open(file_path, "r") as f:
                ec_master_type = f.readline().strip()
        except FileNotFoundError:
            print(f"\033[33mwarning: {file_path} 文件不存在, 未指定EcMasterType, 使用默认值 'elmo' 驱动器类型\033[0m", file=sys.stderr)
            return "elmo"
        if ec_master_type not in ["elmo", "youda", "youda3", "leju"]:
            print(f"\033[33mwarning: ecmaster_type :{ec_master_type} error, 使用默认值 'elmo' 驱动器类型\033[0m", file=sys.stderr)
            return "elmo"
        return ec_master_type

    def get_robot_version(self):
        value = os.environ.get("ROBOT_VERSION")
        if value is not None:
            try:
                return int(value)
            except ValueError:
                print("\033[33mwarning: 环境变量ROBOT_VERSION不是有效的整数\033[0m", file=sys.stderr)
                return 52
        else:
            print("\033[33mwarning: 未设置环境变量ROBOT_VERSION\033[0m", file=sys.stderr)
            return 52

    def get_robot_type_and_slave_num(self, robot_version):
        if robot_version == 13 or robot_version == 14:
            return "roban2", 13
        elif robot_version == 42 or robot_version == 45 or robot_version == 49 or robot_version == 10045 or robot_version == 10049:
            return "kuavo", 14
        elif robot_version == 51 or robot_version == 52:
            return "kuavo5_v52", 15
        elif robot_version == 53:
            return "kuavo5_v53", 13
        else:
            print(f"\033[33mwarning: 机器人版本不存在, 使用默认值 'kuavo' 机器人类型\033[0m", file=sys.stderr)
            return "kuavo", 14

    def get_ec_eni_config(self, driver_type, slave_num):
        if driver_type == "elmo":
            return f"./config/ENI_config/elmo_{slave_num}_c500.xml"
        elif driver_type == "youda":
            # Version 53 使用 Kuavo5_T26_yangxu.xml
            if self.robot_version == 53:
                return f"./config/ENI_config/Kuavo5_T26_yangxu.xml"
            else:
                return f"./config/ENI_config/Kuavo5_T25_kpkd.xml"
            # return f"./config/ENI_config/yd_{slave_num}_c501.xml"
        elif driver_type == "youda3":
            return f"./config/ENI_config/yd300_{slave_num}_c501.xml"

    def build_command(self, eni_config_path):
        base_command = [
            "main",
            "-i8254x", "2", "1",
            "-a", "7",
            "-v", "2",
            "-auxclk", "500",
            "-dcmmode", "mastershift",
            "-t", "0",
            "-log", "./EC_log/EC_log"
        ]
        base_command.extend(['-f', eni_config_path])
        return base_command

    def initialize(self):
        self.robot_version = self.get_robot_version()
        robot_type, self.slave_num = self.get_robot_type_and_slave_num(self.robot_version)
        self.driver_type = self.get_ecmaster_driver_type(self.ec_master_type_ini_path)
        eni_config_path = self.get_ec_eni_config(self.driver_type, self.slave_num)
        self.command_args = self.build_command(eni_config_path)
        self.initialize_slave2joint_mapping()

        print("\033[1;34m====== EC Master 配置参数 ======\033[0m")
        print(f"\033[1;33m驱动器类型 ENI 文件路径:\033[0m {eni_config_path}")
        print(f"\033[1;33mROBOT_VERSION:\033[0m {self.robot_version}")
        print(f"\033[1;33mROBOT_TYPE:\033[0m {robot_type}")
        print(f"\033[1;33m从站数量:\033[0m {self.slave_num}")
        print(f"\033[1;33m驱动器类型:\033[0m {self.driver_type}")
        # print(f"\033[1;33mEC_Master 命令:\033[0m {self.command_args}")
