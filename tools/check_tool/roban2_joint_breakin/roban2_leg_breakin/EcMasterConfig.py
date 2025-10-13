import os
import sys

BIT_17 = 1 << 17
BIT_17_8 = BIT_17 * 8
BIT_17_9 = BIT_17 * 9
BIT_17_10 = BIT_17 * 10
BIT_17_18 = BIT_17 * 18
BIT_17_36 = BIT_17 * 36
BIT_17_25 = BIT_17 * 25

# 机器人类型 -> 电机id(int) -> 编码器范围 (仅支持13、14版本)
ENCODER_RANGE_TABLE = {
    13: {
        1: BIT_17_25,
        2: BIT_17_25, 3: BIT_17_36, 4: BIT_17_25, 5: BIT_17_36, 6: BIT_17_25, 7: BIT_17_25,
        8: BIT_17_25, 9: BIT_17_36, 10: BIT_17_25, 11: BIT_17_36, 12: BIT_17_25, 13: BIT_17_25
    },
    14: {
        1: BIT_17_25,
        2: BIT_17_25, 3: BIT_17_36, 4: BIT_17_25, 5: BIT_17_36, 6: BIT_17_25, 7: BIT_17_25,
        8: BIT_17_25, 9: BIT_17_25, 10: BIT_17_36, 11: BIT_17_25, 12: BIT_17_36, 13: BIT_17_25
    }
}

class EcMasterConfig:
    def __init__(self):
        self.driver_type = None
        self.slave_num = None
        self.driver_config_json_path = "./config/sdo_config/dirver_kuavo.json"
        self.ec_master_type_ini_path = "~/.config/lejuconfig/EcMasterType.ini"
        self.command_args = None
        self.robot_version = None
        self.initialize()

    def get_encoder_range(self, slave_id):
        robot_table = ENCODER_RANGE_TABLE.get(self.robot_version)
        if robot_table is None:
            print(f"\033[33mwarning: 未找到ROBOT_VERSION={self.robot_version}的编码器映射表\033[0m")
            return None
        encoder_range = robot_table.get(slave_id)
        if encoder_range is None:
            print(f"\033[33mwarning: ROBOT_VERSION={self.robot_version} 未配置 slave_id={slave_id} 的编码器范围\033[0m")
        return encoder_range

    def get_driver_config_json_path(self, robot_type):
        return "./config/sdo_config/dirver_roban2.json"

    def get_ecmaster_driver_type(self, ini_path):
        file_path = os.path.expanduser(ini_path)
        ec_master_type = None
        try:
            with open(file_path, "r") as f:
                ec_master_type = f.readline().strip()
        except FileNotFoundError:
            print(f"\033[33mwarning: {file_path} 文件不存在, 未指定EcMasterType, 使用默认值 'youda' 驱动器类型\033[0m", file=sys.stderr)
            return "youda"
        if ec_master_type not in ["youda", "youda3"]:
            print(f"\033[33mwarning: ecmaster_type :{ec_master_type} error, 使用默认值 'youda' 驱动器类型\033[0m", file=sys.stderr)
            return "youda"
        return ec_master_type

    def get_robot_version(self):
        value = os.environ.get("ROBOT_VERSION")
        if value is not None:
            try:
                version = int(value)
                if version not in [13, 14]:
                    print(f"\033[33mwarning: 不支持的机器人版本 {version}，仅支持13、14版本，使用默认值14\033[0m", file=sys.stderr)
                    return 14
                return version
            except ValueError:
                print("\033[33mwarning: 环境变量ROBOT_VERSION不是有效的整数\033[0m", file=sys.stderr)
                return 14
        else:
            print("\033[33mwarning: 未设置环境变量ROBOT_VERSION，使用默认值14\033[0m", file=sys.stderr)
            return 14

    def get_robot_type_and_slave_num(self, robot_version):
        # 只支持13、14版本，都是roban2类型
        return "roban2", 13

    def get_ec_eni_config(self, driver_type, slave_num):
        if driver_type == "youda":
            return f"./config/ENI_config/yd_{slave_num}_c501.xml"
        elif driver_type == "youda3":
            return f"./config/ENI_config/yd300_{slave_num}_c501.xml"
        else:
            # 默认使用youda驱动器
            return f"./config/ENI_config/yd_{slave_num}_c501.xml"

    def build_command(self, eni_config_path):
        base_command = [
            "main",
            "-i8254x", "1", "1",
            "-i8254x", "2", "1",
            "-a", "7",
            "-v", "0",
            "-auxclk", "500",
            "-dcmmode", "mastershift",
            "-t", "0"
        ]
        base_command.extend(['-f', eni_config_path])
        return base_command

    def initialize(self):
        self.robot_version = self.get_robot_version()
        robot_type, self.slave_num = self.get_robot_type_and_slave_num(self.robot_version)
        self.driver_type = self.get_ecmaster_driver_type(self.ec_master_type_ini_path)
        self.driver_config_json_path = self.get_driver_config_json_path(robot_type)
        eni_config_path = self.get_ec_eni_config(self.driver_type, self.slave_num)
        self.command_args = self.build_command(eni_config_path)
        # 打印信息
        # print("\033[1;34m====== EC Master 配置参数 ======\033[0m")
        # print(f"\033[1;33m驱动器类型 ENI 文件路径:\033[0m {eni_config_path}")
        # print(f"\033[1;33mROBOT_VERSION:\033[0m {self.robot_version}")
        # print(f"\033[1;33m从站数量:\033[0m {self.slave_num}")
        # print(f"\033[1;33m驱动器类型:\033[0m {self.driver_type}")
        # print(f"\033[1;33mEC_Master 命令:\033[0m {self.command_args}")
