import sys
import os
current_path = os.path.dirname(os.path.abspath(__file__))
library_path = os.path.join(current_path, 'python-can-3.3.4')
sys.path.append(library_path) 

current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
os.environ["LD_LIBRARY_PATH"] = f"{library_path}:{current_ld_library_path}"

import can
from can.bus import BusState


class WHJ30Tools:
    def __init__(self):
        self.dev = None
        self.dev_info = {
            "dev_type": "bmcan",
            "dev_channel": 0,
            "bit_rate": 1000000,
            "data_rate": 5000000,
            "terminal_res": True,
            "timeout": 1,
        }
        self.instruct_type = {"CMD_RD": 0x01, "CMD_WR": 0x02}
        self.write_type = {"W_SUCC": 0x01, "W_FAIL": 0x00}
        self.control_table = {
            "SYS_ID": 0x01,
            "SYS_ENABLE_DRIVER": 0x0A,
            "SYS_SAVE_TO_FLASH": 0x0C,
            "SYS_SET_ZERO_POS": 0x0E,
            "SYS_CLEAR_ERROR": 0x0F,
            "TAG_WORK_MODE": 0x30,
            "LIT_MIN_POSITION_L": 0x44,
            "LIT_MIN_POSITION_H": 0x45,
            "LIT_MAX_POSITION_L": 0x46,
            "LIT_MAX_POSITION_H": 0x47,
            "IAP_FLAG": 0x49,
        }

    def open_canbus(self):
        try:
            self.dev = can.interface.Bus(
                bustype=self.dev_info["dev_type"],
                channel=self.dev_info["dev_channel"],
                bitrate=self.dev_info["bit_rate"],
                data_bitrate=self.dev_info["data_rate"],
                tres=self.dev_info["terminal_res"],
            )
            if self.dev.state == BusState.ACTIVE:
                print("open bus port succ")
                return True
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def close_canbus(self):
        try:
            self.dev.shutdown()

            return True

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def write_reg(self, dev_id, addr, dat, len):
        try:
            tx_msg = can.Message(
                arbitration_id=dev_id,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True,
                dlc=0x02 + len,
                data=[self.instruct_type["CMD_WR"], addr],
            )
            for i in range(0, len):
                tx_msg.data.append((dat >> i * 8) & 0xFF)

            self.dev.send(tx_msg, self.dev_info["timeout"])

            rx_msg = self.dev.recv(self.dev_info["timeout"])
            if rx_msg is not None:
                if (
                    rx_msg.arbitration_id == (dev_id | 0x100)
                    and rx_msg.dlc == 0x03
                    and rx_msg.data[0] == self.instruct_type["CMD_WR"]
                    and rx_msg.data[1] == addr
                    and rx_msg.data[2] == self.write_type["W_SUCC"]
                ):
                    return True
                else:
                    return rx_msg
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def read_reg(self, dev_id, addr, len):
        try:
            tx_msg = can.Message(
                arbitration_id=dev_id,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True,
                dlc=0x02 + len,
                data=[self.instruct_type["CMD_RD"], addr, len],
            )
            self.dev.send(tx_msg, self.dev_info["timeout"])

            rx_msg = self.dev.recv(self.dev_info["timeout"])
            if rx_msg is not None:
                if (
                    rx_msg.arbitration_id == (dev_id | 0x100)
                    and rx_msg.data[0] == self.instruct_type["CMD_RD"]
                    and rx_msg.data[1] == addr
                ):
                    return rx_msg.data[2:]
                else:
                    return rx_msg
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def skip_iap_update(self, dev_id):
        try:
            return self.write_reg(
                dev_id, self.control_table["IAP_FLAG"], self.write_type["W_FAIL"], 1
            )

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def enable_joint(self, dev_id):
        try:
            return self.write_reg(
                dev_id,
                self.control_table["SYS_ENABLE_DRIVER"],
                self.write_type["W_SUCC"],
                1,
            )

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def disable_joint(self, dev_id):
        try:
            return self.write_reg(
                dev_id,
                self.control_table["SYS_ENABLE_DRIVER"],
                self.write_type["W_FAIL"],
                1,
            )

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def get_joint_state(self, dev_id):
        try:
            tx_msg = can.Message(
                arbitration_id=(dev_id | 0x600),
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True,
                dlc=0x00,
            )
            self.dev.send(tx_msg, self.dev_info["timeout"])

            rx_msg = self.dev.recv(self.dev_info["timeout"])
            if rx_msg is not None:
                if rx_msg.arbitration_id == (dev_id | 0x700) and rx_msg.dlc == 0x10:
                    state_list = []

                    err_code = rx_msg.data[1] << 8 | rx_msg.data[0]
                    state_list.append(err_code)

                    sys_vol = (rx_msg.data[3] << 8 | rx_msg.data[2]) * 0.01
                    state_list.append(sys_vol)

                    sys_tempera = (rx_msg.data[5] << 8 | rx_msg.data[4]) * 0.1
                    state_list.append(sys_tempera)

                    enable_state = rx_msg.data[6]
                    state_list.append(enable_state)

                    brake_state = rx_msg.data[7]
                    state_list.append(brake_state)

                    actual_pos = (
                        rx_msg.data[11] << 24
                        | rx_msg.data[10] << 16
                        | rx_msg.data[9] << 8
                        | rx_msg.data[8]
                    )
                    if actual_pos > 0x7FFFFFFF:
                        actual_pos = actual_pos - (1 << 32)
                    actual_pos = actual_pos * 0.0001
                    state_list.append(actual_pos)

                    actual_cur = (
                        rx_msg.data[15] << 24
                        | rx_msg.data[14] << 16
                        | rx_msg.data[13] << 8
                        | rx_msg.data[12]
                    )
                    if actual_cur > 0x7FFFFFFF:
                        actual_cur = actual_cur - (1 << 32)
                    state_list.append(actual_cur)

                    return state_list
                else:
                    return rx_msg
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def clear_joint_error(self, dev_id):
        try:
            return self.write_reg(
                dev_id,
                self.control_table["SYS_CLEAR_ERROR"],
                self.write_type["W_SUCC"],
                1,
            )

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def set_joint_id(self, dev_id, new_id):
        try:
            return self.write_reg(dev_id, self.control_table["SYS_ID"], new_id, 1)

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def set_joint_zero_position(self, dev_id):
        try:
            return self.write_reg(
                dev_id,
                self.control_table["SYS_SET_ZERO_POS"],
                self.write_type["W_SUCC"],
                1,
            )

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def save_joint_param(self, dev_id):
        try:
            return self.write_reg(
                dev_id,
                self.control_table["SYS_SAVE_TO_FLASH"],
                self.write_type["W_SUCC"],
                1,
            )

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def set_joint_operate_mode(self, dev_id, operate_mode):
        try:
            return self.write_reg(
                dev_id,
                self.control_table["TAG_WORK_MODE"],
                operate_mode,
                1,
            )

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def set_joint_position(self, dev_id, target_position):
        try:
            target_position = int(target_position * 10000)
            if target_position < 0:
                target_position = target_position + (1 << 32)

            tx_msg = can.Message(
                arbitration_id=(dev_id | 0x200),
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True,
                dlc=0x04,
                data=[
                    target_position & 0xFF,
                    (target_position >> 8) & 0xFF,
                    (target_position >> 16) & 0xFF,
                    (target_position >> 24) & 0xFF,
                ],
            )
            self.dev.send(tx_msg, self.dev_info["timeout"])

            rx_msg = self.dev.recv(self.dev_info["timeout"])
            if rx_msg is not None:
                if rx_msg.arbitration_id == (dev_id | 0x500) and rx_msg.dlc == 0x10:
                    state_list = []

                    actual_cur = (
                        rx_msg.data[3] << 24
                        | rx_msg.data[2] << 16
                        | rx_msg.data[1] << 8
                        | rx_msg.data[0]
                    )
                    if actual_cur > 0x7FFFFFFF:
                        actual_cur = actual_cur - (1 << 32)
                    actual_cur = actual_cur
                    state_list.append(actual_cur)

                    actual_speed = (
                        rx_msg.data[7] << 24
                        | rx_msg.data[6] << 16
                        | rx_msg.data[5] << 8
                        | rx_msg.data[4]
                    )
                    if actual_speed > 0x7FFFFFFF:
                        actual_speed = actual_speed - (1 << 32)
                    actual_speed = actual_speed * 0.02
                    state_list.append(actual_speed)

                    actual_pos = (
                        rx_msg.data[11] << 24
                        | rx_msg.data[10] << 16
                        | rx_msg.data[9] << 8
                        | rx_msg.data[8]
                    )
                    if actual_pos > 0x7FFFFFFF:
                        actual_pos = actual_pos - (1 << 32)
                    actual_pos = actual_pos * 0.0001
                    state_list.append(actual_pos)

                    enable_state = rx_msg.data[13] << 8 | rx_msg.data[12]
                    state_list.append(enable_state)

                    err_code = rx_msg.data[15] << 8 | rx_msg.data[14]
                    state_list.append(err_code)

                    return state_list
                else:
                    return rx_msg
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def set_joint_min_position(self, dev_id, min_position):
        try:
            min_position = int(min_position * 10000)
            if min_position < 0:
                min_position = min_position + (1 << 32)

            if (
                self.write_reg(
                    dev_id,
                    self.control_table["LIT_MIN_POSITION_L"],
                    min_position,
                    2,
                )
            ) == True:
                if (
                    self.write_reg(
                        dev_id,
                        self.control_table["LIT_MIN_POSITION_H"],
                        min_position >> 16,
                        2,
                    )
                ) == True:
                    return True
                else:
                    return False
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def set_joint_max_position(self, dev_id, max_position):
        try:
            max_position = int(max_position * 10000)
            if max_position < 0:
                max_position = max_position + (1 << 32)

            if (
                self.write_reg(
                    dev_id,
                    self.control_table["LIT_MAX_POSITION_L"],
                    max_position,
                    2,
                )
            ) == True:
                if (
                    self.write_reg(
                        dev_id,
                        self.control_table["LIT_MAX_POSITION_H"],
                        max_position >> 16,
                        2,
                    )
                ) == True:
                    return True
                else:
                    return False
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg
if "__main__" == __name__:
    wh300 = WHJ30Tools()
    wh300.open_canbus()
    print(wh300.get_joint_state(0x00000001))
