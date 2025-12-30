import sys
import os
import time

current_path = os.path.dirname(os.path.abspath(__file__))
library_path = os.path.join(current_path, "python-can-3.3.4")
sys.path.append(library_path)

current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
os.environ["LD_LIBRARY_PATH"] = f"{library_path}:{current_ld_library_path}"
import can
from can.bus import BusState


class RUIWOTools:
    def __init__(self):
        self.dev = None
        self.dev_info = {
            "dev_type": "bmcan",
            "dev_channel": 0,
            "bit_rate": 1000000,
            "data_rate": 1000000,
            "terminal_res": True,
            "timeout": 1,
            "notimeout": 0,
        }
        self.can_com_param = {
            "CAN_COM_THETA_MIN": -12.5,
            "CAN_COM_THETA_MAX": 12.5,
            "CAN_COM_VELOCITY_MIN": -10.0,
            "CAN_COM_VELOCITY_MAX": 10.0,
            "CAN_COM_TORQUE_MIN": -50.0,
            "CAN_COM_TORQUE_MAX": 50.0,
            "CAN_COM_POS_KP_MIN": 0.0,
            "CAN_COM_POS_KP_MAX": 250.0,
            "CAN_COM_POS_KD_MIN": 0.0,
            "CAN_COM_POS_KD_MAX": 50.0,
            "CAN_COM_VEL_KP_MIN": 0.0,
            "CAN_COM_VEL_KP_MAX": 250.0,
            "CAN_COM_VEL_KD_MIN": 0.0,
            "CAN_COM_VEL_KD_MAX": 50.0,
            "CAN_COM_VEL_KI_MIN": 0.0,
            "CAN_COM_VEL_KI_MAX": 0.05,
        }

    def open_canbus(self):
        result = True
        try:
            self.dev = can.interface.Bus(
                bustype=self.dev_info["dev_type"],
                channel=self.dev_info["dev_channel"],
                bitrate=self.dev_info["bit_rate"],
                data_bitrate=self.dev_info["data_rate"],
                tres=self.dev_info["terminal_res"],
            )

            if self.dev.state != BusState.ACTIVE:
                result = False
        except Exception as exc:
            exc_msg = str(exc)
            print(exc_msg)
            result = False
        finally:
            return result

    def close_canbus(self):
        try:
            self.dev.shutdown()
            return True

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def __float_to_int(self, float_num, min_num, max_num, bit_num):
        return int((float_num - min_num) / (max_num - min_num) * (2**bit_num - 1))

    def __int_to_float(self, int_num, min_num, max_num, bit_num):
        return int_num / (2**bit_num - 1) * (max_num - min_num) + min_num

    def return_motor_state(self, feedback_frame):
        state_list = []

        id = feedback_frame[0]
        state_list.append(id)

        pos = feedback_frame[1] << 8 | feedback_frame[2]
        pos = self.__int_to_float(
            pos,
            self.can_com_param["CAN_COM_THETA_MIN"],
            self.can_com_param["CAN_COM_THETA_MAX"],
            16,
        )
        state_list.append(pos)

        vel = (feedback_frame[3] << 4) | ((feedback_frame[4] >> 4) & 0x0F)
        vel = self.__int_to_float(
            vel,
            self.can_com_param["CAN_COM_VELOCITY_MIN"],
            self.can_com_param["CAN_COM_VELOCITY_MAX"],
            12,
        )
        state_list.append(vel)

        torque = ((feedback_frame[4] & 0x0F) << 8) | feedback_frame[5]
        torque = self.__int_to_float(
            torque,
            self.can_com_param["CAN_COM_TORQUE_MIN"],
            self.can_com_param["CAN_COM_TORQUE_MAX"],
            12,
        )
        state_list.append(torque)

        tempera = feedback_frame[6]
        state_list.append(tempera)

        errcode = feedback_frame[7]
        state_list.append(errcode)

        return state_list
    
    def __return_motor_state(self, feedback_frame):
        state_list = []

        id = feedback_frame[0]
        state_list.append(id)

        pos = feedback_frame[1] << 8 | feedback_frame[2]
        pos = self.__int_to_float(
            pos,
            self.can_com_param["CAN_COM_THETA_MIN"],
            self.can_com_param["CAN_COM_THETA_MAX"],
            16,
        )
        state_list.append(pos)

        vel = (feedback_frame[3] << 4) | ((feedback_frame[4] >> 4) & 0x0F)
        vel = self.__int_to_float(
            vel,
            self.can_com_param["CAN_COM_VELOCITY_MIN"],
            self.can_com_param["CAN_COM_VELOCITY_MAX"],
            12,
        )
        state_list.append(vel)

        torque = ((feedback_frame[4] & 0x0F) << 8) | feedback_frame[5]
        torque = self.__int_to_float(
            torque,
            self.can_com_param["CAN_COM_TORQUE_MIN"],
            self.can_com_param["CAN_COM_TORQUE_MAX"],
            12,
        )
        state_list.append(torque)

        tempera = feedback_frame[6]
        state_list.append(tempera)

        errcode = feedback_frame[7]
        state_list.append(errcode)

        return state_list

    def enter_motor_state(self, dev_id):
        try:
            tx_msg = can.Message(
                arbitration_id=dev_id,
                is_extended_id=False,
                dlc=0x08,
                data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
            )
            self.dev.send(tx_msg, self.dev_info["timeout"])

            rx_msg = self.dev.recv(self.dev_info["timeout"])
            if rx_msg is not None:
                if rx_msg.arbitration_id == dev_id:
                    return self.__return_motor_state(rx_msg.data)
                else:
                    return rx_msg
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def enter_reset_state(self, dev_id):
        try:
            tx_msg = can.Message(
                arbitration_id=dev_id,
                is_extended_id=False,
                dlc=0x08,
                data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
            )
            self.dev.send(tx_msg, self.dev_info["timeout"])

            rx_msg = self.dev.recv(self.dev_info["timeout"])
            if rx_msg is not None:
                if rx_msg.arbitration_id == dev_id:
                    return self.__return_motor_state(rx_msg.data)
                else:
                    return rx_msg
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def set_zero_positon(self, dev_id):
        try:
            tx_msg = can.Message(
                arbitration_id=dev_id,
                is_extended_id=False,
                dlc=0x08,
                data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE],
            )
            self.dev.send(tx_msg, self.dev_info["timeout"])

            rx_msg = self.dev.recv(self.dev_info["timeout"])
            if rx_msg is not None:
                if rx_msg.arbitration_id == dev_id:
                    return self.__return_motor_state(rx_msg.data)
                else:
                    return rx_msg
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def multi_turn_zeroing(self, dev_ids = list(range(0x01, 0x0D))):        
        results = {}
        for dev_id in dev_ids:
            try:
                arbitration_id = 0x600 + dev_id
                print(f"Sending zeroing command to device {dev_id:02X} (Arbitration ID: {arbitration_id:03X})")
                
                tx_msg = can.Message(
                    arbitration_id=arbitration_id,
                    is_extended_id=False,
                    dlc=0x08,
                    data=[0x67, 0x06, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x76],
                )
                self.dev.send(tx_msg, self.dev_info["timeout"])
            except Exception as exc:
                results[dev_id] = str(exc)
                print(f"Error processing device {dev_id:02X}: {str(exc)}")
        
        return results

    def run_servo_mode(self, dev_id, pos, vel, pos_kp, pos_kd, vel_kp, vel_kd, vel_ki):
        try:
            pos = self.__float_to_int(
                pos,
                self.can_com_param["CAN_COM_THETA_MIN"],
                self.can_com_param["CAN_COM_THETA_MAX"],
                16,
            )
            vel = self.__float_to_int(
                vel,
                self.can_com_param["CAN_COM_VELOCITY_MIN"],
                self.can_com_param["CAN_COM_VELOCITY_MAX"],
                8,
            )
            pos_kp = self.__float_to_int(
                pos_kp,
                self.can_com_param["CAN_COM_POS_KP_MIN"],
                self.can_com_param["CAN_COM_POS_KP_MAX"],
                8,
            )
            pos_kd = self.__float_to_int(
                pos_kd,
                self.can_com_param["CAN_COM_POS_KD_MIN"],
                self.can_com_param["CAN_COM_POS_KD_MAX"],
                8,
            )
            vel_kp = self.__float_to_int(
                vel_kp,
                self.can_com_param["CAN_COM_VEL_KP_MIN"],
                self.can_com_param["CAN_COM_VEL_KP_MAX"],
                8,
            )
            vel_kd = self.__float_to_int(
                vel_kd,
                self.can_com_param["CAN_COM_VEL_KD_MIN"],
                self.can_com_param["CAN_COM_VEL_KD_MAX"],
                8,
            )
            vel_ki = self.__float_to_int(
                vel_ki,
                self.can_com_param["CAN_COM_VEL_KI_MIN"],
                self.can_com_param["CAN_COM_VEL_KI_MAX"],
                8,
            )

            tx_msg = can.Message(
                arbitration_id=dev_id,
                is_extended_id=False,
                dlc=0x08,
                data=[
                    (pos >> 8) & 0xFF,
                    pos & 0xFF,
                    vel,
                    pos_kp,
                    pos_kd,
                    vel_kp,
                    vel_kd,
                    vel_ki,
                ],
            )
            self.dev.send(tx_msg, self.dev_info["timeout"])

            rx_msg = self.dev.recv(self.dev_info["timeout"])
            if rx_msg is not None:
                if rx_msg.arbitration_id == dev_id:
                    return self.__return_motor_state(rx_msg.data)
                else:
                    return rx_msg
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def run_ptm_mode(self, dev_id, pos, vel, pos_kp, pos_kd, torque):
        try:
            pos = self.__float_to_int(
                pos,
                self.can_com_param["CAN_COM_THETA_MIN"],
                self.can_com_param["CAN_COM_THETA_MAX"],
                16,
            )
            vel = self.__float_to_int(
                vel,
                self.can_com_param["CAN_COM_VELOCITY_MIN"],
                self.can_com_param["CAN_COM_VELOCITY_MAX"],
                12,
            )
            pos_kp = self.__float_to_int(
                pos_kp,
                self.can_com_param["CAN_COM_POS_KP_MIN"],
                self.can_com_param["CAN_COM_POS_KP_MAX"],
                12,
            )
            pos_kd = self.__float_to_int(
                pos_kd,
                self.can_com_param["CAN_COM_POS_KD_MIN"],
                self.can_com_param["CAN_COM_POS_KD_MAX"],
                12,
            )
            torque = self.__float_to_int(
                torque,
                self.can_com_param["CAN_COM_TORQUE_MIN"],
                self.can_com_param["CAN_COM_TORQUE_MAX"],
                12,
            )

            tx_msg = can.Message(
                arbitration_id=dev_id,
                is_extended_id=False,
                dlc=0x08,
                data=[
                    (pos >> 8) & 0xFF,
                    pos & 0xFF,
                    (vel & 0xFF0) >> 4,
                    (vel & 0x0F) << 4 | (pos_kp & 0xF00) >> 8,
                    pos_kp & 0xFF,
                    (pos_kd & 0xFF0) >> 4,
                    (pos_kd & 0x0F) << 4 | (torque & 0xF00) >> 8,
                    torque & 0xFF,
                ],
            )
            # t0 = time.time()
            self.dev.send(tx_msg, self.dev_info["timeout"])
            # t1 = time.time()
            # print(f"send time: {t1-t0}")
            rx_msg = self.dev.recv(self.dev_info["timeout"])
            # print(f"recv time: {time.time()-t1}")
            if rx_msg is not None:
                if rx_msg.arbitration_id == dev_id:
                    return self.__return_motor_state(rx_msg.data)
                else:
                    return rx_msg
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def run_ptm_mode_No_response(self, dev_id, pos, vel, pos_kp, pos_kd, torque):
        try:
            pos = self.__float_to_int(
                pos,
                self.can_com_param["CAN_COM_THETA_MIN"],
                self.can_com_param["CAN_COM_THETA_MAX"],
                16,
            )
            vel = self.__float_to_int(
                vel,
                self.can_com_param["CAN_COM_VELOCITY_MIN"],
                self.can_com_param["CAN_COM_VELOCITY_MAX"],
                12,
            )
            pos_kp = self.__float_to_int(
                pos_kp,
                self.can_com_param["CAN_COM_POS_KP_MIN"],
                self.can_com_param["CAN_COM_POS_KP_MAX"],
                12,
            )
            pos_kd = self.__float_to_int(
                pos_kd,
                self.can_com_param["CAN_COM_POS_KD_MIN"],
                self.can_com_param["CAN_COM_POS_KD_MAX"],
                12,
            )
            torque = self.__float_to_int(
                torque,
                self.can_com_param["CAN_COM_TORQUE_MIN"],
                self.can_com_param["CAN_COM_TORQUE_MAX"],
                12,
            )

            tx_msg = can.Message(
                arbitration_id=dev_id,
                is_extended_id=False,
                dlc=0x08,
                data=[
                    (pos >> 8) & 0xFF,
                    pos & 0xFF,
                    (vel & 0xFF0) >> 4,
                    (vel & 0x0F) << 4 | (pos_kp & 0xF00) >> 8,
                    pos_kp & 0xFF,
                    (pos_kd & 0xFF0) >> 4,
                    (pos_kd & 0x0F) << 4 | (torque & 0xF00) >> 8,
                    torque & 0xFF,
                ],
            )

            self.dev.send(tx_msg, self.dev_info["notimeout"])

        except Exception as exc:
            # 若需要记录或处理异常，可在此处进行
            pass


    def run_vel_mode(self, dev_id, vel, vel_kp, vel_kd, vel_ki):
        try:
            vel = self.__float_to_int(
                vel,
                self.can_com_param["CAN_COM_VELOCITY_MIN"],
                self.can_com_param["CAN_COM_VELOCITY_MAX"],
                16,
            )
            vel_kp = self.__float_to_int(
                vel_kp,
                self.can_com_param["CAN_COM_VEL_KP_MIN"],
                self.can_com_param["CAN_COM_VEL_KP_MAX"],
                12,
            )
            vel_kd = self.__float_to_int(
                vel_kd,
                self.can_com_param["CAN_COM_VEL_KD_MIN"],
                self.can_com_param["CAN_COM_VEL_KD_MAX"],
                12,
            )
            vel_ki = self.__float_to_int(
                vel_ki,
                self.can_com_param["CAN_COM_VEL_KI_MIN"],
                self.can_com_param["CAN_COM_VEL_KI_MAX"],
                12,
            )

            tx_msg = can.Message(
                arbitration_id=dev_id,
                is_extended_id=False,
                dlc=0x08,
                data=[
                    (vel >> 8) & 0xFF,
                    vel & 0xFF,
                    (vel_kp & 0xFF0) >> 4,
                    (vel_kp & 0x0F) << 4 | (vel_kd & 0xF00) >> 8,
                    vel_kd & 0xFF,
                    (vel_ki & 0xFF0) >> 4,
                    (vel_ki & 0x0F) << 4 & 0xF0,
                    0xAC,
                ],
            )
            self.dev.send(tx_msg, self.dev_info["timeout"])

            rx_msg = self.dev.recv(self.dev_info["timeout"])
            if rx_msg is not None:
                if rx_msg.arbitration_id == dev_id:
                    return self.__return_motor_state(rx_msg.data)
                else:
                    return rx_msg
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    def run_torque_mode(self, dev_id, torque):
        try:
            torque = self.__float_to_int(
                torque,
                self.can_com_param["CAN_COM_TORQUE_MIN"],
                self.can_com_param["CAN_COM_TORQUE_MAX"],
                16,
            )
            tx_msg = can.Message(
                arbitration_id=dev_id,
                is_extended_id=False,
                dlc=0x08,
                data=[
                    (torque >> 8) & 0xFF,
                    torque & 0xFF,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0xAB,
                ],
            )
            self.dev.send(tx_msg, self.dev_info["timeout"])

            rx_msg = self.dev.recv(self.dev_info["timeout"])
            if rx_msg is not None:
                if rx_msg.arbitration_id == dev_id:
                    return self.__return_motor_state(rx_msg.data)
                else:
                    return rx_msg
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg

    # def run_torque4_mode(self,  torque1, torque2, torque3, torque4):
    #     try:
    #         torque1 = self.__float_to_int(
    #             torque1,
    #             self.can_com_param["CAN_COM_TORQUE_MIN"],
    #             self.can_com_param["CAN_COM_TORQUE_MAX"],
    #             16,
    #         )
    #         torque2 = self.__float_to_int(
    #             torque2,
    #             self.can_com_param["CAN_COM_TORQUE_MIN"],
    #             self.can_com_param["CAN_COM_TORQUE_MAX"],
    #             16,
    #         )
    #         torque3 = self.__float_to_int(
    #             torque3,
    #             self.can_com_param["CAN_COM_TORQUE_MIN"],
    #             self.can_com_param["CAN_COM_TORQUE_MAX"],
    #             16,
    #         )
    #         torque4 = self.__float_to_int(
    #             torque4,
    #             self.can_com_param["CAN_COM_TORQUE_MIN"],
    #             self.can_com_param["CAN_COM_TORQUE_MAX"],
    #             16,
    #         )
    #         tx_msg = can.Message(
    #             arbitration_id=0x100,
    #             is_extended_id=False,
    #             dlc=0x08,
    #             data=[
    #                 (torque1 >> 8) & 0xFF,
    #                 torque1 & 0xFF,
    #                 (torque2 >> 8) & 0xFF,
    #                 torque2 & 0xFF,
    #                 (torque3 >> 8) & 0xFF,
    #                 torque3 & 0xFF,
    #                 (torque4 >> 8) & 0xFF,
    #                 torque4 & 0xFF,
    #             ],
    #         )
    #         self.dev.send(tx_msg, self.dev_info["timeout"])

    #         rx_msg = self.dev.recv(self.dev_info["timeout"])
    #         if rx_msg is not None:
    #             if rx_msg.arbitration_id == 0x100:
    #                 return self.__return_motor_state(rx_msg.data)
    #             else:
    #                 return rx_msg
    #         else:
    #             return False

    #     except Exception as exc:
    #         exc_msg = str(exc)
    #     return exc_msg


if __name__ == '__main__':
    from SimpleSDK import RUIWOTools
    ruiwo = RUIWOTools()
    ruiwo.open_canbus()
    id = 0x06
    state = ruiwo.enter_motor_state(id)
    print(state[1])
    ruiwo.run_ptm_mode(id, state[1], 0.0, 25, 8, 0.0)
