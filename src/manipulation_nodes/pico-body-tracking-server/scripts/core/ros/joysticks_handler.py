import enum
import time
from typing import Callable, Set, Tuple
from kuavo_msgs.msg import JoySticks

class ButtonState(enum.Enum):
    IDLE = "IDLE"                   # 空闲
    PRESSED = "PRESSED"             # 点按
    PRESSING = "PRESSING"           # 按压中但未达到触发长按
    LONG_PRESSED = "LONG_PRESSED"   # 长按

class JoySticksHandler:
    def __init__(self) -> None:
        self.btn_names = ['X', 'Y', 'A', 'B']
        self.btn_pressed_time = {btn: 0.0 for btn in self.btn_names}
        self.btn_last_seq = {btn: 0 for btn in self.btn_names}
        self.btn_reported_state = {btn: None for btn in self.btn_names}  # Track last reported state
        self.LONG_PRESS_THRESHOLD = 0.50  # 长按阈值秒
        
        # Trigger相关设置
        self.trigger_names = ['LT', 'RT', 'LG', 'RG']  # LT=Left Trigger, RT=Right Trigger, LG=Left Grip, RG=Right Grip
        self.trigger_threshold = 0.5  # 触发器按下阈值
        
        self.callbacks = {}

    def update(self, joy: JoySticks)->Tuple[Set[str], bool]:
        """
        更新手柄状态并返回当前按键组合和回调触发状态
        
        Args:
            joy: JoySticks消息对象，包含当前手柄的按键和触发器状态
            
        Returns:
            Tuple[Set[str], bool]: 
                - Set[str]: 当前激活的按键组合集合，格式为"按键名_状态"
                - bool: 是否触发了任何回调函数
        """
        key_combination = self._handle_buttons(joy)
        
        # 处理trigger状态
        trigger_combination = self._handle_triggers(joy)
        key_combination.update(trigger_combination)

        # if key_combination:
        #     color_map = {
        #         'IDLE': '\033[90m',             # 灰色
        #         'PRESSED': '\033[92m',          # 绿色
        #         'PRESSING': '\033[93m',         # 黄色
        #         'LONG_PRESSED': '\033[91m'      # 红色
        #     }
        #     colored_keys = []
        #     for key in key_combination:
        #         btn_name, state = key.split('_', 1)
        #         color = color_map.get(state, '\033[0m')
        #         colored_keys.append(f"{color}{key}\033[0m")
        #     print(f"Key combination: {', '.join(colored_keys)}")

        callback_triggered = self._handle_key_combination_callback(key_combination, joy)

        return key_combination, callback_triggered

    def add_callback(self, name: str, key_combination: Set[str], callback: Callable[..., None]) -> None:
        if name in self.callbacks:
            print(f"\033[91mname: {name} already exists\033[0m")
        self.callbacks[name] = (key_combination, callback)

    def remove_callback(self, name: str) -> None:
        if name in self.callbacks:
            del self.callbacks[name]
        else:
            print(f"\033[91mname: {name} not found\033[0m")

    def match_key_combination(self, expected_keys: Set[str], current_keys: Set[str]) -> bool:
        """检查期望的按键组合是否与当前按键组合匹配"""
        return expected_keys.issubset(current_keys)

    def _handle_buttons(self, joy: JoySticks) -> Set[str]:
        btn_seq = {
            'X': joy.left_first_button_pressed,
            'Y': joy.left_second_button_pressed,
            'A': joy.right_first_button_pressed,
            'B': joy.right_second_button_pressed
        }

        current_time = time.time()
        key_combination = set()

        for btn in self.btn_names:
            is_pressed = btn_seq[btn] == 1
            was_pressed = self.btn_last_seq[btn] == 1

            if is_pressed and not was_pressed:
                # Rising edge
                self.btn_pressed_time[btn] = current_time
                self.btn_reported_state[btn] = None

            elif not is_pressed and was_pressed:
                # Falling edge
                duration = current_time - self.btn_pressed_time[btn]
                if self.btn_reported_state[btn] is None:
                    if duration >= self.LONG_PRESS_THRESHOLD:
                        key_combination.add(f"{btn}_{ButtonState.LONG_PRESSED.value}")
                    else:
                        key_combination.add(f"{btn}_{ButtonState.PRESSED.value}")
                self.btn_pressed_time[btn] = 0.0
                self.btn_reported_state[btn] = None
                self.btn_last_seq[btn] = btn_seq[btn]
                continue

            elif is_pressed:
                # Holding state
                duration = current_time - self.btn_pressed_time[btn]
                if duration < self.LONG_PRESS_THRESHOLD:
                    key_combination.add(f"{btn}_{ButtonState.PRESSING.value}")
                elif duration >= self.LONG_PRESS_THRESHOLD and self.btn_reported_state[btn] is None:
                    key_combination.add(f"{btn}_{ButtonState.LONG_PRESSED.value}")
                    self.btn_reported_state[btn] = ButtonState.LONG_PRESSED

            else:
                key_combination.add(f"{btn}_{ButtonState.IDLE.value}")

            self.btn_last_seq[btn] = btn_seq[btn]

        return key_combination

    def _handle_triggers(self, joy: JoySticks) -> Set[str]:
        """处理trigger按下状态"""
        trigger_combination = set()
        
        # 获取当前trigger值
        trigger_values = {
            'LT': joy.left_trigger,     # LT = Left Trigger (左触发器)
            'RT': joy.right_trigger,    # RT = Right Trigger (右触发器)
            'LG': joy.left_grip,        # LG = Left Grip (左握把)
            'RG': joy.right_grip        # RG = Right Grip (右握把)
        }
        
        for trigger_name in self.trigger_names:
            current_value = trigger_values[trigger_name]
            if current_value >= self.trigger_threshold:
                trigger_combination.add(f"{trigger_name}_PRESSED")
            else:
                trigger_combination.add(f"{trigger_name}_IDLE")
            
        return trigger_combination

    def _handle_key_combination_callback(self, key_combination: Set[str], joy: JoySticks) -> bool:
        # 使用最长匹配：找到所有匹配的回调，选择按键组合最长的那个
        matched_callbacks = []
        
        for name, (expected_keys, callback) in self.callbacks.items():
            if self.match_key_combination(expected_keys, key_combination):
                matched_callbacks.append((len(expected_keys), name, expected_keys, callback))
        
        if matched_callbacks:
            # 按按键组合长度降序排序，选择最长的匹配
            matched_callbacks.sort(key=lambda x: x[0], reverse=True)
            _, best_name, best_expected_keys, best_callback = matched_callbacks[0]
            
            # 如果有多个相同长度的匹配，选择第一个（避免歧义）
            if not (best_name == "teleop_unlock" or best_name == "teleop_lock"):
                print(f"匹配的组合键: {best_expected_keys}, 当前按键: {key_combination}, 回调名称: {best_name}")
            best_callback(key_combination, joy)
            return True
        
        return False

if __name__ == "__main__":
    import rospy
    rospy.init_node("joy_button_handler")

    joy_button_handler = JoySticksHandler()

    def sub_callback(joy: JoySticks) -> None:
        joy_button_handler.update(joy)

    def callback1(keys: Set[str], joy: JoySticks) -> None:
        print(f"\033[91mcallback1: {keys}\033[0m")

    def callback2(keys: Set[str], joy: JoySticks) -> None:
        print(f"\033[91mcallback2: {keys}\033[0m")

    rospy.Subscriber("/pico/joy", JoySticks, sub_callback)
    joy_button_handler.add_callback("callback1", {"X_PRESSED", "Y_PRESSED"}, callback1)
    joy_button_handler.add_callback("callback2", {"Y_PRESSED"}, callback2)
    rospy.spin()