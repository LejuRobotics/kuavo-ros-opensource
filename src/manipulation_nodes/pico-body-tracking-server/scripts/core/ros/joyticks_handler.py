import enum
import time
from typing import Callable, Set
from kuavo_msgs.msg import JoySticks

class ButtonState(enum.Enum):
    IDLE = "IDLE"               # 空闲
    PRESSED = "PRESSED"         # 点按
    PRESSING = "PRESSING"       # 按压中但未达到触发长按
    LONG_PRESSED = "LONG_PRESSED"  # 长按

class JoySticksHandler:
    def __init__(self) -> None:
        self.btn_names = ['X', 'Y','A', 'B']
        self.btn_pressed_time = {}
        for btn in self.btn_names:
            self.btn_pressed_time[btn] = 0.0
        self.LONG_PRESS_THRESHOLD = 0.8 # 长按阈值秒
        self.callbacks = {}
    def update(self, joy:JoySticks):
        # 更新按键序列
        key_combination = self._handle_button(joy)

        # 打印 key_combination 需要区分不同颜色
        # if key_combination:
        #     color_map = {
        #         'IDLE': '\033[90m',      # 灰色
        #         'PRESSED': '\033[92m',   # 绿色
        #         'PRESSING': '\033[93m',  # 黄色
        #         'LONG_PRESSED': '\033[91m'  # 红色
        #     }
        #     colored_keys = []
        #     for key in key_combination:
        #         btn_name, state = key.split('_', 1)
        #         color = color_map.get(state, '\033[0m')
        #         colored_keys.append(f"{color}{key}\033[0m")
        #     print(f"Key combination: {', '.join(colored_keys)}")
        # 处理组合键回调函数
        self._handle_key_combination_callback(key_combination, joy)

    def add_callback(self, name:str, key_combination: Set[str], callback:Callable[..., None])->None:
        """添加组合键回调函数。

        Args:
            key_combination: 组合键集合，例如 XY --> {'X_PRESSED', 'Y_PRESSED', 'A_IDLE', 'B_IDLE'}
            callback: 回调函数，当组合键被触发时调用
        """
        if name in self.callbacks:
            print(f"\033[91mname: {name} already exists\033[0m")
        self.callbacks[name] = (key_combination, callback)
    
    def remove_callback(self, name:str)->None:
        """移除组合键回调函数。

        Args:
            key_combination: 要移除的组合键集合，例如 {'X', 'Y'}
        """
        if name in self.callbacks:
            del self.callbacks[name]
        else:
            print(f"\033[91mname: {name} not found\033[0m")

    def _handle_button(self,joy:JoySticks)->Set[str]:
        btn_seq = {} 
        btn_seq['X'] = joy.left_first_button_pressed
        btn_seq['Y'] = joy.left_second_button_pressed
        btn_seq['A'] = joy.right_first_button_pressed
        btn_seq['B'] = joy.right_second_button_pressed
        
        current_time = time.time()
        btn_state = {}    
        for btn in self.btn_names:
            if btn_seq[btn] == 0:
                btn_state[btn] = self._handle_btn_release(btn)
            else:
                btn_state[btn] = self._handle_btn_press(btn, current_time)

            # 更新按键按下时间
            if btn_seq[btn] == 1:
                if self.btn_pressed_time[btn] == 0.0:
                    self.btn_pressed_time[btn] = current_time
            else:
                self.btn_pressed_time[btn] = 0.0
        
        # 返回组合键
        key_combination = set()
        for btn in self.btn_names:
            if btn_state[btn] == f"{btn}_{ButtonState.LONG_PRESSED.value}" \
                  or btn_state[btn] == f"{btn}_{ButtonState.IDLE.value}" \
                  or btn_state[btn] == f"{btn}_{ButtonState.PRESSED.value}":
                key_combination.add(btn_state[btn])
        
        return key_combination
        
    def _handle_btn_press(self, btn:str, current_time:float)->str:
        if self.btn_pressed_time[btn] == 0.0:
            return f"{btn}_{ButtonState.PRESSING.value}"
        
        pressed_duration = current_time - self.btn_pressed_time[btn]
        if pressed_duration >= self.LONG_PRESS_THRESHOLD:
            # TODO: 是否一直返回 LONG_PRESSED
            return f"{btn}_{ButtonState.LONG_PRESSED.value}"
        else:
            return f"{btn}_{ButtonState.PRESSING.value}"

    def _handle_btn_release(self, btn:str)->str:
        if self.btn_pressed_time[btn] == 0.0:
            return f"{btn}_{ButtonState.IDLE.value}" # 持续的 00000
        else:
            current_time = time.time()
            pressed_duration = current_time - self.btn_pressed_time[btn]
            if pressed_duration < self.LONG_PRESS_THRESHOLD:
                return f"{btn}_{ButtonState.PRESSED.value}" # 010 --> 010 下沿触发
            else:
                # TODO: 是否还需要触发？LONG_PRESSED
                return f"{btn}_{ButtonState.LONG_PRESSED.value}"
    
    def _handle_key_combination_callback(self, 
                                         key_combination: Set[str], 
                                         joy:JoySticks)->None:
        for name, (key_combination_set, callback) in self.callbacks.items():
            if key_combination_set == key_combination: # 完全匹配
                callback(key_combination, joy)
                break

if __name__ == "__main__":
    import rospy
    rospy.init_node("joy_button_handler")

    joy_button_handler = JoySticksHandler()
    def sub_callback(joy:JoySticks)->None:
        joy_button_handler.update(joy)

    def callback1(key_combination: Set[str], joy:JoySticks)->None:
        print(f"\033[91mcallback1: {key_combination}\033[0m")

    def callback2(key_combination: Set[str], joy:JoySticks)->None:
        print(f"\033[91mcallback2: {key_combination}\033[0m")
    sub = rospy.Subscriber("/pico/joy", JoySticks, sub_callback)
    joy_button_handler.add_callback("callback1", set(['X_PRESSED', 'Y_PRESSED']), callback1)
    joy_button_handler.add_callback("callback2", set(['Y_PRESSED']), callback2)
    rospy.spin()