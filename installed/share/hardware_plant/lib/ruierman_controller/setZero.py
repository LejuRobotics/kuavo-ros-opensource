# 导入霸码科技USB转CANFD模块Python开发工具包中的WHJ30Tools模块和time模块
from SimpleSDK import WHJ30Tools
import time

if __name__ == "__main__":
    # 创建WHJ30Tools对象实例
    whj30 = WHJ30Tools()

    # 开启CAN总线通讯
    whj30.open_canbus()
    joint_address_list = [0x01, 0x02]
    
    for address in joint_address_list:
        whj30.skip_iap_update(address)
        whj30.disable_joint(address)
        whj30.enable_joint(address)
        print(address)
    time.sleep(4)

    for address in joint_address_list:
        whj30.set_joint_zero_position(address)
        whj30.save_joint_param(address)
        print(whj30.get_joint_state(id))

    # 关闭CAN总线通讯
    whj30.close_canbus()
