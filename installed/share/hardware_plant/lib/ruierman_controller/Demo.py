# 导入霸码科技USB转CANFD模块Python开发工具包中的WHJ30Tools模块和time模块
from SimpleSDK import WHJ30Tools
import time

if __name__ == "__main__":
    # 创建WHJ30Tools对象实例
    whj30 = WHJ30Tools()

    # 开启CAN总线通讯
    whj30.open_canbus()
    id = id
    # 跳过IAP更新操作，传入关节ID id
    whj30.skip_iap_update(id)

    # 等待5秒
    time.sleep(2)

    # 失能关节，传入关节ID id
    whj30.disable_joint(id)

    # 使能关节，传入关节ID id
    whj30.enable_joint(id)
    # whj30.set_joint_id(id,0x01)
    # whj30.save_joint_param(id)
    whj30.set_joint_zero_position(id)
    whj30.save_joint_param(id)
    # 等待5秒
    time.sleep(1)

    # # 获取关节状态，传入关节ID id
    # whj30.get_joint_state(id)
    print(whj30.get_joint_state(id))

    # # 设置关节工作模式为位置模式，传入关节ID id 和操作模式 id（位置模式）
    # whj30.set_joint_operate_mode(id, id)

    # # 以10°的步长进行10次循环设置关节位置
    for i in range(10):
        # 设置关节位置，传入关节ID id 和目标位置 100 + i * 10 度
        whj30.set_joint_position(id, 16 -  i * 2)
        
        # 等待3秒
        time.sleep(0.1)

    # # 设置关节最小位置为 200 度，传入关节ID id 和最小位置 200 度
    # whj30.set_joint_min_position(id, 200)

    # # 设置关节最大位置为 360 度，传入关节ID id 和最大位置 360 度
    # whj30.set_joint_max_position(id, 360)

    # # 获取关节状态，传入关节ID id
    # whj30.get_joint_state(id)

    # # 重置关节最小位置为 0 度，传入关节ID id 和最小位置 0 度
    # whj30.set_joint_min_position(id, 0)

    # # # 设置关节最大位置为 360 度，传入关节ID id 和最大位置 360 度
    # whj30.set_joint_max_position(id, 360)

    # # # 清除关节错误，传入关节ID id
    # whj30.clear_joint_error(id)

    # # # 获取关节状态，传入关节ID id
    # whj30.get_joint_state(id)

    # # 使能关节，传入关节ID id
    # whj30.enable_joint(id)

    # # 等待5秒
    # time.sleep(5)

    # 关闭CAN总线通讯
    whj30.close_canbus()
