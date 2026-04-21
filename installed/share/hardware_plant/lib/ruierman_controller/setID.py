from SimpleSDK import WHJ30Tools
import time

if __name__ == "__main__":
    # 创建WHJ30Tools对象实例
    whj30 = WHJ30Tools()

    # 开启CAN总线通讯
    whj30.open_canbus()
    newid = 0x02
    # 跳过IAP更新操作，传入关节ID id
    whj30.skip_iap_update(0x03)


    # 失能关节，传入关节ID id
    whj30.disable_joint(0x03)

    # 使能关节，传入关节ID id
    whj30.enable_joint(0x03)
    time.sleep(4)

    whj30.set_joint_id(0x03,newid)
    whj30.save_joint_param(0x03)
    # whj30.set_joint_zero_position(id)
    # whj30.save_joint_param(id)
    
    time.sleep(1)


    whj30.close_canbus()
