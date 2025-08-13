from kuavo_humanoid_sdk import KuavoSDK, DexterousHand
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorState

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    import time
    dex_hand = DexterousHand() # init dex hand

    dex_hand.control_left([5, 5, 95, 95, 95, 95]) # 6 dof. 'thumbs-up' gesture.
    dex_hand.control_right([5, 5, 95, 95, 95, 5]) # 6 dof. '666' gesture.
    time.sleep(1.0)

    # open/reset/release
    dex_hand.open() # both.
    time.sleep(1.0)

    # get gesture names
    gestures = dex_hand.get_gesture_names()
    if gestures is None:
        print("Get gesture names failed!")
        exit(1)
    else:
        for gesture in gestures:
            print("Supported gesture: ", gesture)
    
    # make gesture
    dex_hand.make_gesture(l_gesture_name="thumbs-up", r_gesture_name="fist")
if __name__ == "__main__":
    main()
