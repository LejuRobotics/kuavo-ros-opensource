from kuavo_humanoid_sdk import KuavoSDK, DexterousHand
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorState

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    import time
    dex_hand = DexterousHand() # init dex hand
    
    print(f"Joint names: {dex_hand.joint_names()}")
    print(f"Joint count: {dex_hand.joint_count()}")
    
    # Loop for 1 minute, getting the state every 1 second
    start_time = time.time()
    end_time = start_time + 20  # 1 minute = 20 seconds

    print("Getting dexhand state every 1 second for 1 minute...")

    try:
        while time.time() < end_time:
            state = dex_hand.get_state()
            print(f"Dexhand position:{dex_hand.get_position()}")
            print(f"Dexhand velocity:{dex_hand.get_velocity()}")
            print(f"Dexhand effort:{dex_hand.get_effort()}")
            print("-" * 40)
            
            # Wait for 1 second before the next state check
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nOperation cancelled by user.")
    
    print("Finished monitoring dexhand state for 1 minute.")
    
if __name__ == "__main__":
    main()
