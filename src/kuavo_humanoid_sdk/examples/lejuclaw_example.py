from kuavo_humanoid_sdk import KuavoSDK, LejuClaw
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorState

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    claw = LejuClaw()
    
    # close claw
    if claw.close():
        if claw.wait_for_finish(timeout=2.0): # Wait for the claw motion to finish with a 2 second timeout
            print("Claw motion finished successfully")
            
    # open
    if claw.open():
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Open claw motion finished successfully")

    # control_left
    if claw.control_left(target_positions=[50]):
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Left claw motion finished successfully")

    # control_right
    if claw.control_right(target_positions=[80]):
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Right claw motion finished successfully")

    # control_right
    if claw.control(target_positions=[20, 100]):
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Both claw motion finished successfully")

    claw.open()   
    # !!! WARNING: !!!
    # This request may be dropped because the claw may still be executing the previous motion.
    claw.control(target_positions=[10, 10]) 
if __name__ == "__main__":
    main()
