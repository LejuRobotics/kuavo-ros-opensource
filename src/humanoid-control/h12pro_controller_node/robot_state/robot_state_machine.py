import json
from transitions import Machine
from utils.utils import read_json_file
import os
import rospkg
import rospy
# 获取包路径
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('h12pro_controller_node')
config_path = os.path.join(pkg_path, "robot_state", "robot_state.json")

robot_state_config = read_json_file(config_path)

robot_type = os.getenv("ROBOT_TYPE", "ocs2")
kuavo_control_scheme = os.getenv("KUAVO_CONTROL_SCHEME", "ocs2")
if kuavo_control_scheme == "ocs2":
    states = robot_state_config["states"][robot_type]
    transitions = robot_state_config["transitions"][robot_type]
    if robot_type == "ocs2":
        import robot_state.ocs2_before_callback as before_callback
    else:
        import robot_state.before_callback as before_callback
elif kuavo_control_scheme == "rl":
    states = robot_state_config["states"][kuavo_control_scheme]
    transitions = robot_state_config["transitions"][kuavo_control_scheme]
    import robot_state.rl_before_callback as before_callback
elif kuavo_control_scheme == "multi":
    states = robot_state_config["states"][kuavo_control_scheme]
    transitions = robot_state_config["transitions"][kuavo_control_scheme]
    import robot_state.multi_before_callback as before_callback
else:
    raise ValueError(f"Invalid kuavo_control_scheme: {kuavo_control_scheme}")

class RobotStateMachine(object):
    def __init__(self, **kwargs):
        self.machine = Machine(model=self, **kwargs)
        self.setup_transitions()

    def setup_transitions(self):
        """
        Setup transitions, register before callback from external module
        """
        for transition in transitions:
            source = transition["source"]
            dest = transition["dest"]
            trigger = transition["trigger"]
            callback = getattr(before_callback, transition["before"], None)
            
            # 支持 conditions 条件检查
            conditions = []
            if "conditions" in transition:
                condition_names = transition["conditions"]
                if isinstance(condition_names, str):
                    condition_names = [condition_names]
                for condition_name in condition_names:
                    condition_func = getattr(before_callback, condition_name, None)
                    if condition_func:
                        conditions.append(condition_func)
                    else:
                        rospy.logwarn(f"Condition function '{condition_name}' not found in before_callback module")
            
            if callback:
                if conditions:
                    self.machine.add_transition(
                        trigger=trigger,
                        source=source,
                        dest=dest,
                        before=callback,
                        conditions=conditions,
                    )
                else:
                    self.machine.add_transition(
                        trigger=trigger,
                        source=source,
                        dest=dest,
                        before=callback,
                    )
    
    def update_customize_config(self):
        if robot_type == "ocs2":
            before_callback.update_h12_customize_config()


robot_state_machine = RobotStateMachine(
    states=states,
    initial="initial",
    send_event=True,
    auto_transitions=False,
)
