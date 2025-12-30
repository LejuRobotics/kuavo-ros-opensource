import json
from transitions import Machine
from utils.utils import read_json_file
import os
import rospkg
# 获取包路径
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('h12pro_controller_node')
config_path = os.path.join(pkg_path, "robot_state", "robot_state.json")

robot_state_config = read_json_file(config_path)

robot_type = os.getenv("ROBOT_TYPE", "ocs2")
states = robot_state_config["states"][robot_type]
transitions = robot_state_config["transitions"][robot_type]
if robot_type == "ocs2":
    import robot_state.ocs2_before_callback as before_callback
else:
    import robot_state.before_callback as before_callback

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
            if callback:
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
