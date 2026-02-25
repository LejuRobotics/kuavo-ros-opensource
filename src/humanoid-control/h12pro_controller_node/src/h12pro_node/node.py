# #!/usr/bin/env python3
# import time
import rospy
import enum
import time

from h12pro_controller_node.msg import (
    h12proRemoteControllerChannel,
    walkCommand,
)
from transitions.core import MachineError
from robot_state.robot_state_machine import robot_state_machine
from utils.utils import read_json_file, map_value

RELEASE = "RELEASE"
PRESS = "PRESS"
LONG_PRESS = "LONG_PRESS"


class KEY_TYPE(enum.Enum):
    BUTTON = "button"
    SWITCH = "switch"
    JOYSTICK = "joystick"


class H12PROControllerNode:
    def __init__(self, *args, **kwargs) -> None:
        self.load_configuration()
        self.robot_state_machine = robot_state_machine
        self.long_press_threshold = 1.0
        self.key_timestamp = {}
        self.joystick_values = {"x": 0, "y": 0, "z": 0}
        self.setup_subscribers_and_publishers()
        self.real = kwargs.get("real", False)

    def load_configuration(self) -> None:
        """
        Load configuration from the JSON file.

        channel_to_key_name: Mapping of channel number to key name.

        channel_to_key_state: Mapping of channel number to key state.

        robot_state_transition_keycombination: Mapping of robot state to key combination for state transition.

        emergency_stop_key_combination: Key combination for emergency stop.

        joystick_range: Range of joystick values.

        joystick_to_corresponding_axis: Mapping of joystick to corresponding axis.

        """
        self.h12pro_remote_controller_config = read_json_file(
            "src/h12pro_node/h12pro_remote_controller.json"
        )

        self.channel_to_key_name = self.h12pro_remote_controller_config[
            "channel_to_key_name"
        ]
        self.channel_to_key_state = self.h12pro_remote_controller_config[
            "channel_to_key_state"
        ]
        self.robot_state_transition_keycombination = (
            self.h12pro_remote_controller_config[
                "robot_state_transition_keycombination"
            ]
        )
        self.emergency_stop_key_combination = set(
            self.h12pro_remote_controller_config["emergency_stop_key_combination"]
        )
        self.joystick_range = self.h12pro_remote_controller_config["joystick_range"]
        self.joystick_to_corresponding_axis = self.h12pro_remote_controller_config[
            "joystick_to_corresponding_axis"
        ]

    def setup_subscribers_and_publishers(self) -> None:
        """
        Setup subscribers and publishers for the node.
        """

        self.walkCommand_pub = rospy.Publisher(
            "/walkCommand", walkCommand, queue_size=10
        )
        self.h12pro_controller_channels_sub = rospy.Subscriber(
            "/h12pro_channel",
            h12proRemoteControllerChannel,
            self.h12pro_controller_channels_callback,
            queue_size=1,
        )
        self.pub_joystick_timer = rospy.Timer(
            rospy.Duration(0.1), self.pub_joystick_callback
        )

    def h12pro_controller_channels_callback(self, msg):
        key_combination = set()
        channels = msg.channels

        for index, channel in enumerate(channels):
            channel_num = index + 1
            key = self.channel_to_key_name[str(channel_num)]["name"]
            type = self.channel_to_key_name[str(channel_num)]["type"]

            if type == KEY_TYPE.BUTTON.value:
                state = self.handle_button(key, channel)
                if state:
                    key_combination.add(state)
            elif type == KEY_TYPE.SWITCH.value:
                state = self.handle_switch(key, channel)
                if state:
                    key_combination.add(state)
            elif type == KEY_TYPE.JOYSTICK.value:
                self.handle_joystick(key, channel)

        robot_state = self.robot_state_machine.state
        if self.emergency_stop_key_combination.issubset(key_combination):
            try:
                getattr(self.robot_state_machine, "emergency_stop")(source=robot_state)
            except MachineError as e:
                rospy.logerr(e)
            finally:
                return

        triggers = self.robot_state_machine.machine.get_triggers(robot_state)
        for trigger in triggers:
            trigger_key_combination = set(
                self.robot_state_transition_keycombination[robot_state][trigger]
            )
            if trigger_key_combination.issubset(key_combination):
                try:
                    kwargs = {
                        "trigger": trigger,
                        "source": robot_state,
                    }
                    if trigger == "calibrate" or "start" or "set_zero":
                        kwargs["real"] = self.real
                    getattr(self.robot_state_machine, trigger)(**kwargs)
                except Exception as e:
                    rospy.logerr(e)

    def handle_button(self, key, channel) -> str:
        """
        Handles button press logic for different states and timing conditions.

        Args:
            key (str): The key identifier.
            channel (int): The channel data from which the state is derived.

        Returns:
            str: The state of the button (PRESS, LONG_PRESS, or None).
        """
        try:
            state = self.channel_to_key_state[key][str(channel)]
        except Exception as e:
            print(e)
            return None
        current_time = time.time()

        if PRESS in state:
            if key not in self.key_timestamp:
                self.key_timestamp[key] = current_time
                return None  # Early exit if just pressed, no state yet.
            else:
                duration = current_time - self.key_timestamp[key]
                if duration > self.long_press_threshold:
                    return f"{key}_LONG_PRESS"

        elif RELEASE in state:
            if key in self.key_timestamp:
                duration = current_time - self.key_timestamp[key]
                del self.key_timestamp[key]  # Clean up timestamp after release.
                if duration < self.long_press_threshold:
                    return f"{key}_PRESS"
                else:
                    return f"{key}_LONG_PRESS"

        return None  # Return None if none of the conditions are met.

    def handle_switch(self, key, channel) -> str:
        """Handles switch press logic for different states.

        Args:
            key (str): The key identifier.
            channel (int): The channel data from which the state is derived.

        Returns:
            str: The state of the switch (LEFT, RIGHT, or MIDDLE).
        """
        try:
            state = self.channel_to_key_state[key][str(channel)]
        except Exception as e:
            print(e)
            return None
        return state

    def handle_joystick(self, key, channel) -> None:
        """Handles joystick values, maps them to corresponding axis and publishes them to the walkCommand topic.

        Args:
            key (str): The key identifier.
            channel (int): The channel data from which the joystick values are derived.
        """
        if key not in self.joystick_to_corresponding_axis:
            return
        axis = self.joystick_to_corresponding_axis[key]["axis"]
        range = self.joystick_to_corresponding_axis[key]["range"]
        value = map_value(
            channel,
            self.joystick_range["min"],
            self.joystick_range["max"],
            range["min"],
            range["max"],
        )
        if axis == "y":
            value = -value
        self.joystick_values[axis] = value
        return

    def pub_joystick_callback(self, event) -> None:
        """Publishes joystick values to the walkCommand topic.

        Args:
            event (rospy.TimerEvent): The event object.

        """
        if self.robot_state_machine.state == "walk":
            command = walkCommand()
            command.mode = 1
            command.values = [
                self.joystick_values["x"],
                self.joystick_values["y"],
                self.joystick_values["w"],
            ]
            self.walkCommand_pub.publish(command)
