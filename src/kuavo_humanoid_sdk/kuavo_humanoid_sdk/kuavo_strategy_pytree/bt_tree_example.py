from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees import logging as log_tree


class Action(Behaviour):
    def __init__(self, name):
        super(Action, self).__init__(name)

    def setup(self):
        self.logger.debug(f"Action::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"Action::initialise {self.name}")

    def update(self):
        self.logger.debug(f"Action::update {self.name}")
        sleep(1)
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")


class Condition(Behaviour):
    def __init__(self, name):
        super(Condition, self).__init__(name)

    def setup(self):
        self.logger.debug(f"Condition::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"Condition::initialise {self.name}")

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        sleep(1)
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")


def make_bt():
    root = Sequence(name="sequence", memory=True)

    check_battery = Condition("check_battery")
    open_gripper = Action("open_gripper")
    approach_object = Action("approach_object")
    close_gripper = Action("close_gripper")

    root.add_children(
        [
            check_battery,
            open_gripper,
            approach_object,
            close_gripper
        ]
    )

    return root


if __name__ == "__main__":
    log_tree.level = log_tree.Level.DEBUG
    tree = make_bt()
    for i in range(5):
        print(f"\n--- Tick {i} ---")
        # behaviour_tree.tick()
        tree.tick_once()
