import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from .performance_monitor import performance_monitor

##TODO
class DropDetection(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super(DropDetection, self).__init__(name)

    @performance_monitor(method_name="initialise")
    def initialise(self):
        print("hello, DropDetection")
    @performance_monitor(method_name="update")
    def update(self):
        return Status.SUCCESS