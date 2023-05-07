from ros2dashboard.devices.GenericNode import GenericNode
from ros2dashboard.edge.GraphEdge import GraphEdge

class Package(GraphEdge):
    """ Each package may contain multiple executables
    """
    def __init__(self, package_name: str, executable_names: list[str] = []):
        self.executable_names = executable_names
        self.name = package_name

