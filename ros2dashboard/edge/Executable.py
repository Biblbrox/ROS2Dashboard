from ros2dashboard.devices.GenericNode import GenericNode
from ros2dashboard.edge.GraphEdge import GraphEdge

class Executable(GraphEdge):
    """ Each executable belongs to speciefic package and may store multiple nodes. 
    """
    def __init__(self, executable_name: str, package_name: str, node_names: list[str] = []):
        self.name = executable_name
        self.nodes = node_names
        self.package_name = package_name

