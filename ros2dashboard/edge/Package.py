from ros2dashboard.devices.Ros2Node import Ros2Node
from ros2dashboard.edge.GraphEdge import GraphEdge

class Package(GraphEdge):
    def __init__(self, package_name: str, nodes: list[Ros2Node] = []):
        self.nodes = nodes
        self.name = package_name

