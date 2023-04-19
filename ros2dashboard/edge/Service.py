import hashlib

from ros2dashboard.edge.GraphEdge import GraphEdge

class Service(GraphEdge):
    def __init__(self, node_name, service_name):
        super().__init__(node_name)
        self.service_name = service_name

    def unique_key(self) -> str:
        return hashlib.sha256(bytes(self.node_name + self.service_name, 'utf-8')).hexdigest()