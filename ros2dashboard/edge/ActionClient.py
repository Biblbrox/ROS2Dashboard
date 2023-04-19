from ros2dashboard.edge.GraphEdge import GraphEdge
import hashlib


class ActionClient(GraphEdge):
    def __init__(self, node_name, action_client_name):
        super().__init__(node_name)
        self.action_client_name = action_client_name

    def unique_key(self) -> str:
        return hashlib.sha256(bytes(self.node_name + self.action_client_name, 'utf-8')).hexdigest()
