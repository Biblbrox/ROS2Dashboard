from ros2dashboard.edge.GraphEdge import GraphEdge
import hashlib

class ActionServer(GraphEdge):
    def __init__(self, node_name, action_server_name):
        super().__init__(node_name)
        self.action_server_name = action_server_name
        
    def unique_key(self) -> str:
        return hashlib.sha256(bytes(self.node_name + self.action_server_name, 'utf-8')).hexdigest()