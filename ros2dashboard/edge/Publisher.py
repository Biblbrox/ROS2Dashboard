import hashlib

from ros2dashboard.edge.GraphEdge import GraphEdge

class Publisher(GraphEdge):
    def __init__(self, node_name: str, topic_name: str, topic_type: str, port=0):
        super().__init__(node_name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.port = port

    
    def unique_key(self) -> str:
        return hashlib.sha256(bytes(self.node_name + self.topic_name + self.topic_type, 'utf-8')).hexdigest()
