from ros2dashboard.edge.GraphEdge import GraphEdge

class Client(GraphEdge):
    def __init__(self, node_name, client_name):
        super().__init__(node_name)
        self.client_name = client_name