from ros2dashboard.edge.GraphEdge import GraphEdge

class ActionClient(GraphEdge):
    def __init__(self, node_name, action_client_name):
        super().__init__(node_name)
        self.action_client_name = action_client_name