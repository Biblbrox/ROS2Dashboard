from ros2dashboard.edge.GraphEdge import GraphEdge

class ActionServer(GraphEdge):
    def __init__(self, node_name, action_server_name):
        super().__init__(node_name)
        self.action_server_name = action_server_name