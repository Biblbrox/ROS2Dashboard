from ros2dashboard.edge.GraphEdge import GraphEdge

class Service(GraphEdge):
    def __init__(self, node_name, service_name):
        super().__init__(node_name)
        self.service_name = service_name