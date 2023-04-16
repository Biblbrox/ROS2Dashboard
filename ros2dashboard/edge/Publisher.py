from ros2dashboard.edge.GraphEdge import GraphEdge


class Publisher(GraphEdge):
    def __init__(self, node_name: str, publisher_name: str):
        super().__init__(node_name)
        self.publisher_name = publisher_name
