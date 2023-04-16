from ros2dashboard.edge.GraphEdge import GraphEdge

class Subscriber(GraphEdge):
    def __init__(self, node_name, subscriber_name):
        super().__init__(node_name)
        self.subscriber_name = subscriber_name