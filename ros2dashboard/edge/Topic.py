from ros2dashboard.edge.GraphEdge import GraphEdge


class Topic(GraphEdge):
    def __init__(self, node_name, topic_name):
        super().__init__(node_name)
        self.topic_name = topic_name
