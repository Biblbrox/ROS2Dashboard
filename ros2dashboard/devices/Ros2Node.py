from NodeGraphQt import BaseNode

from ros2dashboard.edge.Topic import Topic
from ros2dashboard.edge.Client import Client
from ros2dashboard.edge.Subscriber import Subscriber
from ros2dashboard.edge.Publisher import Publisher
from ros2dashboard.edge.Service import Service
from ros2dashboard.edge.ActionClient import ActionClient
from ros2dashboard.edge.ActionServer import ActionServer


class Ros2Node(BaseNode):

    # unique node identifier domain.
    __identifier__ = 'ros2.node'
    # initial default node name.
    NODE_NAME = 'Ros2Node'

    def __init__(self, node_name="", host="localhost", action_servers=list[ActionServer], action_clients=list[ActionClient], publishers=list[Publisher], subscribers=list[Subscriber], services=list[Service], clients=list[Client]):
        super(Ros2Node, self).__init__()
        self.node_name: str = node_name
        self.action_servers: list[ActionServer] = action_servers
        self.action_clients: list[ActionClient] = action_clients
        self.publishers: list[Publisher] = publishers
        self.subscribers: list[Subscriber] = subscribers
        self.services: list[Service] = services
        self.clients: list[Client] = clients
        self.host = host
