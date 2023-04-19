import hashlib

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

    def has_publisher(self, topic_name) -> bool:
        return len([p for p in self.publishers if p.topic_name == topic_name]) != 0

    def has_subscription(self, topic_name) -> bool:
        return len([s for s in self.subscribers if s.topic_name == topic_name]) != 0

    def get_publisher(self, topic_name) -> Publisher:
        publisher = [
            publisher for publisher in self.publishers if publisher.topic_name == topic_name]
        assert (len(publisher) == 1 or len(publisher) == 0)
        return None if len(publisher) == 0 else publisher[0]

    def get_subscription(self, topic_name) -> Subscriber:
        subscription = [
            subscriber for subscriber in self.publishers if subscriber.topic_name == topic_name]
        assert (len(subscription) == 1 or len(subscription) == 0)
        return None if len(subscription) == 0 else subscription[0]

    def clear_inputs(self):
        """ TODO: There is some strange behaviour with delete_input and delete_output functions.
        Sometimes, it doesn't work as expected and we forced to use it multiple 
        times to delete inputs completly
        """
        while self.input_ports():
            for input_port in self.input_ports():
                self.delete_input(input_port)

    def clear_outputs(self):
        while self.output_ports():
            for output_port in self.output_ports():
                self.delete_output(output_port)

    def unique_key(self) -> str:
        return hashlib.sha256(bytes(self.node_name, 'utf-8')).hexdigest()
