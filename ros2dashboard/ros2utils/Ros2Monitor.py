from PySide2.QtCore import Signal, QObject, QTimer, QThread

from ros2dashboard.edge.Topic import Topic
from ros2dashboard.edge.Client import Client
from ros2dashboard.edge.Subscriber import Subscriber
from ros2dashboard.edge.Publisher import Publisher
from ros2dashboard.edge.Service import Service
from ros2dashboard.ros2utils.Network import Host
from ros2dashboard.devices.Ros2Node import Ros2Node
from ros2dashboard.edge.GraphEdge import GraphEdge
from ros2dashboard.ros2utils.NetworkDiscover import NetworkDiscover
from ros2dashboard.app.logger import logging


def has_method(obj, method):
    return getattr(obj, method, None)  # and callable(obj.method)


class Ros2Monitor(QObject):
    new_topics = Signal(object)
    new_hosts = Signal(object)
    new_subscribers = Signal(object)
    new_publishers = Signal(object)
    new_services = Signal(object)
    new_clients = Signal(object)
    new_nodes = Signal(object)

    def is_equal_edges(self, old_edges: list[GraphEdge], new_edges: list[GraphEdge]) -> bool:
        try:
            logging.debug(f"{len(old_edges)}, {old_edges}")
            logging.debug(f"{len(new_edges)}, {new_edges}")

            if len(old_edges) != len(new_edges):
                return False

            logging.debug("is_equal_edges")
            # assert [has_method(edge, "unique_key") for edge in old_edges]
            # assert [has_method(edge, "unique_key") for edge in new_edges]

            old_edges = sorted(old_edges, key=lambda edge: edge.unique_key())
            new_edges = sorted(new_edges, key=lambda edge: edge.unique_key())
            for old_edge, new_edge in zip(old_edges, new_edges):
                if old_edge.unique_key() != new_edge.unique_key():
                    return False
        except Exception as e:
            logging.error(f"Error while is_equal_edges: {e}")

        return True

    def monitor_changes(self):
        topics = self.network_discover.find_topics()
        publishers = self.network_discover.find_publishers()
        subscribers = self.network_discover.find_subscribers()
        service_servers = self.network_discover.find_action_servers()
        service_clients = self.network_discover.find_action_clients()
        nodes = self.network_discover.find_nodes()
        hosts = self.network_discover.find_hosts()

        if not self.is_equal_edges(topics, self.topics):
            self.topics = topics
            self.new_topics.emit(topics)
        if not self.is_equal_edges(subscribers, self.subscribers):
            self.subscribers = subscribers
            self.new_subscribers.emit(subscribers)
        if not self.is_equal_edges(publishers, self.publishers):
            self.publishers = publishers
            self.new_publishers.emit(publishers)
        if not self.is_equal_edges(service_servers, self.services):
            self.services = service_servers
            self.new_services.emit(service_servers)
        if not self.is_equal_edges(service_clients, self.clients):
            self.clients = service_clients
            self.new_clients.emit(service_clients)
        if not self.is_equal_edges(nodes, self.nodes):
            self.nodes = nodes
            self.new_nodes.emit(nodes)
        if not self.is_equal_edges(hosts, self.hosts):
            self.hosts = hosts
            self.new_hosts.emit(hosts)

    def __init__(self, args, parent=None):
        super(Ros2Monitor, self).__init__(parent)

        self.topics: list[Topic] = []
        self.hosts: list[Host] = []
        self.subscribers: list[Host] = []
        self.publishers: list[Publisher] = []
        self.services: list[Service] = []
        self.clients: list[Client] = []
        self.nodes: list[Ros2Node] = []
        self.network_discover = NetworkDiscover(args=args)

    def start(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.monitor_changes)
        self.timer.setInterval(500)
        self.timer.start()
