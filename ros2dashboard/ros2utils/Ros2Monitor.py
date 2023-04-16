from PySide2.QtCore import Signal, QObject, QTimer, QThread

from ros2dashboard.edge.Topic import Topic
from ros2dashboard.edge.Client import Client
from ros2dashboard.edge.Subscriber import Subscriber
from ros2dashboard.edge.Publisher import Publisher
from ros2dashboard.edge.Service import Service
from ros2dashboard.ros2utils.Network import Host
from ros2dashboard.devices.Ros2Node import Ros2Node
from ros2dashboard.ros2utils.NetworkDiscover import NetworkDiscover

class Ros2Monitor(QObject):
    new_topics = Signal(object)
    new_hosts = Signal(object)
    new_subscribers = Signal(object)
    new_publishers = Signal(object)
    new_services = Signal(object)
    new_clients = Signal(object)
    new_nodes = Signal(object)

    def monitor_changes(self):
        topics = self.network_discover.find_topics()
        publishers = self.network_discover.find_publishers()
        subscribers = self.network_discover.find_subscribers()
        service_servers = self.network_discover.find_action_servers()
        service_clients = self.network_discover.find_action_clients()
        nodes = self.network_discover.find_nodes()
        hosts = self.network_discover.find_hosts()

        if topics != self.topics:
            self.topics = topics
            self.new_topics.emit(topics)
        if hosts != self.hosts:
            self.hosts = hosts
            self.new_hosts.emit(hosts)
        if subscribers != self.subscribers:
            self.subscribers = subscribers
            self.new_subscribers.emit(subscribers)
        if publishers != self.publishers:
            self.publishers = publishers
            self.new_publishers.emit(publishers)
        if service_servers != self.services:
            self.services = service_servers
            self.new_services.emit(service_servers)
        if service_clients != self.clients:
            self.clients = service_clients
            self.new_clients.emit(service_clients)
        if nodes != self.nodes:
            self.nodes = nodes
            self.new_nodes.emit(nodes)

    def __init__(self, args, parent=None):
        super(Ros2Monitor, self).__init__(parent)

        self.topics = []
        self.hosts = []
        self.subscribers = []
        self.publishers = []
        self.services = []
        self.clients = []
        self.nodes = []
        self.network_discover = NetworkDiscover(args=args)
    
    def start(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.monitor_changes)
        self.timer.setInterval(500)
        self.timer.start()