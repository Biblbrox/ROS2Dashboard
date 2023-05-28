from ros2dashboard.devices.GenericNode import GenericNode
from ros2dashboard.edge.ActionClient import ActionClient
from ros2dashboard.edge.ActionServer import ActionServer
from ros2dashboard.edge.Client import Client
from ros2dashboard.edge.Executable import Executable
from ros2dashboard.edge.Package import Package
from ros2dashboard.edge.Service import Service
from ros2dashboard.edge.Topic import Topic
from ros2dashboard.edge.Publisher import Publisher
from ros2dashboard.edge.Subscriber import Subscriber
from ros2dashboard.core.Logger import logging


class Ros2Storage:
    def __init__(self) -> None:
        self._storage = {
            'nodes': {},
            'topics': {},
            'services': {},
            'publishers': {},
            'subscribers': {},
            'packages': {},
            'clients': {},
            'executables': {},
            'action_clients': {},
            'action_servers': {}
        }

    @property
    def nodes(self) -> list[GenericNode]:
        return self._storage['nodes']

    @property
    def action_servers(self) -> list[ActionServer]:
        return self._storage['action_servers']

    @property
    def action_clients(self) -> list[ActionClient]:
        return self._storage['action_clients']

    @property
    def clients(self) -> list[Client]:
        return self._storage['clients']

    @property
    def executables(self) -> list[Executable]:
        return self._storage['executables']

    @property
    def packages(self) -> list[Package]:
        return self._storage['packages']

    @property
    def services(self) -> list[Service]:
        return self._storage['services']

    @property
    def topics(self) -> list[Topic]:
        return self._storage['topics']

    @property
    def publishers(self) -> list[Publisher]:
        return self._storage['publishers']

    @property
    def subscribers(self) -> list[Subscriber]:
        return self._storage['subscribers']

    def update(self, state: dict):
        """Update method used to update Ros2Storage state.

        Args:
            state (dict): state dictionary has the following structure: 
        {
            'packages': [],
            'nodes': [],
            'topics': [],
            'services': [],
            'publishers': [],
            'subscribers': [],
            'clients': [],
            'executables': [],
            'action_clients': [],
            'action_servers': []
        }
        """
        # Top level entities
        self._storage['packages'] = {}
        self._storage['executables'] = {}
        self._storage['nodes'] = {}

        # Node level entities
        self._storage['topics'] = {}
        self._storage['services'] = {}
        self._storage['publishers'] = {}
        self._storage['subscribers'] = {}
        self._storage['clients'] = {}
        self._storage['action_clients'] = {}
        self._storage['action_servers'] = {}

        if 'packages' in state:
            for package in state['packages']:
                self._storage['packages'][package.name] = []

        if 'nodes' in state:
            for node in state['nodes']:
                package_name = node.package_name
                if package_name not in self._storage['packages']:
                    logging.error(
                        f"Node package {package_name} doesn't exists in Ros2Storage. Please, add this package to state")
                    raise ValueError
                if package_name not in self._storage['nodes']:
                    self._storage['nodes'][package_name] = []

                self._storage['nodes'][package_name].append(node)

        if 'topics' in state:
            for topic in state['topics']:
                self._storage[topic.node_name] = topic

        if 'services' in state:
            for service in state['services']:
                self._storage[service.node_name] = service

        if 'publishers' in state:
            for publisher in state['publishers']:
                self._storage[publisher.node_name] = publisher

        if 'subscribers' in state:
            for subscriber in state['subscribers']:
                self._storage[subscriber.node_name] = subscriber

        if 'clients' in state:
            for client in state['clients']:
                self._storage[client.node_name] = client

        if 'cation_clients' in state:
            for action_client in state['action_clients']:
                self._storage[action_client.node_name] = action_client

        if 'acation_servers' in state:
            for action_server in state['action_servers']:
                self._storage[action_server.node_name] = action_server

    def add_node(self, node: GenericNode):
        package_name = node.package_name
        if package_name not in self._storage['nodes']:
            self._storage['nodes'][package_name] = []
        self._storage['nodes'][package_name].append(node)

    def add_action_server(self, action_server: ActionServer):
        node_name = action_server.node_name
        if node_name not in self._storage['action_servers']:
            self._storage['action_servers'][node_name] = []
        self._storage['action_servers'][node_name].append(action_server)

    def add_action_client(self, action_client: ActionClient):
        node_name = action_client.node_name
        if node_name not in self._storage['action_clients']:
            self._storage['action_clients'][node_name] = []
        self._storage['action_clients'][node_name].append(action_client)

    def add_client(self, client: Client):
        node_name = client.node_name
        if node_name not in self._storage['clients']:
            self._storage['clients'][node_name] = []
        self._storage['clients'][node_name].append(client)

    def add_executable(self, executable: Executable):
        package_name = executable.package_name
        if package_name not in self._storage['executables']:
            self._storage['executables'][package_name] = []
        self._storage['executables'][package_name].append(executable)

    def add_package(self, package: Package):
        package_name = package.name
        self._storage['packages'][package_name] = package

    def add_service(self, service: Service):
        node_name = service.node_name
        if node_name not in self._storage['services']:
            self._storage['services'][node_name] = []
        self._storage['services'][node_name].append(service)

    def add_topic(self, topic: Topic):
        node_name = topic.node_name
        if node_name not in self._storage['topics']:
            self._storage['topics'][node_name] = []
        self._storage['topics'][node_name].append(topic)

    def add_publisher(self, publisher: Publisher):
        node_name = publisher.node_name
        if node_name not in self._storage['publishers']:
            self._storage['publishers'][node_name] = []
        self._storage['publishers'][node_name].append(publisher)

    def add_subscriber(self, subscriber: Subscriber):
        node_name = subscriber.node_name
        if node_name not in self._storage['subscrirbers']:
            self._storage['subscrirbers'][node_name] = []
        self._storage['subscrirbers'][node_name].append(subscriber)

    def add_nodes(self, nodes: list[GenericNode]):
        [self.add_node(node) for node in nodes]

    def add_action_servers(self, action_servers: list[ActionServer]):
        [self.add_action_server(action_server)
         for action_server in action_servers]

    def add_action_client(self, action_client: ActionClient):
        [self.add_action_client(action_client)
         for action_client in action_client]

    def add_clients(self, clients: list[Client]):
        [self.add_client(client) for client in clients]

    def add_executables(self, executables: list[Executable]):
        [self.add_executable(executable) for executable in executables]

    def add_packages(self, packages: list[Package]):
        [self.add_package(package) for package in packages]

    def add_services(self, services: list[Service]):
        [self.add_service(service) for service in services]

    def add_topics(self, topics: list[Topic]):
        [self.add_topic(topic) for topic in topics]

    def add_publishers(self, publishers: list[Publisher]):
        [self.add_publisher(publisher) for publisher in publishers]

    def add_subscribers(self, subscribers: list[Subscriber]):
        [self.add_subscriber(subscriber) for subscriber in subscribers]

    def remove_node(self, node: GenericNode):
        package_name = node.package_name
        assert (package_name in self._storage['nodes'])

        # Remove any ros2 entities created with this node
        if node.name in self._storage['action_servers']:
            self.remove_action_server(
                self._storage['action_servers'][node.name])
        if node.name in self._storage['action_clients']:
            self.remove_action_client(
                self._storage['action_clients'][node.name])
        if node.name in self._storage['clients']:
            self.remove_client(self._storage['clients'][node.name])
        if node.name in self._storage['services']:
            self.remove_service(self._storage['services'][node.name])
        if node.name in self._storage['topics']:
            self.remove_topic(self._storage['topics'][node.name])
        if node.name in self._storage['publishers']:
            self.remove_publisher(self._storage['publishers'][node.name])
        if node.name in self._storage['subscribers']:
            self.remove_subscriber(self._storage['subscribers'][node.name])

        # Remove node itself
        self._storage['nodes'].pop(package_name)

    def remove_action_server(self, action_server: ActionServer):
        node_name = action_server.node_name
        assert node_name in self._storage['action_servers']
        self._storage['action_servers'][node_name].pop()

    def remove_action_client(self, action_client: ActionClient):
        node_name = action_client.node_name
        assert node_name in self._storage['action_clients']
        self._storage['action_clients'][node_name].pop()

    def remove_client(self, client: Client):
        node_name = client.node_name
        assert node_name in self._storage['clients']
        self._storage['client'][node_name].pop()

    def remove_executable(self, executable: Executable):
        assert executable.package_name in self._storage['executables']
        self._storage['executables'].pop(executable.package_name)

    def remove_package(self, package: Package):
        assert package.name in self._storage['packages']
        self._storage['packages'].pop(package.name)

    def remove_service(self, service: Service):
        node_name = service.node_name
        assert node_name in self._storage['services']
        self._storage['services'][node_name].pop()

    def remove_topic(self, topic: Topic):
        node_name = topic.node_name
        assert node_name in self._storage['topics']
        self._storage['topics'][node_name].pop()

    def remove_publisher(self, publisher: Publisher):
        node_name = publisher.node_name
        assert node_name in self._storage['publishers']
        self._storage['publishers'][node_name].pop()

    def remove_subscriber(self, subscriber: Subscriber):
        node_name = subscriber.node_name
        assert node_name in self._storage['subscribers']
        self._storage['subscribers'][node_name].pop()
