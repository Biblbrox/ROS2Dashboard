import unittest
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
from ros2dashboard.core.Ros2Storage import Ros2Storage


class TestRos2Storage(unittest.TestCase):

    def setUp(self):
        self.storage = Ros2Storage()

    def test_add_node(self):
        node = GenericNode('node_name', 'package_name')
        self.storage.add_node(node)
        self.assertIn(node, self.storage.nodes())

    def test_add_action_server(self):
        action_server = ActionServer('server_name', 'node_name', 'package_name')
        self.storage.add_action_server(action_server)
        self.assertIn(action_server, self.storage.action_servers())

    def test_add_action_client(self):
        action_client = ActionClient('client_name', 'node_name', 'package_name')
        self.storage.add_action_client(action_client)
        self.assertIn(action_client, self.storage.action_clients())

    def test_add_client(self):
        client = Client('client_name', 'node_name', 'package_name')
        self.storage.add_client(client)
        self.assertIn(client, self.storage.clients())

    def test_add_executable(self):
        executable = Executable('executable_name', 'package_name')
        self.storage.add_executable(executable)
        self.assertIn(executable, self.storage.executables())

    def test_add_package(self):
        package = Package('package_name')
        self.storage.add_package(package)
        self.assertIn(package, self.storage.packages())

    def test_add_service(self):
        service = Service('service_name', 'node_name', 'package_name', 'service_type')
        self.storage.add_service(service)
        self.assertIn(service, self.storage.services())

    def test_add_topic(self):
        topic = Topic('topic_name', 'node_name', 'package_name', 'message_type')
        self.storage.add_topic(topic)
        self.assertIn(topic, self.storage.topics())

    def test_add_publisher(self):
        publisher = Publisher('publisher_name', 'node_name', 'package_name', 'topic_name', 'message_type')
        self.storage.add_publisher(publisher)
        self.assertIn(publisher, self.storage.publishers())

    def test_add_subscriber(self):
        subscriber = Subscriber('subscriber_name', 'node_name', 'package_name', 'topic_name', 'message_type')
        self.storage.add_subscriber(subscriber)
        self.assertIn(subscriber, self.storage.subscribers())


if __name__ == '__main__':
    unittest.main()
