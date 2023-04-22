import subprocess
import socket
import uuid
import time
from threading import Lock, RLock

from PySide2.QtCore import Signal, QObject
import rclpy
from rclpy.node import Node
import ros2node.api as cli
import ros2lifecycle.api as api


from ros2dashboard.edge.Topic import Topic
from ros2dashboard.edge.Client import Client
from ros2dashboard.edge.ActionClient import ActionClient
from ros2dashboard.edge.ActionServer import ActionServer
from ros2dashboard.edge.Subscriber import Subscriber
from ros2dashboard.edge.Publisher import Publisher
from ros2dashboard.edge.Service import Service
from ros2dashboard.ros2utils.Network import Host
from ros2dashboard.ros2utils.Ros2Discover import Ros2Discover
from ros2dashboard.devices.Ros2Node import Ros2Node, VISUALIZATION_NODE_PREFIX


def generate_random_node_name():
    name = str(uuid.uuid1()).replace('-', '_')
    if not name[0].isalpha():
        name = f"a{name}"

    return name


class NetworkDiscover(Ros2Discover):
    def __init__(self, args) -> None:
        self.lock = RLock()
        if not rclpy.ok():
            rclpy.init(args=args)

        self.ip = socket.gethostbyname(socket.gethostname())

    def find_node_topics(self, node_name: str, topics_type="all"):
        self.lock.acquire()
        # Run the 'ros2 node info' command to get information about the node
        proc = subprocess.Popen(
            ['ros2', 'node', 'info', node_name], stdout=subprocess.PIPE)
        output, _ = proc.communicate()
        # Get a list of all the publishers associated with the node
        topics = []
        if topics_type == "Publisher":
            # Parse the output to find publishers and subscribers
            is_publisher = False
            for line in output.decode().split('\n'):
                line = line.strip()
                if line.startswith('Publishers:'):
                    is_publisher = True
                    continue

                if not line.startswith('/') and is_publisher:
                    is_publisher = False

                if is_publisher:
                    publishers_info = [word.strip()
                                       for word in line.split(':')]
                    topics.append(
                        Publisher(node_name, topic_name=publishers_info[0], topic_type=publishers_info[1]))
        elif topics_type == "Subscriber":
            # Parse the output to find publishers and subscribers
            is_subscriber = False
            for line in output.decode().split('\n'):
                line = line.strip()
                if line.startswith('Subscribers:'):
                    is_subscriber = True
                    continue

                if not line.startswith('/') and is_subscriber:
                    break

                if is_subscriber:
                    subscribers_info = [word.strip()
                                        for word in line.split(':')]
                    topics.append(Subscriber(
                        node_name, topic_name=subscribers_info[0], topic_type=subscribers_info[1]))
        else:
            # Parse the output to find publishers and subscribers
            is_topic = False
            for line in output.decode().split('\n'):
                line = line.strip()

                if line.startswith('Subscribers:') or line.startswith('Publishers:'):
                    is_topic = True
                    continue

                if not line.startswith('/') and is_topic:
                    break

                if is_topic:
                    topics_info = [word.strip()
                                   for word in line.split(':')]
                    topics.append(
                        Topic(node_name, topic_name=topics_info[0], topic_type=topics_info[1]))

        self.lock.release()
        return topics

    def find_action_clients(self, node_name=None) -> list[ActionClient]:
        return []

    def find_action_servers(self, node_name=None) -> list[ActionServer]:
        return []

    def find_topics(self, node_name=None) -> list[Topic]:
        self.lock.acquire()
        """
        Find topics for all nodes
        """
        topics = []
        if not node_name:
            node_names = self.find_node_names()
            for node_name_ in node_names:
                node_topics = self.find_node_topics(node_name_)

                for node_topic in node_topics:
                    topics.append(node_topic)
        else:
            node_topics = self.find_node_topics(node_name)
            for node_topic in node_topics:
                topics.append(node_topic)

        self.lock.release()
        return topics

    def find_subscribers(self, node_name=None) -> list[Subscriber]:
        self.lock.acquire()
        """
        Find subscribers for all nodes
        """
        subscribers = []

        if not node_name:
            for node_name_ in self.find_node_names():
                node_subscribers = self.find_node_topics(
                    node_name_, "Subscriber")
                for node_subscriber in node_subscribers:
                    subscribers.append(node_subscriber)
        else:
            node_subscribers = self.find_node_topics(node_name, "Subscriber")
            for node_subscriber in node_subscribers:
                subscribers.append(node_subscriber)

        self.lock.release()
        return subscribers

    def find_publishers(self, node_name=None) -> list[Publisher]:
        self.lock.acquire()
        """
        Find publishers for all nodes
        """
        publishers = []

        if not node_name:
            for node_name_ in self.find_node_names():
                node_publishers = self.find_node_topics(
                    node_name_, "Publisher")

                for node_publisher in node_publishers:
                    publishers.append(node_publisher)
        else:
            node_publishers = self.find_node_topics(node_name, "Publisher")
            for node_publisher in node_publishers:
                publishers.append(node_publisher)

        self.lock.release()
        return publishers

    def find_service_servers(self):
        return []

    def find_nodes(self) -> list[Ros2Node]:
        self.lock.acquire()
        node_names = self.find_node_names()
        nodes = []
        for node_name in node_names:
            publishers = self.find_publishers(node_name=node_name)
            subscribers = self.find_subscribers(node_name=node_name)
            nodes.append(
                Ros2Node(node_name, publishers_=publishers, subscribers_=subscribers))

        self.lock.release()
        return nodes

    def find_node_names(self):
        # Run the `ros2 node list` command and capture its output

        output = subprocess.check_output(["ros2", "node", "list"])

        # Decode the byte string to a regular string
        node_names = output.decode('utf-8').split()

        result = []
        for node_name in node_names:
            if not node_name.startswith(f"/{VISUALIZATION_NODE_PREFIX}"):
                result.append(node_name)

        return result

    def find_hosts(self):
        prefix = self.ip.rsplit('.', 1)[0]

        hosts = []
        for i in range(1, 255):
            address = prefix + '.' + str(i)
            result = subprocess.call(
                ['ping', '-c', '1', '-W', '1', address], stdout=subprocess.DEVNULL)
            if result == 0:
                hosts.append(Host(address, "_"))

        return hosts

    def find_ros2_hosts(self):
        pass
