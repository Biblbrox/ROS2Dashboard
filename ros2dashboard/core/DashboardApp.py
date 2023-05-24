from threading import Lock, RLock

from NodeGraphQt import NodeGraph
from NodeGraphQt import BaseNode

from PySide2.QtCore import Signal, Slot
from PySide2.QtQuickWidgets import QQuickWidget
from sensor_msgs.msg import Image

from ros2dashboard.ros2utils.NetworkDiscover import NetworkDiscover
from ros2dashboard.core.Logger import logging

from ros2dashboard.edge.Topic import Topic
from ros2dashboard.edge.Client import Client
from ros2dashboard.edge.ActionClient import ActionClient
from ros2dashboard.edge.ActionServer import ActionServer
from ros2dashboard.edge.Subscriber import Subscriber
from ros2dashboard.edge.Publisher import Publisher
from ros2dashboard.edge.Service import Service
from ros2dashboard.edge.Package import Package
from ros2dashboard.ros2utils.Network import Host
from ros2dashboard.ros2utils.Ros2Discover import Ros2Discover
from ros2dashboard.devices.GenericNode import GenericNode
from ros2dashboard.qml_models.PackageListModel import PackageModel


class DashboardApp:
    def __init__(self, package_model: PackageModel, parent = None):
        self.parent = parent
        self.nodes: list[GenericNode] = []
        self.subscribers: list[Subscriber] = []
        self.publishers: list[Publisher] = []
        self.packages: list[Package] = []
        self.package_model = package_model
        # self.lock = RLock()

        # Init gui
        self.graph = NodeGraph()
        self.graph.register_node(GenericNode)
        self.graph_widget = self.graph.widget


    def update_models(self):
        
        pass

    @Slot(object)
    def update_subscribers(self, subscribers: list[Subscriber]):
        #self.lock.acquire()
        self.subscribers = subscribers
        self.update_connections(self.subscribers, self.publishers)
        #self.lock.release()

    @Slot(object)
    def update_publishers(self, publishers: list[Publisher]):
        #self.lock.acquire()
        self.publishers = publishers
        self.update_connections(self.subscribers, self.publishers)
        #self.lock.release()

    @Slot(object)
    def update_packages(self, new_packages: list[Package]):
        #self.lock.acquire()
        self.packages = new_packages
        self.package_model.packages = new_packages
        #self.lock.release()
        

    def remove_node_gui(self, node: any):
        self.graph.remove_node(node)

    def create_node_gui(self, node, pos: tuple) -> GenericNode:
        node_identifier = node.__identifier__ + ".GenericNode"
        created_node: BaseNode = self.graph.create_node(
                    node_identifier, name=node.node_name, pos=pos)
        created_node.set_port_deletion_allowed(True)
        created_node.node_name = node.node_name
        return created_node

    @Slot(object)
    def update_nodes(self, new_nodes: list[GenericNode]):
        #self.lock.acquire()
        try:
            for node in self.nodes:
                if node.node_name not in [ros2_node.node_name for ros2_node in new_nodes]:
                    logging.debug(f"Remove node {node.node_name}")
                    self.remove_node_gui(node)

            pos = (10, 10)
            for node in new_nodes:
                if node.node_name in [node.node_name for node in self.nodes]:
                    logging.debug(
                        f"Node with name {node.node_name} was already addedd")
                    continue

                logging.debug(f"Creatig ui node with name: {node.node_name}")
                
                created_node: BaseNode = self.create_node_gui(node, pos)
                self.nodes.append(created_node)
                pos += (10, 10)

                logging.debug(f"Node with name {node.node_name} were added")
        except Exception as e:
            logging.error(
                f"Unable to update nodes. Error: {e}")
            exit(-1)

        self.update_connections(self.subscribers, self.publishers)

        self.graph.auto_layout_nodes(self.nodes, start_nodes=self.nodes)

        #self.lock.release()

    def update_connections(self, subscriptions: list[Subscriber], publishers: list[Publisher]):
        #self.lock.acquire()
        publisher_color = (255, 0, 0)
        subscriber_color = (0, 255, 0)

        try:
            # Update subscriptions and publishers
            for idx, node in enumerate(self.nodes):
                # Updating subscribers
                node.subscribers = [
                    subscription for subscription in subscriptions if subscription.node_name == node.node_name]
                node.clear_inputs()
                if len(node.inputs()) != 0:
                    logging.error("This should not be happened")
                    exit(-1)
                for subscriber in node.subscribers:
                    logging.debug(
                        f"Registering input port with name {subscriber.topic_name} for node {node.node_name}")
                    port = node.add_input(
                        name=subscriber.topic_name, color=subscriber_color)
                    subscriber.port = port

                # Updating publishers
                node.publishers = [
                    publisher for publisher in publishers if publisher.node_name == node.node_name]
                node.clear_outputs()
                if len(node.outputs()) != 0:
                    logging.error("This should not be happened")
                    exit(-1)
                for publisher in node.publishers:
                    logging.debug(
                        f"Registering output port with name {publisher.topic_name} for node {node.node_name}")
                    port = node.add_output(
                        name=publisher.topic_name, color=publisher_color, multi_output=True)
                    publisher.port = port


            for idx, node in enumerate(self.nodes):
                for nd in self.nodes[idx + 1:]:
                    for subscriber in nd.subscribers:
                        publisher = node.get_publisher(subscriber.topic_name)
                        if publisher is not None:
                            publisher.port.connect_to(subscriber.port)

        except Exception as e:
            logging.error(
                f"Unable to update connections. Error: {e}")
            exit(-1)
        #self.lock.release()

    def find_publisher(self, topic_name) -> Publisher:
        for node in self.nodes:
            publisher = node.get_publisher(topic_name)
            if publisher:
                return publisher

        return None

    def find_subscriber(self, topic_name) -> Subscriber:
        for node in self.nodes:
            subscriber = node.get_subscription(topic_name)
            if subscriber:
                return subscriber

        return None

    def show(self):
        # show the node graph widget.
        self.graph_widget.show()