from NodeGraphQt import NodeGraph
from NodeGraphQt import BaseNode

from PySide2.QtCore import Signal, Slot

from ros2dashboard.ros2utils.NetworkDiscover import NetworkDiscover
from ros2dashboard.devices.Ros2Node import Ros2Node
from ros2dashboard.app.logger import logging

class Ros2Dashboard:
    def __init__(self) -> None:
        # create node graph controller.
        self.graph = NodeGraph()
        self.ui_nodes: list[BaseNode] = []

        # register ui nodes
        self.register_nodes()

        self.graph_widget = self.graph.widget

    def register_nodes(self):
        self.graph.register_node(Ros2Node)

    @Slot(object)
    def update_nodes(self, ros2_nodes: list[Ros2Node]):
        pos = (10, 10)
        for ui_node in self.ui_nodes:
            logging.debug(f"****{ui_node.node_name}*******")
            if ui_node.node_name not in [ros2_node.node_name for ros2_node in ros2_nodes]:
                logging.debug(f"Remove node {ui_node.node_name}")
                self.graph.remove_node(ui_node)

        for ros2_node in ros2_nodes:
            if ros2_node.node_name in [node.node_name for node in self.ui_nodes]:
                logging.debug(f"Node with name {ros2_node.node_name} was already addedd")
                continue

            node_identifier = ros2_node.__identifier__ + ".Ros2Node"
            logging.debug(f"Creatig ui node with identifier: {node_identifier}")

            ui_node: BaseNode = self.graph.create_node(
                node_identifier, name=ros2_node.node_name, pos=pos)
            logging.debug(f"Node with name {ros2_node.node_name} addedd")
            publisher_color = (255, 0, 0)
            subscriber_color = (0, 255, 0)
            for publisher in ros2_node.publishers:
                # Check if some node subscribed to this publisher
                for ros2_node_ in ros2_nodes:
                    for subscriber in ros2_node_.subscribers:
                        if subscriber.subscriber_name == publisher.publisher_name:
                            pass
                ui_node.add_output(name=publisher.publisher_name, color=publisher_color)

            for subscriber in ros2_node.subscribers:
                ui_node.add_input(name=subscriber.subscriber_name, color=subscriber_color)

            self.ui_nodes.append(ros2_node)

            pos += (10, 10)


    def show(self):
        # show the node graph widget.
        self.graph_widget.show()
