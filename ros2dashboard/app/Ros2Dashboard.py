import sys
import configparser

from PySide2.QtGui import QGuiApplication
from PySide2.QtWidgets import QApplication
from PySide2 import QtWidgets
from NodeGraphQt import NodeGraph, BaseNode

from ros2dashboard.devices.Camera import CameraNode
from ros2dashboard.devices.Stereo import StereoNode


class Ros2Dashboard:
    def __init__(self) -> None:
        # create node graph controller.
        self.graph = NodeGraph()

        self.node_types = [CameraNode, StereoNode]
        self.nodes = []

        # register the FooNode node class.
        self.register_nodes()

        self.graph_widget = self.graph.widget

        # create two nodes.
        self.create_nodes()

        # connect node_a to node_b
        # node_a.set_output(0, node_b.input(2))

    def register_nodes(self):
        for node in self.node_types:
            self.graph.register_node(node)

    def create_nodes(self):
        pos = (10, 10)
        for node in self.node_types:
            self.nodes.append(self.graph.create_node(
                node.__identifier__ + f".{str(node.__name__)}", name='node A', pos=pos))
            pos += (10, 10)

    def show(self):
        # show the node graph widget.
        self.graph_widget.show()
