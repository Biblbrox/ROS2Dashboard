import hashlib
import uuid

import PySide2.QtWidgets as QtWidgets
import rclpy
from NodeGraphQt import BaseNode, NodeBaseWidget
from PySide2.QtCore import Signal, Slot, QSize
from PySide2 import QtGui
from rclpy.node import Node
from sensor_msgs.msg import Image

from ros2dashboard.app.logger import logging
from ros2dashboard.edge.ActionClient import ActionClient
from ros2dashboard.edge.ActionServer import ActionServer
from ros2dashboard.edge.Client import Client
from ros2dashboard.edge.Publisher import Publisher
from ros2dashboard.edge.Service import Service
from ros2dashboard.edge.Subscriber import Subscriber
from ros2dashboard.edge.Topic import Topic
from ros2dashboard.visualizer.TopicVisualizer import (TopicVisualizer,
                                                      VideoVisualizer)

""" This prefix is used to avoid to show unecessary nodes in graph, which are 
begin used for internal purposes
"""
VISUALIZATION_NODE_PREFIX = "ros2dashboard_visualization_"


class NodeControlWidget(QtWidgets.QWidget):
    """
    Custom widget to be embedded inside a node.
    """

    def __init__(self, node_name: str, parent=None):
        super(NodeControlWidget, self).__init__(parent)
        logging.debug("Creating NodeControlWidget")
        self.btn_visualize = QtWidgets.QPushButton(
            text='Topic data', parent=self)
        self.btn_stop = QtWidgets.QPushButton(text='Stop', parent=self)
        self.node_name = node_name
        self.visualizer = None

        self.layout = QtWidgets.QVBoxLayout(self)
        # self.layout.setContentsMargins(0, 0, 0, 0)
        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.addWidget(self.btn_visualize)
        btn_layout.addWidget(self.btn_stop)
        btn_wiget = QtWidgets.QWidget()
        btn_wiget.setLayout(btn_layout)
        self.layout.addWidget(btn_wiget)

        self.visualiser_opened = False
        self.visualization_inited = False
        self.adjustSize()
        self.min_size = QSize(0, 0)

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        if self.min_size == QSize(0, 0):
            self.min_size = self.size()

    def toggle_layout(self):
        if self.visualizer is None:
            logging.warning("Unable to open visualizer, it is not ready")
            return

        if not self.visualiser_opened:
            logging.debug("Show visualizer wiget")
            if self.visualization_inited:
                self.visualizer.widget.setVisible(True)
                self.adjustSize()
            else:
                self.layout.addWidget(self.visualizer.widget)
                self.adjustSize()

            self.visualiser_opened = True
            self.visualization_inited = True
        else:
            # self.layout.removeWidget(self.visualizer.widget)
            self.visualizer.widget.setVisible(False)
            self.visualiser_opened = False
            self.adjustSize()
            self.resize(self.min_size)

    def add_visualizer(self, message_type, topic_name):
        if self.visualizer is None:
            self.visualizer = TopicVisualizer(self.node_name, parent=self)

        self.visualizer.subscribe_topic(topic_name, message_type)

    def has_visualizer(self, message_type, topic_name) -> bool:
        if self.visualizer is None:
            self.visualizer = TopicVisualizer(self.node_name, parent=self)

        return self.visualizer.has_visualizer(topic_name, message_type)

    def support_visualizer(self, message_type):
        if self.visualizer is None:
            self.visualizer = TopicVisualizer(self.node_name, parent=self)

        return self.visualizer.support_visualizer(message_type)


class NodeControlWidgetWrapper(NodeBaseWidget):
    """
    Wrapper that allows the widget to be added in a node object.
    """

    def __init__(self, node_name: str, parent=None):
        super(NodeControlWidgetWrapper, self).__init__(parent)
        self.parent = parent
        # set the name for node property.
        self.node_name = node_name
        self.set_name(self.node_name + ".widget")

        # set the label above the widget.
        self.set_label('Custom Widget')

        # set the custom widget.
        self.set_custom_widget(NodeControlWidget(
            self.node_name, parent=self.widget()))

        # connect up the signals & slots.
        self.wire_signals()

    def get_value(self):
        self.control_widget = self.get_custom_widget()
        return "value"

    def set_value(self, text):
        pass

    def add_visualization(self, message_type, topic_name):
        self.control_widget.add_visualizer(message_type, topic_name)

    def has_visualizer(self, message_type, topic_name):
        return self.control_widget.has_visualizer(message_type, topic_name)

    def support_visualizer(self, message_type):
        return self.control_widget.support_visualizer(message_type)

    def wire_signals(self):
        self.control_widget = self.get_custom_widget()

        # wire up the button.
        self.control_widget.btn_visualize.clicked.connect(
            self.on_btn_viz_clicked)
        self.control_widget.btn_stop.clicked.connect(self.on_btn_stop_clicked)

    @Slot()
    def on_btn_viz_clicked(self):
        print('Clicked on node: "{}"'.format(self.node.name()))
        self.control_widget = self.get_custom_widget()
        self.control_widget.toggle_layout()
        self.adjustSize()
        

    @Slot()
    def on_btn_stop_clicked(self):
        print('Clicked on node: "{}"'.format(self.node.name()))


class Ros2Node(BaseNode):

    # unique node identifier domain.
    __identifier__ = 'ros2.node'
    # initial default node name.
    NODE_NAME = 'Ros2Node'

    def __init__(self, node_name_="", host_="localhost", action_servers_: list[ActionServer] = [],
                 action_clients_: list[ActionClient] = [], publishers_: list[Publisher] = [],
                 subscribers_: list[Subscriber] = [], services_: list[Service] = [], clients_: list[Client] = []):
        try:
            super(Ros2Node, self).__init__()
            self._node_name: str = node_name_
            self._action_servers: list[ActionServer] = action_servers_
            self._action_clients: list[ActionClient] = action_clients_
            self._publishers: list[Publisher] = publishers_
            self._subscribers: list[Subscriber] = subscribers_
            self._services: list[Service] = services_
            self._clients: list[Client] = clients_
            self._host = host_

            self.visualizer_node_name = VISUALIZATION_NODE_PREFIX + uuid.uuid4().hex
            self.node_widget = NodeControlWidgetWrapper(
                self.visualizer_node_name, parent=self.view)
            self.add_custom_widget(self.node_widget, tab='Custom')
        except Exception as e:
            logging.error(f"Unable to initialize node. Error: {e}")
            exit(-1)

    @property
    def node_name(self):
        return self._node_name

    @node_name.setter
    def node_name(self, value):
        self._node_name = value

    @property
    def action_servers(self):
        return self._action_servers

    @action_servers.setter
    def action_servers(self, value):
        self._action_servers = value

    @property
    def action_clients(self):
        return self._action_clients

    @action_clients.setter
    def action_clients(self, value):
        self._action_clients = value

    @property
    def publishers(self):
        return self._publishers

    @publishers.setter
    def publishers(self, value):
        self._publishers = value

    @property
    def subscribers(self):
        return self._subscribers

    @subscribers.setter
    def subscribers(self, value):
        self._subscribers = value

        # Update visualization
        for subscriber in value:
            if not self.node_widget.has_visualizer(subscriber.topic_type, subscriber.topic_name) and self.node_widget.support_visualizer(subscriber.topic_type):
                logging.debug(
                    f"Added visualization for topic with type {subscriber.topic_type} and name {subscriber.topic_name}")
                self.node_widget.add_visualization(
                    subscriber.topic_type, subscriber.topic_name)
            elif self.node_widget.has_visualizer(subscriber.topic_type, subscriber.topic_name):
                logging.warning(
                    f"Visualizer for topic type {subscriber.topic_type} with name {subscriber.topic_name} has already been added")
            else:
                logging.warning(
                    f"Not supporting type for visualizing: {subscriber.topic_type}")

    @property
    def services(self):
        return self._services

    @services.setter
    def services(self, value):
        self._services = value

    @property
    def clients(self):
        return self._clients

    @clients.setter
    def clients(self, value):
        self._clients = value

    @property
    def host(self):
        return self._host

    @host.setter
    def host(self, value):
        self._host = value

    def has_publisher(self, topic_name) -> bool:
        return len([p for p in self._publishers if p.topic_name == topic_name]) != 0

    def has_subscription(self, topic_name) -> bool:
        return len([s for s in self._subscribers if s.topic_name == topic_name]) != 0

    def get_publisher(self, topic_name) -> Publisher:
        publisher = [
            publisher for publisher in self._publishers if publisher.topic_name == topic_name]
        assert (len(publisher) == 1 or len(publisher) == 0)
        return None if len(publisher) == 0 else publisher[0]

    def get_subscription(self, topic_name) -> Subscriber:
        subscription = [
            subscriber for subscriber in self._subscribers if subscriber.topic_name == topic_name]
        assert (len(subscription) == 1 or len(subscription) == 0)
        return None if len(subscription) == 0 else subscription[0]

    def pure_name(self):
        node_name = self._node_name
        if node_name.startswith('/'):
            node_name = node_name[1:]
        return node_name

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
        return hashlib.sha256(bytes(self._node_name, 'utf-8')).hexdigest()
