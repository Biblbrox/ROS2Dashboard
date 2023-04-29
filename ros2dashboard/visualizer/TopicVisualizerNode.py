import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PySide2.QtWidgets import QVBoxLayout, QWidget

from ros2dashboard.app.logger import logging
from ros2dashboard.visualizer.VideoVisualizer import VideoVisualizer
from ros2dashboard.ros2utils.SpinThread import SpinThread


class TopicVisualizerNode(Node):

    def __init__(self, node_name: str) -> None:
        assert (not node_name.startswith('/'))
        super().__init__(node_name=node_name)
        assert rclpy.ok()


class TopicVisualizer:
    def __init__(self, node_name: str, parent=None) -> None:
        assert (not node_name.startswith('/'))
        assert rclpy.ok()
        self.node_name = node_name
        self.node = None
        self.visualizers = {
            "sensor_msgs/msg/Image": VideoVisualizer(parent=parent)
        }

        self.visualizer_types = {
            "sensor_msgs/msg/Image": Image
        }

        self.visualizer_subscriptions = {

        }

        layout = QVBoxLayout(parent)
        for _, value in self.visualizers.items():
            layout.addWidget(value.widget)
        self.widget = QWidget(parent)
        self.widget.setLayout(layout)
        self.spin_executor = SpinThread()
        # self.widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)

    def init_node(self):
        self.node = TopicVisualizerNode(self.node_name)
        self.spin_executor.add_node_execution(self.node)

    def has_visualizer(self, topic_name, topic_type):
        return topic_type in self.visualizer_subscriptions

    def support_visualizer(self, topic_type):
        return topic_type in self.visualizers

    def subscribe_topic(self, topic_name, topic_type):
        if self.node is None:
            self.init_node()
        if topic_type not in self.visualizers.keys():
            logging.error(
                f"Unsupported messge type for visualization: {topic_type}")
            return

        if topic_type in self.visualizer_subscriptions:
            self.visualizer_subscriptions[topic_type].destroy()

        self.visualizer_subscriptions[topic_type] = self.node.create_subscription(
            self.visualizer_types[topic_type], topic_name,
            lambda msg: self.visualizers[topic_type].update_widget(msg), 10)

    def unsibscribe_all(self):
        if self.node is None:
            self.init_node()
        for _, subscribtion in self.visualizer_subscriptions:
            subscribtion.destroy()
