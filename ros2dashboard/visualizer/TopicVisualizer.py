from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from PySide2.QtMultimediaWidgets import QVideoWidget
from PySide2.QtMultimedia import QMediaPlayer, QVideoFrame, QMediaContent
from PySide2.QtGui import QImage, QPixmap
from PySide2.QtWidgets import QVBoxLayout, QLabel, QWidget, QSizePolicy

from ros2dashboard.edge import Publisher, Subscriber, Topic
from ros2dashboard.app.logger import logging


class Visualizer(ABC):
    def __init__(self) -> None:
        super().__init__()
        self.widget = None
        self.is_active = False

    @abstractmethod
    def update_widget(self, data) -> None:
        pass


class VideoVisualizer(Visualizer):
    def __init__(self, parent=None) -> None:
        super().__init__()
        # self.media_player = QMediaPlayer(self)
        placeholder = QPixmap(
            "ros2dashboard/content/placeholder_image.png")
        placeholder = placeholder.scaledToHeight(288)
        placeholder = placeholder.scaledToWidth(512)
        image_label = QLabel()
        image_label.setPixmap(placeholder)
        image_layout = QVBoxLayout()
        image_layout.addWidget(image_label)
        self.widget = QWidget(parent=parent)
        self.widget.setLayout(image_layout)
        # self.media_player.setVideoOutput(self.widget)
        # self.media_player.setMedia()
        self.img_format_table = {
            'rgb8': QImage.Format_RGB888, 'mono8': QImage.Format_Mono}
        # self.widget.show()

    def __call__(self, msg):
        print("SHow")

    def update_widget(self, data: Image):
        try:
            # format = self.img_format_table[data.encoding]
            image = QImage(data.data, data.width, data.height, format)
            self.widget = QLabel()
            self.widget.setPixmap(QPixmap(image))
        except Exception as e:
            logging.error(f"Unable to update VideoVisualizer. Error: {e}")


class TopicVisualizer(Node):
    def __init__(self, node_name: str, parent=None) -> None:
        assert (not node_name.startswith('/'))
        super().__init__(node_name=node_name)
        assert rclpy.ok()

        self.visualizers = {
            "sensor_msgs/msg/Image": VideoVisualizer(parent=parent)
        }
        self.visualizer_types = {
            "sensor_msgs/msg/Image": Image
        }

        self.visualizer_subscriptions = {

        }

        layout = QVBoxLayout()
        for key, value in self.visualizers.items():
            layout.addWidget(value.widget)
        self.widget = QWidget()
        self.widget.setLayout(layout)
        self.widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)

    def has_visualizer(self, topic_name, topic_type):
        return topic_type in self.visualizer_subscriptions

    def support_visualizer(self, topic_type):
        return topic_type in self.visualizers

    def subscribe_topic(self, topic_name, topic_type):
        if topic_type not in self.visualizers.keys():
            logging.error(
                f"Unsupported messge type for visualization: {topic_type}")
            return

        if topic_type in self.visualizer_subscriptions:
            self.visualizer_subscriptions[topic_type].destroy()

        self.visualizer_subscriptions[topic_type] = self.create_subscription(
            self.visualizer_types[topic_type], topic_name, self.visualizers[topic_type], 10)

    def unsibscribe_all(self):
        for _, subscribtion in self.visualizer_subscriptions:
            subscribtion.destroy()
