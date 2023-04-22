from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from PySide2.QtMultimediaWidgets import QVideoWidget
from PySide2.QtMultimedia import QMediaPlayer, QVideoFrame, QMediaContent
from PySide2.QtGui import QImage

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
        self.widget = None
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
            self.widget = image
        except Exception as e:
            logging.error(f"Unable to update VideoVisualizer. Error: {e}")


class TopicVisualizer(Node):
    def __init__(self, node_name: str) -> None:
        assert(not node_name.startswith('/'))
        super().__init__(node_name=node_name)
        assert rclpy.ok()

        self.visualizers = {
            Image: VideoVisualizer()
        }

        self.visualizer_subscriptions = {

        }

    def subscribe_topic(self, topic_name, topic_type):
        if topic_type not in self.visualizers.keys():
            return

        if self.visualizer_subscriptions[topic_type] is not None:
            self.visualizer_subscriptions[topic_type].destroy()

        self.visualizer_subscriptions[topic_type] = self.create_subscription(
            topic_type, topic_name, self.visualizers[topic_type], 10)

    def unsibscribe_all(self):
        for _, subscribtion in self.visualizer_subscriptions:
            subscribtion.destroy()
