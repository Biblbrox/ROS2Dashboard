from PySide2.QtWidgets import QWidget, QLabel, QVBoxLayout
from PySide2.QtGui import QImage, QPixmap
from PySide2.QtCore import QUrl, QThread, QObject, Signal
from sensor_msgs.msg import Image

from ros2dashboard.visualizer.Visualizer import Visualizer
from ros2dashboard.core.Logger import logging


class VideoVisualizer(Visualizer, QObject):
    def __init__(self, node_name: str, parent=None):
        super().__init__(node_name)

        self.image_label = QLabel(parent)
        self.width = 512
        self.height = 288
        self.qml_path = "ros2dashboard/visualizer/video/VideoVisualizer.qml"

    def init(self):
        super().init()
        self.widget.setSource(QUrl.fromLocalFile(self.qml_path))


    def update_widget(self, data: Image):
        # We need to do nothing because qml file contains all logic stuff.
        pass