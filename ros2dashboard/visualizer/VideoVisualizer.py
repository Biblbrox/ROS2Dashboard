from PySide2.QtWidgets import QWidget, QLabel, QVBoxLayout
from PySide2.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image

from ros2dashboard.visualizer.Visualizer import Visualizer
from ros2dashboard.app.logger import logging


class VideoVisualizer(Visualizer):
    def __init__(self, parent=None) -> None:
        super().__init__()
        # self.media_player = QMediaPlayer(self)
        self.image_label = QLabel(parent)
        self.widget = QWidget(parent=parent)
        self.width = 512
        self.height = 288

        placeholder_image = "ros2dashboard/content/placeholder_image.png"
        placeholder = QPixmap(placeholder_image)

        self.set_frame(placeholder)

        image_layout = QVBoxLayout(parent)
        image_layout.addWidget(self.image_label)
        self.widget.setLayout(image_layout)
        # self.media_player.setVideoOutput(self.widget)
        # self.media_player.setMedia()
        self.img_format_table = {
            'rgb8': QImage.Format_RGB888, 'mono8': QImage.Format_Mono, 'bgr8': QImage.Format_BGR888}
        # self.widget.show()

    def set_frame(self, frame: QPixmap):
        frame = frame.scaledToHeight(self.height)
        frame = frame.scaledToWidth(self.width)
        self.image_label.setPixmap(frame)
        self.widget.update()
        # self.media_player.setVideoOutput(self.widget)
        # self.media_player.setMedia()

    def update_widget(self, data: Image):
        try:
            format = self.img_format_table[data.encoding]
            image = QImage(data.data, data.width, data.height, format)
            pixmap = QPixmap(image)
            self.set_frame(pixmap)
        except Exception as e:
            logging.error(f"Unable to update VideoVisualizer. Error: {e}")
