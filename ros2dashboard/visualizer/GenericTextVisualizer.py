from PySide2.QtWidgets import QWidget, QLabel, QVBoxLayout
from PySide2.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image

from ros2dashboard.visualizer.Visualizer import Visualizer
from ros2dashboard.app.logger import logging


class GenericTextVisualizer(Visualizer):
    """
    This uses generic text message representation generated with __repr__
    method.
    """
    def __init__(self, parent=None) -> None:
        super().__init__()
        # self.media_player = QMediaPlayer(self)
        self.text_label = QLabel(parent)
        self.widget = QWidget(parent=parent)
        layout = QVBoxLayout()
        layout.addWidget(self.text_label)
        self.widget.setLayout(layout)
        

    def update_widget(self, data):
        try:
            str_repr = data.__repr__()
            self.text_label.setText(str_repr)
            self.widget.update()
        
        except Exception as e:
            logging.error(f"Unable to update VideoVisualizer. Error: {e}")
