import os

from PySide2.QtGui import QGuiApplication
from PySide2.QtWidgets import QApplication
from NodeGraphQt import NodeGraph, BaseNode
from ros2dashboard.app.logger import logging

os.environ["ROS_DOMAIN_ID"] = str(1)

logging.setLevel(level='DEBUG')

