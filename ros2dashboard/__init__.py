import os

from PySide2.QtGui import QGuiApplication
from PySide2.QtWidgets import QApplication
from NodeGraphQt import NodeGraph, BaseNode

os.environ["ROS_DOMAIN_ID"] = str(1)