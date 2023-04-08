from PySide2.QtGui import QGuiApplication
from PySide2.QtWidgets import QApplication
from PySide2 import QtWidgets
from NodeGraphQt import NodeGraph, BaseNode


class CameraNode(BaseNode):

    # unique node identifier domain.
    __identifier__ = 'ros2dashboard.devices.CameraNode'

    # initial default node name.
    NODE_NAME = 'Camera node'

    def __init__(self):
        super(CameraNode, self).__init__()

        # create an input port.
        self.add_input('in', color=(180, 80, 0))

        # create an output port.
        self.add_output('out')
