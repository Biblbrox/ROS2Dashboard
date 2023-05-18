from PySide2.QtWidgets import QWidget

from ros2dashboard.core.DashboardApp import DashboardApp as DashboardApp
from ros2dashboard.ui.ui_nodewidget import Ui_NodeWidget
from ros2dashboard.devices.GenericNode import GenericNode


class NodeWidget(QWidget):
    def __init__(self, node: GenericNode, parent=None):
        super(NodeWidget, self).__init__(parent)
        self.node = node
        self.load_ui()

    def load_ui(self):
        self.ui = Ui_NodeWidget()
        self.ui.setupUi(self)
        self.ui.node_host.setText(self.node.host)
        self.ui.node_name.setText(self.node.node_name)

    def getText(self):
        return self.node.node_name
    
    def setText(self, text: str):
        self.ui.node_name.setText(text)