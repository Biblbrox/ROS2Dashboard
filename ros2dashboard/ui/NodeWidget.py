from PySide2.QtWidgets import QWidget

from ros2dashboard.app.Ros2Dashboard import Ros2Dashboard as Ros2Dashboard
from ros2dashboard.ui.ui_nodewidget import Ui_NodeWidget
from ros2dashboard.devices.Ros2Node import Ros2Node


class NodeWidget(QWidget):
    def __init__(self, node: Ros2Node, parent=None):
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