from PySide2.QtWidgets import QMainWindow, QListWidgetItem
from PySide2 import QtCore

from ros2dashboard.app.Ros2Dashboard import Ros2Dashboard as Ros2Dashboard
from ros2dashboard.ui.ui_mainwindow import Ui_MainWindow
from ros2dashboard.ui.NodeWidget import NodeWidget
from ros2dashboard.devices.Ros2Node import Ros2Node
from ros2dashboard.ros2utils.Ros2Monitor import Ros2Monitor
from ros2dashboard.app.logger import logging


class MainWindow(QMainWindow):
    def __init__(self, args, parent=None):
        super(MainWindow, self).__init__(parent)
        self.args = args
        self.load_ui()

    def load_ui(self):
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ros2_dashboard = Ros2Dashboard()
        self.ros2monitor = Ros2Monitor(self.args)
        self.dashboard_widget = self.ros2_dashboard.graph_widget
        self.ui.mainLayout.replaceWidget(self.ui.dummy_dashboard, self.dashboard_widget)

        self.init_network()
        self.init_slots()

    def init_slots(self):
        self.ros2monitor.new_nodes.connect(self.update_nodes, type=QtCore.Qt.BlockingQueuedConnection)
        self.ros2monitor.new_nodes.connect(self.ros2_dashboard.update_nodes, type=QtCore.Qt.BlockingQueuedConnection)
        self.ros2monitor.new_subscribers.connect(self.ros2_dashboard.update_subscribers, type=QtCore.Qt.BlockingQueuedConnection)
        self.ros2monitor.new_publishers.connect(self.ros2_dashboard.update_publishers, type=QtCore.Qt.BlockingQueuedConnection)

    def init_network(self):
        self.thread = QtCore.QThread(self)
        self.ros2monitor.moveToThread(self.thread)
        self.thread.started.connect(self.ros2monitor.start)
        self.thread.start()

    
    @QtCore.Slot(object)
    def update_nodes(self, nodes: list[Ros2Node]):    
        logging.debug("New nodes found")

        self.ui.networkObserver.clear()
        for node in nodes:
            item = QListWidgetItem()
            widget = NodeWidget(node, parent=self)
            item.setSizeHint(widget.sizeHint())
            self.ui.networkObserver.addItem(item)
            self.ui.networkObserver.setItemWidget(item, widget)

    #new_topics = Signal(list[Topic])
    #new_hosts = Signal(list[Host])
    #new_subscribers = Signal(list[Subscriber])
    #new_publishers = Signal(list[Publisher])
    #new_services = Signal(list[Service])
    #new_clients = Signal(list[Client])