from PySide2.QtWidgets import QMainWindow, QListWidgetItem
from PySide2 import QtCore

from ros2dashboard.core.DashboardApp import DashboardApp as DashboardApp
from ros2dashboard.ui.ui_mainwindow import Ui_MainWindow
from ros2dashboard.ui.NodeWidget import NodeWidget
from ros2dashboard.devices.GenericNode import GenericNode
from ros2dashboard.ros2utils.Ros2Monitor import Ros2Monitor
from ros2dashboard.core.Logger import logging
from ros2dashboard.qml_models.PackageListModel import PackageListModel


class MainWindow(QMainWindow):
    def __init__(self, args, parent=None):
        super(MainWindow, self).__init__(parent)
        self.args = args
        self.load_ui()

    def load_ui(self):
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setup_models()

        self.ros2_dashboard = DashboardApp(self.package_model)
        self.ros2monitor = Ros2Monitor(self.args)
        self.dashboard_widget = self.ros2_dashboard.graph_widget
        self.ui.mainLayout.replaceWidget(
            self.ui.dummy_dashboard, self.dashboard_widget)

        self.init_network()
        self.init_slots()

    def setup_models(self):
        self.package_model = PackageListModel(None, parent=self.parent())
        self.ui.packageObserver.rootContext().setContextProperty(
            "packageModel", self.package_model)

    def init_slots(self):
        self.ros2monitor.new_nodes.connect(
            self.update_nodes, type=QtCore.Qt.BlockingQueuedConnection)
        self.ros2monitor.new_nodes.connect(
            self.ros2_dashboard.update_nodes, type=QtCore.Qt.BlockingQueuedConnection)
        self.ros2monitor.new_subscribers.connect(
            self.ros2_dashboard.update_subscribers, type=QtCore.Qt.BlockingQueuedConnection)
        self.ros2monitor.new_publishers.connect(
            self.ros2_dashboard.update_publishers, type=QtCore.Qt.BlockingQueuedConnection)
        self.ros2monitor.new_packages.connect(
            self.ros2_dashboard.update_packages, type=QtCore.Qt.BlockingQueuedConnection)
        self.ros2monitor.new_state.connect(
            self.ros2_dashboard.update_state, type=QtCore.Qt.BlockingQueuedConnection)

    def init_network(self):
        self.thread = QtCore.QThread(self)
        self.ros2monitor.moveToThread(self.thread)
        self.thread.started.connect(self.ros2monitor.start)
        self.thread.start()

    @QtCore.Slot(object)
    def update_nodes(self, nodes: list[GenericNode]):
        logging.debug("New nodes found")

        self.ui.networkObserver.clear()
        for node in nodes:
            item = QListWidgetItem()
            widget = NodeWidget(node, parent=self)
            item.setSizeHint(widget.sizeHint())
            self.ui.networkObserver.addItem(item)
            self.ui.networkObserver.setItemWidget(item, widget)

    # new_topics = Signal(list[Topic])
    # new_hosts = Signal(list[Host])
    # new_subscribers = Signal(list[Subscriber])
    # new_publishers = Signal(list[Publisher])
    # new_services = Signal(list[Service])
    # new_clients = Signal(list[Client])
