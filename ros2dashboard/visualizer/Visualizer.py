from abc import ABC, abstractmethod

from PySide2.QtQuickWidgets import QQuickWidget


class Visualizer:
    def __init__(self, node_name: str) -> None:
        super().__init__()
        self.node_name = node_name
        self.is_active = False

    def init(self):
        # We need to do all initialization stuff in init method instead of constructor
        self.widget = QQuickWidget()
        self.widget.rootContext().setContextProperty('nodeName', self.node_name)
        self.widget.engine().addImportPath("/usr/local/share/")

    @abstractmethod
    def update_widget(self, data) -> None:
        pass

