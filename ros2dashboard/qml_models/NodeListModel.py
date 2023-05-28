import os.path as path
from PySide2.QtCore import Qt, QAbstractListModel, QModelIndex, QObject, Property, Slot
from PySide2.QtGui import QStandardItem, QStandardItemModel
from PySide2.QtQml import QQmlApplicationEngine, qmlRegisterType
from PySide2.QtWidgets import QApplication

import ros2dashboard.devices.GenericNode as GenericNode
from ros2dashboard.core.Logger import logging


class NodeListModel(QAbstractListModel):
    def __init__(self, parent):
        super().__init__(parent)

    def data(self, index, role):
        if not self.is_index_valid(index):
            return None

        if (role == Qt.DisplayRole) or (role == Qt.EditRole):
            item: ConfigItem = self._config.get_ith_item(index.row())
            if index.column() == 0:
                return item.name
            elif index.column() == 1:
                return item.type
            else:
                return item.value

        return None

    def is_index_valid(self, index) -> bool:
        if not index.isValid():
            return None

        if not 0 <= index.row() < len(self._config.config.keys()):
            return None

        return True

    def addNode(self, node: GenericNode):
        pass

    def removeNode(self, node: GenericNode):
        pass

    def setData(self, index, value, role) -> bool:
        if not self.is_index_valid(index):
            return False

        logging.debug(f"Editing {index.row()} {index.column()}")
        item: ConfigItem = self._config.get_ith_item(index.row())
        if index.column() == 0:
            item.name = value
        elif index.column() == 1:
            item.type = value
        else:
            item.value = value

        # self.dataChanged.emit()
        return True

    def flags(self, index: QModelIndex):
        return Qt.ItemIsEditable | Qt.ItemIsSelectable | QAbstractTableModel.flags(index)

    def rowCount(self, parent=QModelIndex()):
        return len(self._config.config.keys()) if self._config else 0

    def columnCount(self, parent=QModelIndex()):
        return 3

    def item_by_idx(self, idx) -> ConfigItem:
        return self._config.config[list(self._config.config.keys())[idx]]

    @Slot(int, result=str)
    def generateRichDescription(self, idx):
        field_description = self.fieldDescription(idx)
        group_description = self.groupDescription(idx)
        item = self.item_by_idx(idx)

        html = f"""<center><h1>{item.name}</h1></center>
        <b>Описание группы:</b> {group_description}
        <br>
        <b>Описание параметра</b> {field_description}
        """

        return html

    @Slot(int, result=str)
    def fieldDescription(self, idx):
        keys = list(self._field_description.keys())

        if idx < 0 or idx >= len(keys):
            return ""

        return self._field_description[keys[idx]]

    @Slot(str, result=str)
    def getVal(self, key: str) -> str:
        return self._config.config[key]

    @Slot(int, result=str)
    def groupDescription(self, idx):
        keys = list(self._group_description.keys())

        if idx < 0 or idx >= len(keys):
            return ""

        return self._group_description[keys[idx]]

    @Slot()
    def loadConfig(self):
        config_path: str = self._jetson_model.config_path.strip()
        self.beginInsertRows(QModelIndex(), 0, 0)
        if self._jetson_model.is_network_device:
            credentials = self._jetson_model.credentials
            fs = SSHFileSystem(self._jetson_model.url,
                               username=credentials["username"],
                               password=credentials["password"]
                               )

            if config_path.startswith("~"):
                config_path = path.expanduser(config_path)
            if not fs.isfile(config_path):
                logging.error(f"File {config_path} doesn't exists")
                return

            with fs.open(config_path) as stream:
                config_str = stream.read()
                self._config = PrometeiConfig()
                self._config.load_from_str(config_str.decode("utf-8"))
        else:
            self._config = PrometeiConfig(self._jetson_model.config_path)

        self.endInsertRows()

    @Slot()
    def saveConfig(self):
        if not self._config:
            logging.warn("Config was not loaded in ConfigModel. Doing nothing")
            return

        # Path in jetson
        config_path = self._jetson_model.config_path
        if self._jetson_model.is_network_device:
            # If network url contains only ip
            credentials = self._jetson_model.credentials
            self.fs = SSHFileSystem(self._jetson_model.url,
                                    username=credentials["username"],
                                    password=credentials["password"]
                                    )
            self.fs.write_bytes(config_path, str(self._config).encode())
        else:
            self._config.save(config_path)

    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                if section == 0:
                    return "Name"
                elif section == 1:
                    return "Type"
                elif section == 2:
                    return "Value"
        return None
