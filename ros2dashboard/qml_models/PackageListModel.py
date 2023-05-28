from typing import Dict
from PySide2.QtCore import QAbstractListModel, QModelIndex, Qt

from ros2dashboard.edge.Package import Package


class PackageListModel(QAbstractListModel):
    def __init__(self, packages: list[Package], parent=None):
        super().__init__(parent)
        self._packages: list[Package] = packages
        self._roles = {
            'name': Qt.DisplayRole,
            'executable_names': Qt.DisplayRole + 1
        }

    @property
    def packages(self):
        return self._packages

    @packages.setter
    def packages(self, new_packages):
        self._packages = new_packages

    def rowCount(self, parent=QModelIndex()):
        return len(self._packages)

    def data(self, index, role=Qt.DisplayRole):
        row = index.row()

        if role == self._roles['name']:
            # Return the data to be displayed
            return self._packages[row].name

        elif role == self._roles['executable_names']:
            return self._packages[row].executable_names

        return None

    def roleNames(self) -> Dict:
        return self._roles
