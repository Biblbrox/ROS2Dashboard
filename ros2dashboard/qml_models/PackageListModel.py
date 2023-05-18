from typing import Dict
from PySide6.QtCore import QAbstractListModel, QModelIndex, Qt
from PySide6.QtGui import QFont, QColor


class PackageModel(QAbstractListModel):
    def __init__(self, packages=None, parent=None):
        super().__init__(parent)
        self._packages = packages
        self._roles = {
            'name': Qt.DisplayRole,
            'nodes': Qt.DisplayRole + 1
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
            return self._packages[row]

        elif role == self._roles['nodes']:
            return None

        return None
    
    def roleNames(self) -> Dict:
        return self._roles
    
