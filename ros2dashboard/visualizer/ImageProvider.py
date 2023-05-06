from os.path import exists
from pathlib import Path

from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QFileInfo
from PySide6.QtWidgets import QFileIconProvider
from PySide6.QtCore import QSize
from PySide6.QtGui import QImage
from PySide6.QtQuick import QQuickImageProvider

from ros2dashboard.core.Logger import logging


def get_icon(file_path) -> QImage:
    if not exists(file_path):
        logging.error(f"Wrong file path to extract icon: {file_path}")
        return QImage()

    icon_provider = QFileIconProvider()
    file_info = QFileInfo(file_path)
    file_icon = icon_provider.icon(file_info)
    image = file_icon.pixmap(32, 32).toImage()
    return image


class ImageProvider(QQuickImageProvider):
    def __init__(self):
        super().__init__(QQuickImageProvider.Image)

    def requestImage(self, filePath: str, size: QSize, requestedSize: QSize) -> QImage:
        image = QImage(filePath)

        if size is not None:
            size.setWidth(image.size().width())
            size.setHeight(image.size().height())

        return image