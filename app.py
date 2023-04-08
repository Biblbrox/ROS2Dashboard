import sys
import configparser

from PySide2.QtGui import QGuiApplication
from PySide2.QtWidgets import QApplication
from PySide2 import QtWidgets
from PySide2.QtCore import QFile
from NodeGraphQt import NodeGraph, BaseNode
import rc

from ros2dashboard.app.Ros2Dashboard import Ros2Dashboard as Ros2Dashboard


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("ROS2 Dashboard")
    
    styleFile = QFile(":/styles/app.qss")
    styleFile.open(QFile.ReadOnly)
    app.setStyleSheet(str(styleFile.readAll()))


    # Init config
    config = configparser.ConfigParser()
    config.read('config.ini')

    ros2_dashboard = Ros2Dashboard()
    ros2_dashboard.show()

    app.exec_()


if __name__ == '__main__':
    main()