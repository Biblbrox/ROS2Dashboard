import sys
import configparser
from PySide2.QtGui import QGuiApplication
import PySide2.QtGui as QtGui
from PySide2.QtWidgets import QApplication
from PySide2 import QtWidgets
from PySide2.QtCore import QFile
from NodeGraphQt import NodeGraph, BaseNode
from ros2dashboard.ui.MainWindow import MainWindow
import rc

def main(args=None):
    app = QApplication(sys.argv)
    app.setApplicationName("ROS2 Dashboard")
    
    styleFile = QFile(":/styles/app.qss")
    styleFile.open(QFile.ReadOnly)
    app.setStyleSheet(str(styleFile.readAll()))


    # Init config
    config = configparser.ConfigParser()
    config.read('config.ini')

    widget = MainWindow(args=args)
    #print(widget.mainDashboard)
    #widget.mainDashboard = dashboard_widget
    widget.show()

    # ros2_dashboard.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()