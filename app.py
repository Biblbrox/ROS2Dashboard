import sys
import configparser
from PySide2.QtWidgets import QApplication
from PySide2.QtCore import QFile
from ros2dashboard.ui.MainWindow import MainWindow
from ros2dashboard.app.logger import logging
import rc

def main(args=None):
    try:
        app = QApplication(sys.argv)
        app.setApplicationName("ROS2 Dashboard")
        
        styleFile = QFile(":/styles/styles/app.qss")
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
    except Exception as e:
        logging.error(f"Unhanled error: {e}")
        exit(-1)


if __name__ == '__main__':
    main()