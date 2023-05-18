import sys
from os.path import exists
from PySide2.QtWidgets import QApplication
from PySide2.QtCore import QFile
from ros2dashboard.ui.MainWindow import MainWindow
from ros2dashboard.core.Logger import logging
from ros2dashboard.core.Config import Config
from ros2dashboard.qml_models import PackageListModel
import rc


def main(args=None):
    try:
        config_path = "config/config.ini"
        has_config = True
        if not exists(config_path):
            logging.warning("Unable to find config. Staying with default values")
            has_config = False
        
        config = Config(config_path)
        config.load()

        is_debug = config.get_value('Core', 'DebugMode', expected_type=bool) if has_config else True
        is_debug_view = config.get_value('Core', 'DebugView', expected_type=bool) if has_config else True

        app = QApplication(sys.argv)
        app.setApplicationName("ROS2 Dashboard")

        styleFile = QFile(":/styles/styles/app.qss")
        styleFile.open(QFile.ReadOnly)
        # app.setStyleSheet(str(styleFile.readAll()))
        
        if is_debug and is_debug_view:
            app.setStyleSheet("""
            * {
                border: 1px solid red;
            }
        """)

        widget = MainWindow(args=args)
    
        widget.show()

        sys.exit(app.exec_())
    except Exception as e:
        logging.error(f"Unhanled error: {e}")
        exit(-1)


if __name__ == '__main__':
    main()
