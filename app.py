import sys
from os.path import exists
from PySide2.QtWidgets import QApplication
from PySide2.QtCore import QFile
from PySide2.QtGui import QGuiApplication
from PySide2.QtQml import QQmlApplicationEngine
from ros2dashboard.ui.MainWindow import MainWindow
from ros2dashboard.core.Logger import logging
from ros2dashboard.core.Config import Config
from ros2dashboard.qml_models import PackageListModel
import rc


def init_qt_app(args, is_debug, is_debug_view):
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

    return app, widget


def init_qml_app(args, is_debug, is_debug_view):
    app = QGuiApplication(sys.argv)
    # app.setOrganizationName(config['org_name'])
    # app.setOrganizationDomain(config['org_domain'])

    engine = QQmlApplicationEngine(app)
    engine.quit.connect(app.quit)

    # import_paths = [
    #    ".",
    #    "trainHelper/",
    #    "trainHelper/bridges",
    #    "trainHelper/models",
    #    "trainHelper/core",
    #    "trainHelper/qml",
    #    "trainHelper/qml/Reusable",
    #    "trainHelper/",
    #    "trainHelper/models",
    #    "trainHelper/core",
    #    "trainHelper/qml",
    #    "trainHelper/qml/Reusable",
    #    "trainHelper/bridges",
    #    ".."
    # ]
    # for import_path in import_paths:
    #    engine.addImportPath(import_path)

    context = engine.rootContext()

    # context.setContextProperty(
    #    "applicationDirPath", os.fspath(CURRENT_DIRECTORY))

    # imageProvider = IconProvider()
    # engine.addImageProvider("imageProvider", imageProvider)

    # load_models(context, app, config)
    engine.load('ros2dashboard/qml/Main.qml')

    return app, None


def main(args=None):
    try:
        config_path = "config/config.ini"
        has_config = True
        if not exists(config_path):
            logging.warning(
                "Unable to find config. Staying with default values")
            has_config = False

        config = Config(config_path)
        config.load()
    

        is_debug = config.get_value(
            'Core', 'DebugMode', expected_type=bool) if has_config else True
        is_debug_view = config.get_value(
            'Core', 'DebugView', expected_type=bool) if has_config else True

        qml_app = False
        if qml_app:
            app, _ = init_qml_app(args, is_debug, is_debug_view)
        else:
            app, _ = init_qt_app(args, is_debug, is_debug_view)

        sys.exit(app.exec_())
    except Exception as e:
        logging.error(f"Unhanled error: {e}")
        exit(-1)


if __name__ == '__main__':
    main()
