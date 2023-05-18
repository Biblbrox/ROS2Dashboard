#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

pyside2-rcc $SCRIPT_DIR/../res.qrc > $SCRIPT_DIR/../rc.py
pyside2-uic $SCRIPT_DIR/../ros2dashboard/ui/mainwindow.ui > $SCRIPT_DIR/../ros2dashboard/ui//ui_mainwindow.py