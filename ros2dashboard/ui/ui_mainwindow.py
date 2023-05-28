# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainwindow.ui'
##
## Created by: Qt User Interface Compiler version 5.15.8
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *  # type: ignore
from PySide2.QtGui import *  # type: ignore
from PySide2.QtWidgets import *  # type: ignore


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1085, 674)
        self.actionOpen_workflow = QAction(MainWindow)
        self.actionOpen_workflow.setObjectName(u"actionOpen_workflow")
        self.actionSave_workflow = QAction(MainWindow)
        self.actionSave_workflow.setObjectName(u"actionSave_workflow")
        self.actionAbout = QAction(MainWindow)
        self.actionAbout.setObjectName(u"actionAbout")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        sizePolicy = QSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.mainLayout = QHBoxLayout()
        self.mainLayout.setObjectName(u"mainLayout")
        self.mainLayout.setSizeConstraint(QLayout.SetMaximumSize)
        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")

        self.verticalLayout_2.addWidget(self.label)

        self.networkObserver = QListWidget(self.centralwidget)
        self.networkObserver.setObjectName(u"networkObserver")
        sizePolicy1 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.networkObserver.sizePolicy().hasHeightForWidth())
        self.networkObserver.setSizePolicy(sizePolicy1)

        self.verticalLayout_2.addWidget(self.networkObserver)


        self.mainLayout.addLayout(self.verticalLayout_2)

        self.dummy_dashboard = QWidget(self.centralwidget)
        self.dummy_dashboard.setObjectName(u"dummy_dashboard")
        sizePolicy.setHeightForWidth(self.dummy_dashboard.sizePolicy().hasHeightForWidth())
        self.dummy_dashboard.setSizePolicy(sizePolicy)

        self.mainLayout.addWidget(self.dummy_dashboard)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")

        self.verticalLayout_3.addWidget(self.label_2)

        self.packageObserver = QListWidget(self.centralwidget)
        self.packageObserver.setObjectName(u"packageObserver")

        self.verticalLayout_3.addWidget(self.packageObserver)


        self.mainLayout.addLayout(self.verticalLayout_3)


        self.verticalLayout.addLayout(self.mainLayout)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1085, 19))
        self.menuFile = QMenu(self.menubar)
        self.menuFile.setObjectName(u"menuFile")
        self.menuAbout = QMenu(self.menubar)
        self.menuAbout.setObjectName(u"menuAbout")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuAbout.menuAction())
        self.menuFile.addAction(self.actionOpen_workflow)
        self.menuFile.addAction(self.actionSave_workflow)
        self.menuAbout.addSeparator()
        self.menuAbout.addAction(self.actionAbout)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Ros2 Dashboard", None))
        self.actionOpen_workflow.setText(QCoreApplication.translate("MainWindow", u"Open workflow", None))
        self.actionSave_workflow.setText(QCoreApplication.translate("MainWindow", u"Save workflow", None))
        self.actionAbout.setText(QCoreApplication.translate("MainWindow", u"About", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Node explorer", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Package explorer", None))
        self.menuFile.setTitle(QCoreApplication.translate("MainWindow", u"File", None))
        self.menuAbout.setTitle(QCoreApplication.translate("MainWindow", u"Help", None))
    # retranslateUi

