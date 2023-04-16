# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'nodewidget.ui'
##
## Created by: Qt User Interface Compiler version 5.15.8
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *  # type: ignore
from PySide2.QtGui import *  # type: ignore
from PySide2.QtWidgets import *  # type: ignore


class Ui_NodeWidget(object):
    def setupUi(self, NodeWidget):
        if not NodeWidget.objectName():
            NodeWidget.setObjectName(u"NodeWidget")
        NodeWidget.resize(400, 300)
        self.verticalLayout_2 = QVBoxLayout(NodeWidget)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label = QLabel(NodeWidget)
        self.label.setObjectName(u"label")

        self.horizontalLayout.addWidget(self.label)

        self.node_name = QLabel(NodeWidget)
        self.node_name.setObjectName(u"node_name")

        self.horizontalLayout.addWidget(self.node_name)


        self.verticalLayout.addLayout(self.horizontalLayout)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.label_2 = QLabel(NodeWidget)
        self.label_2.setObjectName(u"label_2")

        self.horizontalLayout_2.addWidget(self.label_2)

        self.node_host = QLabel(NodeWidget)
        self.node_host.setObjectName(u"node_host")

        self.horizontalLayout_2.addWidget(self.node_host)


        self.verticalLayout.addLayout(self.horizontalLayout_2)


        self.verticalLayout_2.addLayout(self.verticalLayout)


        self.retranslateUi(NodeWidget)

        QMetaObject.connectSlotsByName(NodeWidget)
    # setupUi

    def retranslateUi(self, NodeWidget):
        NodeWidget.setWindowTitle(QCoreApplication.translate("NodeWidget", u"Form", None))
        self.label.setText(QCoreApplication.translate("NodeWidget", u"Node name:", None))
        self.node_name.setText(QCoreApplication.translate("NodeWidget", u"TextLabelsf", None))
        self.label_2.setText(QCoreApplication.translate("NodeWidget", u"Node host:", None))
        self.node_host.setText(QCoreApplication.translate("NodeWidget", u"TextLabel", None))
    # retranslateUi

