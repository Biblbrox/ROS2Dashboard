import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtQuick.Shapes 1.15

Rectangle {
    id: rect

    property int sceneWidth
    property alias color: rect.color
    property alias name: nodeLabel.text

    radius: 10
    width: width
    height: max(outputPorts.height, inputPorts.height, height)

    ColumnLayout {
        anchors.fill: parent

        Label {
            id: nodeLabel

            width: parent.width
            Layout.rightMargin: 20
            Layout.leftMargin: 20
            Layout.fillWidth: true
        }

    }

    ColumnLayout {
        id: inputPorts

        anchors {
            left: parent.left
            top: parent.top
            bottom: parent.bottom
        }

        Port {
        }

    }

    ColumnLayout {
        id: outputPorts

        anchors {
            right: parent.right
            top: parent.top
            bottom: parent.bottom
        }

        Port {
        }

        Port {
        }

        Port {
        }

    }

    MouseArea {
        hoverEnabled: true
        drag.target: rect
        drag.minimumX: 0
        drag.maximumX: sceneWidth - rect.width

        anchors {
            left: inputPorts.right
            right: outputPorts.left
            top: parent.top
            bottom: parent.bottom
        }

    }

}
