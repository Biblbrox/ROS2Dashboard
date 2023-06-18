import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.3

Rectangle {
    id: nodeObserver

    visible: true
    width: 800
    height: 500
    color: "#2f2e40"

    ColumnLayout {
        spacing: 40
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.leftMargin: 20
        anchors.topMargin: 20
        anchors.rightMargin: 20

        Label {
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            text: "Node obserber"
            font.pixelSize: 18
            color: "white"
        }

        ToolSeparator {
            orientation: Qt.Horizontal
            Layout.fillWidth: true
        }

        ListView {
            id: nodeListView
            Layout.fillWidth: true
            width: parent.width
            //height: 100

            model: nodeListModel

            delegate: Rectangle {
                width: parent.width

                color: "white"

                Label {
                    id: label

                    height: parent.height
                    anchors.fill: parent
                    //anchors.right: parent.right
                    //horizontalAlignment: Text.AlignHCenter
                    //verticalAlignment: Text.AlignVCenter
                    font.pointSize: 21
                    //renderType: Text.QtRendering
                    //anchors.left: parent.left
                    color: "white"
                    text: name
                }

            }

        }


    }

}
