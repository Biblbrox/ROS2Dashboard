import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.3

Rectangle {
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
            text: "Package obserber"
            font.pixelSize: 18
        }

        ListView {
            //model: packageModel

            id: packageListModel

            delegate: Rectangle {
                Label {
                    id: label

                    x: 0
                    y: 0
                    //width: 578
                    height: 100
                    anchors.right: parent.right
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 21
                    renderType: Text.QtRendering
                    anchors.left: parent.left
                    color: "#000000"
                    text: packageModel.name
                }

            }

        }

    }

}
