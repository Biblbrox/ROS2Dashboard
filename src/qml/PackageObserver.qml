import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.3

Rectangle {
    id: packageObserver

    color: "#2f2e40"
    height: 500
    visible: true
    width: 800

    ColumnLayout {
        anchors.left: parent.left
        anchors.leftMargin: 20
        anchors.right: parent.right
        anchors.rightMargin: 20
        anchors.top: parent.top
        anchors.topMargin: 20
        spacing: 40

        Label {
            id: packageObserverTitle

            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            color: "white"
            font.pixelSize: 18
            text: "Package observer"
        }
        ToolSeparator {
            id: packageObserverSeparator

            Layout.fillWidth: true
            orientation: Qt.Horizontal
        }
        ListView {
            id: nodeListView

            Layout.fillWidth: true
            clip: true
            height: packageObserver.height - packageObserverTitle.height - packageObserverSeparator.height
            model: packageListModel
            spacing: 20
            width: parent.width

            delegate: Rectangle {
                color: Theme.panel.color.elementBackground
                height: 100
                radius: 10
                width: ListView.width

                TextArea {
                    id: label

                    anchors.centerIn: parent
                    color: Theme.font.color.primary
                    font.pointSize: 18
                    readOnly: true
                    height: parent.height
                    horizontalAlignment: TextEdit.AlignHCenter
                    text: name
                    verticalAlignment: TextEdit.AlignVCenter
                    width: parent.width
                }
                MouseArea {
                    acceptedButtons: Qt.LeftButton
                    anchors.fill: parent

                    onClicked: {
                        console.debug("Clicked to package " + name);
                    }
                }
            }
        }
    }
}

