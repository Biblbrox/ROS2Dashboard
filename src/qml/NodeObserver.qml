import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.3

Rectangle {
    id: nodeObserver

    color: Theme.panel.color.background
    visible: true
    width: 800

    ColumnLayout {
        anchors.left: parent.left
        anchors.leftMargin: 20
        anchors.right: parent.right
        anchors.rightMargin: 20
        anchors.top: parent.top
        anchors.topMargin: 20
        anchors.bottom: parent.bottom
        spacing: 40

        Label {
            id: nodeObserverTitle

            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            color: "white"
            font.pixelSize: 18
            text: "Node observer"
        }
        ToolSeparator {
            id: nodeObserverSeparator

            Layout.fillWidth: true
            orientation: Qt.Horizontal
        }
        ListView {
            id: nodeListView

            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            height: nodeObserver.height - nodeObserverTitle.height - nodeObserverSeparator.height
            model: nodeListModel
            spacing: 20
            width: parent.width

            ScrollBar.vertical: ScrollBar {
            }
            delegate: Component {
                id: childrenRect

                Rectangle {
                    color: Theme.panel.color.elementBackground
                    height: 100
                    radius: 10
                    width: nodeListView.width

                    TextArea {
                        id: label

                        color: Theme.font.color.primary
                        font.pointSize: 18
                        height: parent.height
                        horizontalAlignment: TextEdit.AlignHCenter
                        readOnly: true
                        text: name
                        verticalAlignment: TextEdit.AlignVCenter
                        width: parent.width
                    }
                    MouseArea {
                        acceptedButtons: Qt.LeftButton
                        anchors.fill: parent

                        onClicked: {
                            console.debug("Clicked to node " + name);
                        }
                    }
                }
            }
        }
    }
}
