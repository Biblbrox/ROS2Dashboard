import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.3

Rectangle {
    id: packageObserver

    color: Theme.panel.color.background
    visible: true
    width: 800

    ColumnLayout {
        id: columnLayout
        anchors.left: parent.left
        anchors.leftMargin: 20
        anchors.right: parent.right
        anchors.rightMargin: 20
        anchors.top: parent.top
        anchors.topMargin: 20
        anchors.bottom: parent.bottom
        spacing: 40

        Label {
            id: packageObserverTitle

            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            color: "white"
            font.pixelSize: 18
            text: qsTr("Package observer")
        }
        ToolSeparator {
            id: packageObserverSeparator

            Layout.fillWidth: true
            orientation: Qt.Horizontal
        }
        ListView {
            id: packageListView

            Layout.fillHeight: true
            Layout.fillWidth: true
            clip: true
            height: packageObserver.height - packageObserverTitle.height - packageObserverSeparator.height
            model: packageListModel
            spacing: 20
            width: parent.width

            ScrollBar.vertical: ScrollBar {
            }
            delegate: Rectangle {
                color: Theme.panel.color.elementBackground
                height: 100
                radius: 10
                width: packageListView.width

                TextArea {
                    id: label

                    anchors.centerIn: parent
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
                        console.debug("Clicked to package " + name);
                    }
                }
            }
        }
    }
}

