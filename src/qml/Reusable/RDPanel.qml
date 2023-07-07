import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: panel

    property var listModel
    property string title

    color: Theme.panel.color.background
    visible: true

    ColumnLayout {
        id: columnLayout

        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.leftMargin: 20
        anchors.right: parent.right
        anchors.rightMargin: 20
        anchors.top: parent.top
        anchors.topMargin: 20
        spacing: 40

        Label {
            id: panelTitle

            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            color: "white"
            font.pixelSize: 18
            text: title
        }
        ToolSeparator {
            id: panelSeparator

            Layout.fillWidth: true
            orientation: Qt.Horizontal
        }
        ListView {
            id: listView

            Layout.fillHeight: true
            Layout.fillWidth: true
            clip: true
            height: panel.height - panelTitle.height - panelSeparator.height
            model: listModel
            spacing: 20
            width: parent.width

            ScrollBar.vertical: ScrollBar {
            }
            delegate: Rectangle {
                color: Theme.panel.color.elementBackground
                height: 100
                radius: 10
                width: listView.width

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
                RDButton {
                    id: expandMore

                    anchors.bottom: parent.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    icon.color: Theme.font.color.primary
                    icon.height: 30
                    icon.source: "qrc:///ui/icons/ExpandMore.svg"
                    icon.width: 30

                    onClicked: {
                        hiddenContent.active = !hiddenContent.active;
                        hiddenContent.detailInfo = detailInfo;
                    }
                }
                Loader {
                    id: hiddenContent

                    //property alias detailInfo: sourceComponent.detailInfo.text

                    active: false

                    sourceComponent: Item {
                        id: hiddenContentComponent

                        //property string infoSource

                        Label {
                            id: detailInfo

                            anchors.fill: parent
                            text: infoSource
                        }
                    }
                }
                /*MouseArea {
                    acceptedButtons: Qt.LeftButton
                    anchors.fill: parent

                    onClicked: {
                        console.debug("Clicked to element " + name);
                    }
                }*/
            }
        }
    }
}

