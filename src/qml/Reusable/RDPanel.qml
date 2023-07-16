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
            color: Theme.font.color.primary
            elide: Text.ElideRight
            font.underline: true
            font.capitalization: Font.AllUppercase
            font.pixelSize: Theme.font.pointSize.normal
            text: title
            width: parent.width
            wrapMode: Label.WordWrap
        }
        Rectangle {
            id: panelSeparator

            Layout.fillWidth: true
            //orientation: Qt.Horizontal
            width: parent.width
            height: 2
            color: Theme.font.color.primary
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
                id: elementItem

                color: Theme.panel.color.elementBackground
                height: 100
                radius: 10
                width: listView.width

                ColumnLayout {
                    id: elementLayout

                    anchors.fill: parent

                    TextEdit {
                        id: header

                        Layout.alignment: Qt.AlignHCenter
                        Layout.fillWidth: true
                        color: Theme.font.color.primary
                        font.pixelSize: Theme.font.pointSize.normal
                        horizontalAlignment: TextEdit.AlignHCenter
                        readOnly: true
                        selectByMouse: true
                        text: headerMetrics.elidedText
                        verticalAlignment: TextEdit.AlignVCenter
                        wrapMode: Text.ElideRight

                        TextMetrics {
                            id: headerMetrics

                            elide: Text.ElideRight
                            elideWidth: header.width - 10
                            font: header.font
                            text: name
                        }
                    }
                    RDButton {
                        id: expandMore

                        Layout.alignment: Qt.AlignHCenter
                        Layout.fillWidth: true
                        icon.color: Theme.font.color.primary
                        icon.height: 30
                        icon.source: "qrc:///ui/icons/ExpandMore.svg"
                        icon.width: 30

                        onClicked: {
                            detailInfo.visible = !detailInfo.visible;
                            detailInfoMetrics.text = detail_info;
                            if (detailInfo.visible) {
                                elementItem.height += detailInfo.height;
                            } else {
                                elementItem.height -= detailInfo.height;
                            }
                        }
                    }
                    TextArea {
                        id: detailInfo

                        Layout.alignment: Qt.AlignHCenter
                        Layout.fillWidth: true
                        selectByMouse: true
                        text: detailInfoMetrics.elidedText
                        visible: false
                        wrapMode: Text.ElideRight

                        TextMetrics {
                            id: detailInfoMetrics

                            //elide: Text.ElideRight
                            elideWidth: elementLayout.width - 5
                            font: detailInfo.font
                        }
                    }
                }
            }
        }
    }
}

