/*
Params per group
 */
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Shapes
import QtQuick.Window


Rectangle {
    id: settings

    property var params: []

    Component.onCompleted: {
        let paramsPerGroup = settingsModel.getGroupParams(currentGroup);

        for (let i = 0; i < paramsPerGroup.length; i++) {
            settingsPerGroup
        }
    }

    SplitView {
        id: splitView
        width: parent.width
        height: parent.height

        ListView {
            id: settingsGroups
            //width: parent.width
            SplitView.preferredWidth: 200
            height: parent.height

            model: settingsModel

            delegate: Rectangle {
                color: settingsGroups.currentIndex === index ? "lightsteelblue" : "white"
                height: 40
                width: settingsGroups.width

                RowLayout {
                    Text {
                        text: group
                    }
                }

                MouseArea {
                    anchors.fill: parent

                    onClicked: {
                        settingsGroups.currentIndex = index;
                        settingsWindow.currentGroup = group;
                    }
                }
            }
        }


        ColumnLayout {
            id: settingsPerGroup
            SplitView.preferredWidth: parent.width - settingsGroups.width
            height: parent.height

            RowLayout {
                width: parent.width
                height: parent.height
                Text {
                    text: name
                }

                Text {
                    text: value
                }
            }
        }
    }
}
