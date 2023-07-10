import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: vizItemContainer

    property string componentItemFile
    property string topicName
    property string topicType

    border.color: Theme.vizControls.color.background
    border.width: 3
    color: "transparent"

    Component.onCompleted: {
        var component = Qt.createComponent(componentItemFile);
        if (component.status === Component.Ready) {
            var vizItem = component.createObject(vizItemContainer, {
                    "topicName": topicName,
                    "topicType": topicType,
                    "height": vizItemContainer.height,
                    "width": vizItemContainer.width - vizControls.width,
                    "anchors.right": vizControls.left,
                    "anchors.left": vizItemContainer.left,
                    "anchors.bottom": vizItemContainer.bottom,
                    "anchors.top": topicNameLabel.bottom
                });
        }
    }

    Label {
        id: topicNameLabel

        anchors.horizontalCenter: parent.horizontalCenter
        font.pixelSize: Theme.font.pointSize.normal
        text: topicName
        //width: parent.width
        height: 50
        anchors {
            //right: parent.right
            top: parent.top
            //left: parent.left
        }
    }
    VizControls {
        id: vizControls

        height: parent.height
        width: 40

        anchors {
            bottom: parent.bottom
            right: parent.right
            top: topicNameLabel.top
        }
    }
}