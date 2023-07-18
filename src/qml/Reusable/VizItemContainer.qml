import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import com.viz.types 1.0

Rectangle {
    id: vizItemContainer

    property string componentItemFile
    property string topicName
    property string topicType

    border.color: Theme.vizControls.color.background
    border.width: 3
    color: "transparent"

    Component.onCompleted: {
        Logger.debug("Loading vizualizer from " + componentItemFile + " for topicName: " + topicName);
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
        } else {
            Logger.error(component.errorString());
        }
    }

    Rectangle {
        id: topicNameLabel

        color: Theme.sideBar.color.background
        height: 50
        width: parent.width

        Label {
            //font.capitalization: Font.AllUppercase
            anchors.centerIn: parent
            font.pixelSize: Theme.font.pointSize.normal
            font.underline: true
            //width: parent.width
            //height: parent.height
            text: topicName
        }
        //anchors.horizontalCenter: parent.horizontalCenter
        anchors {
            //right: parent.right
            top: parent.top
            //left: parent.left
        }
    }
    RDControls {
        id: vizControls

        height: parent.height
        width: 50

        anchors {
            bottom: parent.bottom
            right: parent.right
            top: topicNameLabel.bottom
        }
    }
    Connections {
        function onVizControlClicked(action) {
            Logger.debug("VizControls clicked. Action: " + action + " Topic: " + topicName);
            if (action === "pause") {
                visualizerModel.pauseViz(topicName);
            } else if (action === "close") {
                visualizerModel.removeViz(topicName);
            } else if (action === "resume") {
                visualizerModel.resumeViz(topicName);
            } else if (action === "OpenNew") {
                visualizerModel.openFloatingWindow(componentItemFile, topicName, topicType);
            }
        }

        target: vizControls
    }
}