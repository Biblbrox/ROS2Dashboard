import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import com.viz.types 1.0

Rectangle {
    id: visualizationWindow

    property var vizItems: {"dummy": 0}

    function addVisualizer(topicName, topicType) {
        if (visualizerModel.hasTopicViz(topicName)) {
            Logger.warn("Visualizer for topic " + topicName + " already exists");
            return;
        }
        let topicGroup = visualizerModel.getTopicCategory(topicType);
        if (topicGroup === "unknown") {
            Logger.error("The topic with type " + topicType + " is unknown");
            return;
        }
        let componentFile = "";
        if (topicGroup === "text")
            componentFile = "qrc:///ui/GenericTextVizComponent.qml";
        else if (topicGroup === "raster")
            componentFile = "qrc:///ui/RasterVizComponent.qml";
        else if (topicGroup === "geometry")
            componentFile = "qrc:///ui/GeometryVizComponent.qml";
        if (componentFile === "") {
            Logger.error("Unknown topic group " + topicGroup);
            return;
        }
        let vizItemComponent = Qt.createComponent("qrc:///ui/VizItemContainer.qml");
        if (vizItemComponent.status === Component.Ready) {
            let vizObject = vizItemComponent.createObject(vizContainer, {
                    "topicName": topicName,
                    "topicType": topicType,
                    "componentItemFile": componentFile,
                    "height": vizContainer.height,
                    "Layout.fillWidth": true
                });
            visualizationWindow.vizItems[topicName] = vizObject;
            Logger.debug("Added visualizer for topic " + topicName);
        } else {
            Logger.error(vizItemComponent.errorString());
        }
    }

    color: Theme.vizArea.color.background

    Component.onCompleted: {
        Logger.debug("Visualization area loaded");
    }

    Rectangle {
        id: delimeter

        color: "#3d3948"
        height: 30
        width: parent.width

        anchors {
            left: parent.left
            right: parent.right
            top: parent.top
        }
    }
    SplitView {
        id: splitView

        height: parent.height
        orientation: Qt.Horizontal
        width: parent.width

        anchors {
            bottom: parent.bottom
            left: parent.left
            right: parent.right
            top: delimeter.bottom
        }
        ListView {
            id: topicList

            Layout.fillHeight: true
            clip: true
            height: parent.height
            implicitWidth: 600
            model: topicListModel
            spacing: 10

            ScrollBar.vertical: ScrollBar {
            }
            delegate: Item {
                height: 70
                width: topicList.width

                Rectangle {
                    //clip: true
                    color: Theme.panel.color.elementBackground
                    height: parent.height
                    radius: 10
                    width: parent.width

                    TextArea {
                        id: topicName

                        anchors.bottom: parent.bottom
                        anchors.left: parent.left
                        anchors.right: addVizIcon.left
                        anchors.top: parent.top
                        color: Theme.font.color.primary
                        font.pixelSize: Theme.font.pointSize.normal
                        height: parent.height
                        horizontalAlignment: TextEdit.AlignHCenter
                        readOnly: true
                        text: name
                        verticalAlignment: TextEdit.AlignVCenter
                        width: parent.width - addVizIcon.width
                    }

                    RDButton {
                        iconSource: "Add"
                        width: 40
                        height: 40

                        id: addVizIcon

                        anchors.bottom: parent.bottom
                        anchors.right: parent.right
                        anchors.top: parent.top
                    }
                }
                MouseArea {
                    acceptedButtons: Qt.LeftButton
                    anchors.fill: parent

                    onClicked: {
                        Logger.debug("Clicked to element " + name);
                        addVisualizer(name, type);
                    }
                }
            }
        }
        SplitView {
            id: vizContainer

            SplitView.fillHeight: true
            SplitView.fillWidth: true
            height: parent.height
            orientation: Qt.Horizontal
        }
    }
    Connections {
        id: vizConnections
        function onTopicVizRemoved(topicName) {
            Logger.debug("vizContainer.children.length " + vizContainer.children.length);
            Logger.debug("vizItems.length " + visualizationWindow.vizItems.length);
            vizContainer.removeItem(visualizationWindow.vizItems[topicName]);
            Logger.debug("Topic " + topicName + " removed");
        }

        target: visualizerModel
    }
}
