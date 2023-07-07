import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import com.viz.types 1.0

Rectangle {
    id: visualizationWindow

    function addVisualizer(topicName, topicType) {
        if (visualizerModel.hasTopicViz(topicName)) {
            console.log("Visualizer for topic " + topicName + " already exists");
            return;
        }
        let topicGroup = visualizerModel.getTopicCategory(topicType);
        if (topicGroup === "unknown") {
            console.error("The topic with type " + topicType + " is unknown");
            return;
        }
        let componentFile = "";
        if (topicGroup === "text") {
            componentFile = "qrc:///ui/GenericTextVizComponent.qml";
        } else if (topicGroup === "raster") {
            componentFile = "qrc:///ui/VideoVizComponent.qml";
        }
        if (componentFile === "")
            return;
        let vizComponent = Qt.createComponent(componentFile);
        if (vizComponent.status === Component.Ready) {
            let vizObject = vizComponent.createObject(vizContainer, {
                    "topicName": topicName,
                    "topicType": topicType,
                    //"width": width,
                    "height": vizContainer.height,
                    "Layout.fillWidth": true
                });
            console.log("Addedd visualizer for topic " + topicName);
        }
    }

    color: "#2f2e40"

    Component.onCompleted: {
        console.log("Visualization area loaded");
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
                        font.pointSize: 18
                        height: parent.height
                        horizontalAlignment: TextEdit.AlignHCenter
                        readOnly: true
                        text: name
                        verticalAlignment: TextEdit.AlignVCenter
                        width: parent.width - addVizIcon.width
                    }
                    Image {
                        id: addVizIcon

                        anchors.bottom: parent.bottom
                        anchors.right: parent.right
                        anchors.top: parent.top
                        source: "qrc:///ui/icons/Add.svg"
                        //width: 20
                    }
                }
                MouseArea {
                    acceptedButtons: Qt.LeftButton
                    anchors.fill: parent

                    onClicked: {
                        console.debug("Clicked to element " + name);
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
}
