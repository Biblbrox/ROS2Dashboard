import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtQuick.Shapes 1.15
import QtQuick.Window 2.12

Rectangle {
    //width: 1920
    //height: 1080

    id: sceneGraph

    /**
    True if user clicked to some output port node. Then he will point this line to input port
    of the another node. At this moment isNodeConnectingState will be turned to false
     */
    property bool isNodeConnectingState
    property variant connectorLines: []
    
    function addConnection(sourcePort, destPort) {
        component = Qt.createComponent("ConnectorLine.qml");
        item = component.createObject(null, {
            "sourceX": sourcePort.x,
            "sourceY": sourcePort.y,
            "targetX": destPort.x,
            "targetY": destPort.y
        });
        connectorLines.push(item);
    }

    color: "transparent"
    visible: true

    Item {
        id: modelSize

        property real originalWidth: 200
        property real originalHeight: 100

        height: 100
        width: 200
    }

    Flow {
        /*Repeater {
            model: nodeGraph
        }*/

        id: flow

        anchors.fill: parent

        Node {
            sceneWidth: sceneGraph.width
            color: "red"
            name: "Node name1"
            width: modelSize.width
            height: modelSize.height
            x: 1000
            onOutputPortClicked: function(topicName) {
                console.log("Output port clicked: topicName", topicName);
            }
        }

        Node {
            sceneWidth: sceneGraph.width
            color: "green"
            name: "Node name2"
            width: modelSize.width
            height: modelSize.height
            x: 500
            onOutputPortClicked: function(topicName) {
                console.log("Output port clicked: topicName", topicName);
            }
        }

    }

    MouseArea {
        /*if (zoomArea.scale === targetScale)
                    zoomArea.scale *= scaleFactor;
                else
                    zoomArea.scale = targetScale;
                zoomArea.scale = Math.min(maxScale, Math.max(minScale, zoomArea.scale));*/

        property real lastMouseX: 0
        property real lastMouseY: 0

        preventStealing: true
        anchors.fill: parent
        width: parent.width
        z: -1
        height: parent.height
        onWheel: function(wheel) {
            if (wheel.angleDelta.y < 0) {
                let scaleStep = 1;
                grid.gridSize += 1;
                modelSize.width += scaleStep;
                modelSize.height += scaleStep;
            } else {
                let scaleStep = 1;
                grid.gridSize -= 1;
                modelSize.width -= scaleStep;
                modelSize.height -= scaleStep;
            }
        }
        onPressed: {
            lastMouseX = mouseX;
            lastMouseY = mouseY;
        }
        onPositionChanged: {
            let deltaX = mouseX - lastMouseX;
            let deltaY = mouseY - lastMouseY;
            sceneGraph.x += deltaX;
            sceneGraph.y += deltaY;
            lastMouseX = mouseX;
            lastMouseY = mouseY;
        }
    }

}
