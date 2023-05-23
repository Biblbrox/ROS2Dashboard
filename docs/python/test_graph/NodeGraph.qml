import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtQuick.Shapes 1.15
import QtQuick.Window 2.12

Rectangle {
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
    
    visible: true
    //width: 1920
    //height: 1080

    Flow {
        //anchors.fill: parent

        Repeater {
            model: nodeGraph

            delegate: Node {
                sceneWidth: sceneGraph.width
                color: "red"
                name: "Node name1"
                width: 200
                height: 100
                x: 1000
                onOutputPortClicked: function(topicName) {
                    console.log("Output port clicked: topicName", topicName);
                }
            }

        }

    }

    Item {
        id: zoomArea

        width: parent.width
        height: parent.height

        GridBackground {
            id: grid

            anchors.fill: parent
        }

    }

    MouseArea {
        property real minScale: 0.2
        property real maxScale: 3

        anchors.fill: parent
        width: parent.width
        z: -1
        height: parent.height
        onWheel: {
            /*if (zoomArea.scale === targetScale)
                    zoomArea.scale *= scaleFactor;
                else
                    zoomArea.scale = targetScale;
                zoomArea.scale = Math.min(maxScale, Math.max(minScale, zoomArea.scale));*/
            console.log("Mouse event", wheel.angleDelta);
            if (wheel.angleDelta.y < 0)
                grid.gridSize += 1;
            else
                grid.gridSize -= 1;
        }
    }

}
