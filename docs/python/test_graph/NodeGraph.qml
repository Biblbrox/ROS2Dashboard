import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtQuick.Shapes 1.15
import QtQuick.Window 2.12

Window {
    id: sceneGraph

    visible: true
    width: 1920
    height: 1080

    CustomLine {
        id: customLine

        visible: false
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

    Node {
        id: rectangle2

        sceneWidth: sceneGraph.width
        color: "red"
        name: "Node name1"
        width: 150
        height: 80
        x: 1000
    }

    Node {
        id: rectangle1

        sceneWidth: sceneGraph.width
        color: "green"
        name: "Node name2"
        width: 150
        height: 80
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
