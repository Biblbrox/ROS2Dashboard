import "../../../ros2dashboard/qml"
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtQuick.Shapes 1.15
import QtQuick.Window 2.12

Window {
    id: mainWindow

    visible: true
    width: 1920
    height: 1080

    SplitView {
        width: parent.width
        height: parent.height
        anchors.fill: parent
        orientation: Qt.Horizontal

        NodeObserver {
            height: parent.height
            implicitWidth: 400
            Layout.fillHeight: true
            z: 2            
        }

        NodeGraph {
            id: nodeGraph

            implicitWidth: 600
            SplitView.fillWidth: true
            //anchors.left: parent.left
            //height: parent.height
            SplitView.fillHeight: true
            z: 1
        }

        PackageObserver {
            height: parent.height
            implicitWidth: 400
            Layout.fillHeight: true
            z: 2
        }

        background: GridBackground {
            id: grid
            //z: 1
            anchors.fill: parent
        }

    }

}
