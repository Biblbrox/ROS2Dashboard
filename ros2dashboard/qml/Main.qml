import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtQuick.Shapes 1.15
import QtQuick.Window 2.12

ApplicationWindow {
    id: mainWindow

    visible: true
    width: 1920
    height: 1080

    SplitView {
        orientation: Qt.Vertical
        width: parent.width
        height: parent.height

        SplitView {
            width: parent.width
            //height: parent.height
            implicitHeight: 500
            //anchors.fill: parent
            orientation: Qt.Horizontal
            SplitView.fillWidth: true
            SplitView.fillHeight: true

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

        VisualizationWindow {
            SplitView.fillWidth: true
            implicitHeight: 400
            
        }

    }

    header: ToolBar {
        RowLayout {
            height: parent.height

            ToolButton {
                text: "Button1"
            }

            ToolButton {
                text: "Button2"
            }

            ToolButton {
                text: "Button3"
            }

            ToolButton {
                text: "Button4"
            }

        }

        background: Rectangle {
            width: parent.width
            height: parent.height
            color: "#3d3948"
        }

    }

}
