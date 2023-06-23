import "." as Qan
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Shapes
import QtQuick.Window
import QtQuick.Controls.Material
import QuickQanava 2.0 as Qan
import "qrc:/QuickQanava" as Qan

ApplicationWindow {
    id: application

    Material.theme: Material.Dark
    height: 1080
    title: qsTr("ROS2 Dashboard")
    visible: true
    width: 1920

    header: ToolBar {
        background: Rectangle {
            color: "#3d3948"
            height: parent.height
            width: parent.width
        }

        RowLayout {
            height: parent.height

            ToolButton {
                text: "File"
            }
            ToolButton {
                text: "Help"
            }
        }
    }


    Sidebar {
        id: sidebar

        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.top: parent.top
        width: 100
    }
    SplitView {
        height: parent.height
        orientation: Qt.Vertical
        width: parent.width - sidebar.width - rightSidebar.width

        anchors {
            left: sidebar.right
            right: rightSidebar.left
            top: application.top
        }
        SplitView {
            SplitView.fillHeight: true
            SplitView.fillWidth: true
            implicitHeight: 500
            orientation: Qt.Horizontal
            width: parent.width

            NodeObserver {
                Layout.fillHeight: true
                height: parent.height
                implicitWidth: 400
                z: 2
            }
            Qan.GraphView {
                Layout.fillHeight: true
                gridThickColor: Theme.background.color.primary
                height: parent.height
                implicitWidth: 1000
                resizeHandlerColor: Material.accent

                graph: Qan.Graph {
                    id: graph

                    function createNodes() {
                        var nodeComponent = Qt.createComponent("qrc:/ui/Node.qml");
                        for (var i = 0; i < nodeListModel.rowCount(); ++i) {
                            var node = graph.insertNode(nodeComponent);
                            node.item.name = nodeListModel.getRow(i, "name");
                            node.item.x = 50 + i * 10;
                            node.item.y = 50;
                        }
                    }
                    function createPorts() {
                        for (var i = 0; i < connectionListModel.rowCount(); ++i) {
                            console.log("*******");
                        }
                    }

                    anchors.fill: parent
                    connectorColor: Material.accent
                    connectorEdgeColor: Material.accent
                    objectName: "graph"
                    selectionColor: Material.accent

                    Component.onCompleted: {
                        console.log("Creating nodes");
                        createNodes();
                        createPorts();
                    }
                }
            }
            PackageObserver {
                Layout.fillHeight: true
                height: parent.height
                implicitWidth: 400
                z: 2
            }
        }
        VisualizationWindow {
            SplitView.fillWidth: true
            implicitHeight: 400
        }
    }
    Sidebar {
        id: rightSidebar

        anchors.bottom: parent.bottom
        anchors.right: parent.right
        anchors.top: parent.top
        width: 100
    }
    Connections {
        function onSidebarClicked(itemName) {
            console.log("Clicked on ", itemName);
        }

        target: sidebar
    }
}
