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

    visible: true
    width: 1920
    height: 1080
    title: qsTr("ROS2 Dashboard")
    Material.theme: Material.Dark

    Sidebar {
        id: sidebar

        width: 100
    }

    SplitView {
        orientation: Qt.Vertical
        width: parent.width - sidebar.width
        height: parent.height

        anchors {
            left: sidebar.right
            right: application.right
            top: application.top
        }

        SplitView {
            width: parent.width
            implicitHeight: 500
            orientation: Qt.Horizontal
            SplitView.fillWidth: true
            SplitView.fillHeight: true

            NodeObserver {
                height: parent.height
                implicitWidth: 400
                Layout.fillHeight: true
                z: 2
            }

            Qan.GraphView {
                resizeHandlerColor: Material.accent
                gridThickColor: Theme.background.color.primary
                height: parent.height
                implicitWidth: 1000
                Layout.fillHeight: true

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

                    selectionColor: Material.accent
                    connectorColor: Material.accent
                    connectorEdgeColor: Material.accent
                    objectName: "graph"
                    anchors.fill: parent
                    Component.onCompleted: {
                        createNodes();
                        createPorts();
                    }
                }

            }

            PackageObserver {
                height: parent.height
                implicitWidth: 400
                Layout.fillHeight: true
                z: 2
            }

        }

        VisualizationWindow {
            SplitView.fillWidth: true
            implicitHeight: 400
        }

    }

    Connections {
        function onSidebarClicked(itemName) {
            console.log("Clicked on ", itemName);
        }

        target: sidebar
    }

    header: ToolBar {
        RowLayout {
            height: parent.height

            ToolButton {
                text: "File"
            }

            ToolButton {
                text: "Help"
            }

        }

        background: Rectangle {
            width: parent.width
            height: parent.height
            color: "#3d3948"
        }

    }

}
