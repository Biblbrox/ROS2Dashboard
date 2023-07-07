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

            RDPanel {
                id: nodeObserver
                Layout.fillHeight: true
                height: parent.height
                listModel: nodeListModel
                title: qsTr("Node observer")
                implicitWidth: 400
                width: 800
                z: 2
            }
            Qan.GraphView {
                Layout.fillHeight: true
                gridThickColor: Theme.background.color.primary
                height: parent.height
                implicitWidth: 1000
                resizeHandlerColor: Material.accent

                graph: Qan.Graph {
                    id: graphObj

                    function createNodes() {
                        let nodeComponent = Qt.createComponent("qrc:/ui/Node.qml");
                        let nodes = {};
                        /// Create nodes
                        for (let i = 0; i < nodeListModel.rowCount(); ++i) {
                            let node = graphObj.insertNode(nodeComponent);
                            node.item.name = nodeListModel.getRow(i, "name");
                            console.log("Create node " + node.item.name);
                            node.item.x = 50 + i * 10;
                            node.item.y = 50;
                            node["inPorts"] = {};
                            node["outPorts"] = {};

                            /// Add output ports
                            for (let j = 0; j < nodeListModel.getRow(i, "publishers").length; ++j) {
                                let port = graphObj.insertPort(node, Qan.NodeItem.Right);
                                port.label = nodeListModel.getRow(i, "publishers")[j];
                                node.outPorts[port.label] = port;
                            }

                            /// Add input ports
                            for (let j = 0; j < nodeListModel.getRow(i, "subscribers").length; ++j) {
                                let port = graphObj.insertPort(node, Qan.NodeItem.Left);
                                port.label = nodeListModel.getRow(i, "subscribers")[j];
                                node.inPorts[port.label] = port;
                            }
                            nodes[node.item.name] = node;
                        }

                        /// Create connections
                        for (let i = 0; i < connectionListModel.rowCount(); ++i) {
                            let outNodeName = connectionListModel.getRow(i, "src_node_name");
                            let inNodeName = connectionListModel.getRow(i, "dst_node_name");
                            let topicName = connectionListModel.getRow(i, "topic_name");
                            console.debug("Create connection from " + outNodeName + " and " + inNodeName + " nodes");
                            let outNode = nodes[outNodeName];
                            let inNode = nodes[inNodeName];
                            let outPort = outNode.outPorts[topicName];
                            let inPort = inNode.inPorts[topicName];

                            // Connect them with edge
                            let edge = graphObj.insertEdge(inNode, outNode);
                            defaultEdgeStyle.lineType = Qan.EdgeStyle.Curved;
                            graphObj.bindEdgeSource(edge, outPort);
                            graphObj.bindEdgeDestination(edge, inPort);
                        }
                    }

                    anchors.fill: parent
                    connectorColor: Theme.node.color.edge
                    connectorEdgeColor: Theme.node.color.edge
                    connectorEnabled: true
                    objectName: "graph"
                    selectionColor: Material.accent

                    Component.onCompleted: {
                        console.log("Creating nodes");
                        createNodes();
                        defaultEdgeStyle.lineWidth = 3;
                        defaultEdgeStyle.lineColor = Qt.binding(function () {
                                return Theme.node.color.edge;
                            });
                    }
                }
            }
            RDPanel {
                id: packageObserver
                Layout.fillHeight: true
                SplitView.fillWidth: true
                height: parent.height
                implicitWidth: 400
                listModel: packageListModel
                width: 800
                title: qsTr("Package observer")
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
