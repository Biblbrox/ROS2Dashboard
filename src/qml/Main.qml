import "." as Qan
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Shapes
import QtQuick.Window
import com.viz.types 1.0
import QuickQanava 2.0 as Qan
import "qrc:/QuickQanava" as Qan

ApplicationWindow {
    id: application

    color: Theme.background.color.field
    height: 1080
    title: qsTr("ROS2 Dashboard")
    visible: true
    width: 1920

    header: ToolBar {
        background: Rectangle {
            color: Theme.sideBar.color.background
            height: parent.height
            width: parent.width
        }

        RowLayout {
            height: parent.height

            ToolButton {
                id: fileButton

                contentItem: Label {
                    color: Theme.font.color.primary
                    text: qsTr("File")
                }
            }
            ToolButton {
                id: helpButton

                contentItem: Label {
                    color: Theme.font.color.primary
                    text: qsTr("Help")
                }
            }
        }
    }

    Sidebar {
        id: sidebar

        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.top: parent.top
        width: 100
        z: 2
    }
    SplitView {
        height: parent.height
        orientation: Qt.Vertical
        width: parent.width - sidebar.width// - rightSidebar.width

        anchors {
            left: sidebar.right
            right: parent.right
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
                implicitWidth: 400
                listModel: nodeListModel
                title: qsTr("Node observer")
                width: 800
                z: 2
            }
            Qan.GraphView {
                id: graphView

                property int portHeight: 16
                property int portWidth: 16

                Layout.fillHeight: true
                gridThickColor: Theme.background.color.field
                height: parent.height
                implicitWidth: 1000
                resizeHandlerColor: Theme.background.color.primary

                graph: Qan.Graph {
                    id: graphObj

                    function updateNodes() {
                        graphObj.clearGraph();
                        let nodeComponent = Qt.createComponent("qrc:/ui/Node.qml");
                        let nodes = {};
                        /// Create nodes
                        for (let i = 0; i < nodeListModel.rowCount(); ++i) {
                            let node = graphObj.insertNode(nodeComponent);
                            node.item.state = nodeListModel.getRow(i, "state");
                            node.item.name = nodeListModel.getRow(i, "name");
                            node.item.host = nodeListModel.getRow(i, "host_ip");
                            Logger.debug("Create node " + node.item.name + " with state " + node.item.state);
                            node.item.x = 50 + i * 10;
                            node.item.y = 50;

                            /// Add output ports
                            let outPorts = {};
                            let inPorts = {};
                            for (let j = 0; j < nodeListModel.getRow(i, "publishers").length; ++j) {
                                let port = graphObj.insertPort(node, Qan.NodeItem.Right, Qan.PortItem.Out, nodeListModel.getRow(i, "publishers")[j]);
                                port.setPortType("out");
                                port.setPortLabel(nodeListModel.getRow(i, "publishers")[j]);
                                outPorts[port.label] = port;
                            }

                            /// Add input ports
                            for (let j = 0; j < nodeListModel.getRow(i, "subscribers").length; ++j) {
                                let port = graphObj.insertPort(node, Qan.NodeItem.Left, Qan.PortItem.In, nodeListModel.getRow(i, "subscribers")[j]);
                                port.setPortType("in");
                                port.setPortLabel(nodeListModel.getRow(i, "subscribers")[j]);
                                inPorts[port.label] = port;
                            }
                            let maxPortCount = Math.max(Object.keys(inPorts).length, Object.keys(outPorts).length);
                            node.item.width = 500;
                            node.item.height = maxPortCount * 50;
                            node.inPorts = inPorts;
                            node.outPorts = outPorts;
                            nodes[node.item.name] = node;
                        }

                        /// Create connections
                        for (let i = 0; i < connectionListModel.rowCount(); ++i) {
                            let outNodeName = connectionListModel.getRow(i, "src_node_name");
                            let inNodeName = connectionListModel.getRow(i, "dst_node_name");
                            let topicName = connectionListModel.getRow(i, "topic_name");
                            Logger.debug("Create connection from " + outNodeName + " and " + inNodeName + " nodes");
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

                    //anchors.fill: graphView
                    connectorColor: Theme.node.color.edge
                    connectorEdgeColor: Theme.node.color.edge
                    connectorEnabled: true
                    objectName: "graph"
                    selectionColor: Theme.background.color.secondary

                    portDelegate: Component {
                        id: youpi

                        Qan.PortItem {
                            id: port

                            property string portLabel
                            property string portType

                            function setPortLabel(label_) {
                                port.portLabel = label_;
                            }
                            function setPortType(type_) {
                                port.portType = type_;
                            }

                            height: graphView.portHeight

                            RowLayout {
                                anchors.fill: parent
                                layoutDirection: port.portType === "in" ? Qt.LeftToRight : Qt.RightToLeft

                                Image {
                                    Layout.fillHeight: true
                                    source: port.portType === "in" ? "qrc:///ui/icons/InputSquare.svg" : "qrc:///ui/icons/OutputSquare.svg"
                                    width: graphView.portWidth
                                }
                                Label {
                                    text: port.portLabel
                                }
                            }
                        }
                    }

                    Component.onCompleted: {
                        Logger.debug("Creating nodes");
                        updateNodes();
                        defaultEdgeStyle.lineWidth = 3;
                        defaultEdgeStyle.lineColor = Qt.binding(function () {
                                return Theme.node.color.edge;
                            });
                    }
                }
                grid: Qan.AbstractLineGrid {
                }

                /*TapHandler {
                    acceptedButtons: Qt.LeftButton | Qt.RightButton
                    target: graphView

                    onSingleTapped: {
                        console.log("onSingleTapped called...");
                        graphView.grabToImage(function (result) {
                                minimap.source = result.url;
                            });
                    }
                    onTapped: {
                        console.log("onTapped called...");
                    }
                }*/

                //z: -30

                /*Rectangle {
                    anchors.margins: 10
                    height: 200
                    width: 200
                    color: Theme.background.color.field
                    opacity: 0.5
                    border.color: Theme.background.color.primary

                    anchors {
                        right: parent.right
                        top: parent.top
                    }
                    Image {
                        id: minimap

                        height: parent.height
                        width: parent.width
                    }
                }*/
            }
            RDPanel {
                id: packageObserver

                Layout.fillHeight: true
                //SplitView.fillWidth: true
                height: parent.height
                implicitWidth: 400
                listModel: packageListModel
                title: qsTr("Package observer")
                width: 800
                z: 2
            }
        }
        VisualizationWindow {
            SplitView.fillWidth: true
            implicitHeight: 400
        }
    }
    /*Sidebar {
        id: rightSidebar

        anchors.bottom: parent.bottom
        anchors.right: parent.right
        anchors.top: parent.top
        width: 100
        z: 2
    }*/
    Connections {
        function onSidebarClicked(itemName) {
            Logger.debug("Clicked on " + itemName);
            if (itemName === "Update") {
                // Make DaemonClient update request
                Logger.debug("Update request");
                daemonClientModel.update();
                graphObj.updateNodes();
            } else if (itemName === "Start")
            // Make DaemonClient start request
            {
            }
        }

        target: sidebar
    }
}
