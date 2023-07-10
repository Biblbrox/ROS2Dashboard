import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QuickQanava 2.0 as Qan

Qan.NodeItem {
    id: customNode

    property alias name: title.text

    // TODO: adapt to count of ports
    height: 180
    width: 160
    x: 150
    y: 15

    /*leftDock: Qan.VerticalDock {
        id: leftDockVert
        Label {
            Layout.preferredWidth: height
            font.bold: true
            rotation: -90
            text: "Custom Dock"
        }
    }*/

    /*rightDock: Qan.VerticalDock {
        id: rightDockVert

    }*/

    Item {
        anchors.fill: parent

        Rectangle {
            id: header

            color: Theme.node.color.title
            height: 60
            width: parent.width

            anchors {
                left: parent.left
                right: parent.right
                top: parent.top
            }
            RDButton {
                id: killButton

                icon.color: "transparent"
                icon.source: "qrc:///ui/icons/Close.svg"
                width: 50

                onClicked: {
                    console.debug("Kill node " + title.text);
                    daemonClientModel.killNode(title.text);
                }

                anchors {
                    right: parent.right
                    top: parent.top
                }
            }
            Text {
                id: title

                anchors.centerIn: parent
                anchors.fill: parent
                anchors.margins: 5
                color: Theme.node.color.titleFont
                horizontalAlignment: Text.AlignHCenter
                //text: "Custom Node"
                verticalAlignment: Text.AlignVCenter
            }
        }
        // Body

        Rectangle {
            id: body

            color: Theme.node.color.body
            height: customNode.height - header.height

            anchors {
                bottom: parent.bottom
                left: parent.left
                right: parent.right
                top: header.bottom
            }
        }
    }
}