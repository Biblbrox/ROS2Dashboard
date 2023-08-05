import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import com.viz.types 1.0
import QuickQanava 2.0 as Qan

Qan.NodeItem {
    id: customNode

    property alias host: hostText.text
    property alias name: title.text
    property alias state: stateText.text

    // TODO: adapt to count of ports
    height: 180
    width: 160

    /*leftDock: Qan.VerticalDock {
        id: leftDockVert

        y: 100
    }
    rightDock: Qan.VerticalDock {
        id: rightDockVert

        y: 100
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
            NodeControls {
                id: nodeControls

                anchors {
                    right: parent.right
                    top: parent.top
                }
            }
            Text {
                id: stateText

                anchors.margins: 5
                //height: 60
                //width: 120
                color: Theme.node.color.titleFont
                font.pointSize: Theme.font.pointSize.nodeTitle

                anchors {
                    left: parent.left
                    top: parent.top
                }
            }
            Text {
                id: hostText

                anchors.margins: 5
                //height: 60
                //width: 120
                color: Theme.node.color.titleFont
                font.pointSize: Theme.font.pointSize.nodeTitle

                anchors {
                    left: parent.left
                    top: stateText.bottom
                }
            }
            Text {
                id: title

                anchors.centerIn: parent
                //anchors.fill: parent
                anchors.margins: 5
                color: Theme.font.color.primary
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
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