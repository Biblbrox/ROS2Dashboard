import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QuickQanava 2.0 as Qan

Qan.NodeItem {
    id: customNode

    width: 160
    height: 180
    x: 150
    y: 15

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

            Text {
                anchors.fill: parent
                anchors.centerIn: parent
                anchors.margins: 5
                text: "Custom Node"
                color: Theme.node.color.titleFont
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }

        }
        // Body

        Rectangle {
            id: body

            color: Theme.node.color.body
            height: customNode.height - header.height

            anchors {
                left: parent.left
                right: parent.right
                top: header.bottom
                bottom: parent.bottom
            }

        }

    }

}