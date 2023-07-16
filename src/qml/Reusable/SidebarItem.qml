import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: sidebarItem

    property var handler
    property alias iconSource: btn.iconSource
    property string name: ""

    color: "transparent"
    height: 100
    width: parent.width

    RDButton {
        id: btn

        anchors.horizontalCenter: parent.horizontalCenter
        height: parent.height
        icon.height: height
        icon.width: width
        padding: 20
        width: parent.width
        onClicked: {
            //anim.start();
            handler();
        }/*

        SequentialAnimation {
            id: anim

            // Expand the button
            PropertyAnimation {
                duration: 200
                easing.type: Easing.InOutQuad
                property: "scale"
                target: btn
                to: 1.2
            }

            // Shrink back to normal
            PropertyAnimation {
                duration: 200
                easing.type: Easing.InOutQuad
                property: "scale"
                target: btn
                to: 1
            }

        }

        palette {
            button: sidebarItem.color
        }

        background: Rectangle {
            color: sidebarItem.color
        }*/

    }

}
