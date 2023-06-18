import QtQuick 2.0
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.15

Rectangle {
    id: sidebarItem

    property var handler
    property string name: ""
    property alias iconSource: btn.icon.source

    width: parent.width
    height: 100
    color: "#3e3949"

    Button {
        id: btn
        anchors.horizontalCenter: parent.horizontalCenter
        padding: 20

        width: parent.width
        height: parent.height

        icon.width: width
        icon.height: height
        onClicked: {
            console.log("adasd");
            anim.start();
            handler();
        }

        SequentialAnimation {
            id: anim

            // Expand the button
            PropertyAnimation {
                target: btn
                property: "scale"
                to: 1.2
                duration: 200
                easing.type: Easing.InOutQuad
            }

            // Shrink back to normal
            PropertyAnimation {
                target: btn
                property: "scale"
                to: 1
                duration: 200
                easing.type: Easing.InOutQuad
            }

        }

        palette {
            button: sidebarItem.color
        }

        background: Rectangle {
            color: sidebarItem.color
        }

    }

}
