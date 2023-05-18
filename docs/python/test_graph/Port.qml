import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtQuick.Shapes 1.15

Rectangle {
    /*MouseArea {
        id: mouseArea

        anchors.fill: parent
        propagateComposedEvents: true
        onClicked: {
            console.log("Port clicked");
            portClicked(input);
        }
    }*/

    id: port

    property bool input: true

    signal portClicked(bool isInput)

    width: 10
    height: width
    //color: "blue"
    border.color: "black"
    border.width: 1
    radius: width * 0.5
    color: mouse.hovered ? "black" : "red"

    HoverHandler {
        id: mouse

        enabled: true
        //acceptedDevices: PointerDevice.Mouse
        cursorShape: Qt.PointingHandCursor
    }

    TapHandler {
        acceptedPointerTypes: PointerDevice.GenericPointer | PointerDevice.Finger | PointerDevice.Pen
        onTapped: {
            console.log("clicked");
            portClicked(input);
        }
    }

}
