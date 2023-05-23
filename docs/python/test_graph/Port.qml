import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtQuick.Shapes 1.15

Rectangle {
    id: port

    property bool input: true
    property string topicName

    signal portClicked(bool isInput, string topicName)

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
            port.portClicked(port.input, port.topicName);
        }
    }

}
