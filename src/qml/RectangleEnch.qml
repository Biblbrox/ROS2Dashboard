import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Shapes 1.15

Rectangle {
    id: clipper

    property bool topRadius: false
    property bool leftRadius: false
    property bool rightRadius: false
    property bool bottomRadius: false
    property real radiusRect: 0

    clip: true
    color: "transparent"

    Rectangle {
        id: clipped

        width: parent.width
        height: parent.height + radius
        anchors.topMargin: -radius
        radius: radiusRect
    }

}
