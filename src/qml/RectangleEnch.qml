import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Shapes

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
