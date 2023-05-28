import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtQuick.Shapes 1.15

Rectangle {
    /* Rectangle {
        anchors.fill: parent
        anchors.rightMargin: rightRadius ? -parent.radiusRect : 0
        anchors.leftMargin: leftRadius ? -parent.radiusRect : 0
        anchors.bottomMargin: bottomRadius ? -parent.radiusRect : 0
        anchors.topMargin: topRadius ? -parent.radiusRect : 0
        radius: parent.radiusRect
        opacity: 0.5


    }*/

    id: clipper

    property bool topRadius: false
    property bool leftRadius: false
    property bool rightRadius: false
    property bool bottomRadius: false
    property real radiusRect: 0
    property alias colorEnch: clipped.color

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
