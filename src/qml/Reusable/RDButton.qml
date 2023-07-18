import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Button {
    id: button

    property string backgroundColor: ""
    property string iconSource

    height: parent.height
    hoverEnabled: true
    icon.source: icon.getSource()
    width: parent.width

    background: Rectangle {
        id: backgroundRect

        color: function colorHandler() {
            if (button.down) {
                return Theme.button.color.selected;
            } else if (button.hovered) {
                return Theme.button.color.hovered;
            } else {
                if (backgroundColor != "")
                    return backgroundColor;
                else
                    return Theme.button.color.primary;
            }
        }()
        height: parent.height
        radius: 2
        width: parent.width
    }

    /*MouseArea {
        id: mouseArea

        anchors.fill: parent
        hoverEnabled: true
        propagateComposedEvents: true

        onEntered: {
            colorAnimation.to = Theme.button.color.primary;
            colorAnimation.start();
        }
        onExited: {
            colorAnimation.to = Theme.button.color.selected;
            colorAnimation.start();
        }
    }
    PropertyAnimation {
        id: colorAnimation

        duration: 300
        property: "color"
        target: backgroundRect
    }*/
    RDIcon {
        id: icon

        source: iconSource
    }
}

