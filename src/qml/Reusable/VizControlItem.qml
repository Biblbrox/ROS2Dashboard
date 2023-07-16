import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: vizControlItem
    property string iconSource
    property var userCallback: function() {}

    RDIcon {
        id: rdIcon
        source: vizControlItem.iconSource
    }

    RDButton {
        id: btn
        icon.source: rdIcon.getSource()

        onClicked: {
            vizControlItem.userCallback();
        }
    }
}