import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: vizControlItem
    property alias iconSource: btn.icon.source
    property var userCallback: function() {}

    RDButton {
        id: btn

        onClicked: {
            vizControlItem.userCallback();
        }
    }
}