import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: vizControlItem
    property string iconSource
    property var userCallback: function () {
    }
    property string tooltipText: ""

    RDIcon {
        id: rdIcon
        source: vizControlItem.iconSource
    }

    RDButton {
        id: btn
        icon.source: rdIcon.getSource()

        ToolTip.delay: Theme.tooltip.delay
        ToolTip.timeout: Theme.tooltip.timeout
        ToolTip.visible: hovered
        ToolTip.text: tooltipText

        onClicked: {
            vizControlItem.userCallback();
        }
    }
}