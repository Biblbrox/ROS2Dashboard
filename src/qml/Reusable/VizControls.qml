import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: vizControlContainer

    signal vizControlClicked(string actionName)

    color: Theme.vizControls.color.background

    ColumnLayout {
        id: controlsLayout

        width: parent.width
        spacing: 20

        VizControlItem {
            id: playPauseControl

            Layout.fillWidth: true
            Layout.alignment: Qt.AlignHCenter
            iconSource: "qrc:/ui/icons/Pause.svg"
            userCallback: function () {
                vizControlClicked("stop");
            }
        }
    }
}