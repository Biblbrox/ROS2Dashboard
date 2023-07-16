import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import com.viz.types 1.0

Rectangle {
    id: vizControlContainer

    // Orientation property
    property int orientation: Qt.Vertical

    signal vizControlClicked(string actionName)

    color: Theme.vizControls.color.background

    GridLayout {
        id: controlsLayout

        columnSpacing: orientation === Qt.Horizontal ? 20 : 0
        columns: orientation === Qt.Horizontal ? 3 : 1
        rowSpacing: orientation === Qt.Horizontal ? 0 : 20
        rows: orientation === Qt.Horizontal ? 1 : 3
        width: parent.width

        VizControlItem {
            id: resumeControl

            Layout.alignment: Qt.AlignHCenter
            Layout.fillWidth: true
            height: 60
            iconSource: "Launch"
            userCallback: function () {
                Logger.debug("resumeControl clicked");
                vizControlClicked("resume");
            }
        }
        VizControlItem {
            id: playPauseControl

            Layout.alignment: Qt.AlignHCenter
            Layout.fillWidth: true
            height: 60
            iconSource: "Pause"
            userCallback: function () {
                Logger.debug("playPauseControl clicked");
                vizControlClicked("pause");
            }
        }
        VizControlItem {
            id: closeControl

            Layout.alignment: Qt.AlignHCenter
            Layout.fillWidth: true
            height: 60
            iconSource: "Close"
            userCallback: function () {
                Logger.debug("closeControl clicked");
                vizControlClicked("close");
            }
        }

        VizControlItem {
            id: openInNew

            Layout.alignment: Qt.AlignHCenter
            Layout.fillWidth: true
            height: 60
            iconSource: "OpenNew"
            userCallback: function () {
                Logger.debug("openNew clicked");
                vizControlClicked("OpenNew");
            }
        }
    }
}

