import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Shapes
import QtQuick.Window

//import com.viz.types 1.0

Window {
    id: settingsWindow

    SplitView {
        id: splitView

        ListView {
            id: settingsList

            model: settingsModel

            delegate: Rectangle {
                color: settingsList.currentIndex === index ? "lightsteelblue" : "white"
                height: 40
                width: settingsList.width

                Text {
                    anchors.centerIn: parent
                    text: parameter
                }
                MouseArea {
                    anchors.fill: parent

                    onClicked: settingsList.currentIndex = index
                }
            }
        }
    }
}
