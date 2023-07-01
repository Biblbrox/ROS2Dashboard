import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: sidebar

    property var folderModel

    signal sidebarClicked(string elementName)

    width: 120
    height: parent.height

    color: "#3e3949"

    Rectangle {
        id: borderRight

        width: 1
        height: parent.height
        anchors.right: parent.right
        color: "#000000"
        z: 2
    }

    ColumnLayout {
        width: parent.width
        spacing: 0

        anchors {
            left: parent.left
            right: parent.right
            top: parent.top
        }

        SidebarItem {

            iconSource: "qrc:///ui/icons/Launch.svg"
            handler: function() {
                sidebarClicked("Launch");
            }
        }

        SidebarItem {

            iconSource: "qrc:///ui/icons/Pause.svg"
            handler: function() {
                sidebarClicked("Pause");
            }
        }

        SidebarItem {

            iconSource: "qrc:///ui/icons/Save.svg"
            handler: function() {
                sidebarClicked("Save");
            }
        }

        SidebarItem {

            iconSource: "qrc:///ui/icons/Question.svg"
            handler: function() {
                sidebarClicked("Question");
            }
        }

    }

}
