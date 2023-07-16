import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: sidebar

    property var folderModel

    signal sidebarClicked(string elementName)

    color: Theme.sideBar.color.background
    height: parent.height
    width: 120

    Rectangle {
        id: borderRight

        anchors.right: parent.right
        color: "#000000"
        height: parent.height
        width: 1
        z: 2
    }
    ColumnLayout {
        id: sidebarLayout
        spacing: 0
        width: parent.width

        anchors {
            left: parent.left
            right: parent.right
            top: parent.top
        }
        SidebarItem {
            handler: function () {
                sidebarClicked("Launch");
            }
            iconSource: "Launch"
        }
        SidebarItem {
            handler: function () {
                sidebarClicked("Pause");
            }
            iconSource: "Pause"
        }
        SidebarItem {
            handler: function () {
                sidebarClicked("Save");
            }
            iconSource: "Save"
        }
        SidebarItem {
            handler: function () {
                sidebarClicked("Update");
            }
            iconSource: "Update"
        }
    }

    SidebarItem {
        anchors {
            left: parent.left
            right: parent.right
            bottom: parent.bottom
        }
        handler: function () {
            sidebarClicked("Question");
        }
        iconSource: "Question"
    }
}
